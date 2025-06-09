use crate::{futures::TimerDelay, pins::pyro::*};
use bmi323::{Bmi323, interface::SpiInterface};
use bno080::{interface::SensorInterface, wrapper::BNO080};
use core::sync::atomic::AtomicU32;
use core::{cell::Cell, convert::Infallible, f32::consts::PI, fmt::Write};
use cortex_m::interrupt::Mutex;
use derive_more::{From, Into};
use embassy_futures::block_on;
use embedded_hal::digital::StatefulOutputPin;
use embedded_hal::{delay::DelayNs, digital::OutputPin, i2c::I2c, spi::SpiDevice};
use fugit::{Duration, ExtU32, RateExtU32 as _};
use futures::join;
use heapless::{String, Vec};
use icm20948_driver::icm20948::{NoDmp, i2c::IcmImu};
use nmea0183::{GGA, ParseResult, datetime::Time};
use serde::{Deserialize, Serialize};
use stm32f4xx_hal::{
    ClearFlags,
    adc::{Adc, config::SampleTime},
    pac::{ADC1, TIM12},
    rcc::Clocks,
    timer::{self, Counter, Instance},
};
use storage_types::logs::AccelerometerSample;
use storage_types::{
    CONFIG_KEYS, ConfigKey, MissionStage, PyroPin, Role, ValueType,
    logs::{GPSSample, IMUSample, LocalCtxt, MessageType, PressureTempSample, RadioCtxt},
};
use thingbuf::mpsc::{StaticChannel, StaticReceiver};

use crate::{
    BUZZER, BUZZER_TIMER, PYRO_TIMER, RTC,
    altimeter::{ALTIMETER_FRAME_COUNT, FifoFrames},
    futures::{NbFuture, YieldFuture},
    gps,
    logs::get_logger,
    pins::i2c::Altimeter,
    radio::{self, RECEIVED_MESSAGE_QUEUE},
    usb_logger::get_serial,
};

pub static mut ROLE: Role = Role::Cansat;
pub static mut PYRO_ADC: Option<Adc<ADC1>> = None;
pub static mut PYRO_CONT1: Option<PyroCont1> = None;
pub static mut PYRO_CONT2: Option<PyroCont2> = None;
pub static mut PYRO_ENABLE_PIN: Option<PyroEnable> = None;
pub static mut PYRO_FIRE2: Option<PyroFire2> = None;
pub static mut PYRO_FIRE1: Option<PyroFire1> = None;

pub static DETECTED_APOGEE: AtomicU32 = AtomicU32::new(0);

static STAGE: Mutex<Cell<MissionStage>> = Mutex::new(Cell::new(MissionStage::Armed));

pub fn role() -> Role {
    // SAFETY: Role is mutated once by main prior to mission begin.
    unsafe { ROLE }
}

pub async fn update_pyro_state() {
    // SAFETY: This is the only place we use these static muts
    //         and as we are not async, we can guarantee that
    //         we are not accessing them concurrently.
    let mv = unsafe {
        // Need to disarm pyro to read voltage correctly
        let pyro_enable = PYRO_ENABLE_PIN.as_mut().unwrap();
        let previous = pyro_enable.is_set_high().unwrap();

        pyro_enable.set_low();
        let pyro = PYRO_CONT1.as_mut().unwrap();
        let adc = PYRO_ADC.as_mut().unwrap();
        let sample = {
            #[cfg(feature = "target-ultra")]
            {
                adc.convert(pyro, SampleTime::Cycles_56)
            }
            #[cfg(not(feature = "target-ultra"))]
            {
                0
            }
        };

        if previous {
            pyro_enable.set_high();
        }
        adc.sample_to_millivolts(sample)
    };

    // todo!("set pyro state");
    // FIXME: unfortunate. we should be able to submit this to the executor to avoid blocking.
    block_on(get_logger().log(MessageType::new_pyro(mv).into_message(current_rtc_time())));
}

pub async fn stage_update_handler(channel: StaticReceiver<FifoFrames>) {
    let mut start_altitude: f32 = 0.0;
    let mut sea_level_pressure: f32 = 0.0;
    const MAIN_DEPLOYMENT_HEIGHT: f32 = 300.0;

    loop {
        let frames = channel.recv().await;
        if let None = frames {
            YieldFuture::new().await;
            continue;
        }
        let frames = frames.unwrap();
        if frames.len() < 5 {
            YieldFuture::new().await;
            continue;
        }
        if sea_level_pressure == 0.0 {
            let mut vec: Vec<f32, ALTIMETER_FRAME_COUNT> = frames
                .iter()
                .map(|frame| frame.pressure)
                .into_iter()
                .collect();
            vec.sort_unstable_by(f32::total_cmp);
            // Take the average of the middle 5 values
            let mid = vec.len() / 2;
            sea_level_pressure = vec[mid - 2..mid + 3].iter().sum::<f32>() / 5.0;
        }

        let stage = current_stage();
        let altitudes: Vec<f32, ALTIMETER_FRAME_COUNT> = frames
            .iter()
            .map(|frame| {
                let altitude = (libm::powf(sea_level_pressure / frame.pressure, 1.0 / 5.257) - 1.0)
                    * (frame.temperature + 273.15)
                    / 0.0065;
                altitude
            })
            .collect();

        if start_altitude == 0.0 {
            // If we don't have a start pressure, we can't do anything
            let mut vec: Vec<f32, ALTIMETER_FRAME_COUNT> = altitudes.clone().into_iter().collect();
            vec.sort_unstable_by(f32::total_cmp);
            start_altitude = vec[7..12].iter().sum::<f32>() / 5.0;
        }

        match stage {
            MissionStage::Disarmed => {
                /* Nothing to update, just wait for arm command */
                cortex_m::interrupt::free(|cs| STAGE.borrow(cs).set(MissionStage::Disarmed));
            }
            MissionStage::Armed => {
                // If we're above 10m, we must be ascending
                if altitudes
                    .iter()
                    .filter(|a| **a > 15.0 + start_altitude)
                    .count()
                    > 12
                {
                    get_logger().log_str("stage,detected ascent!").await;
                    cortex_m::interrupt::free(|cs| STAGE.borrow(cs).set(MissionStage::Ascent));
                }
            }
            MissionStage::Ascent => {
                let max_altitude = altitudes.iter().cloned().fold(0.0, f32::max);
                if max_altitude as u32 > DETECTED_APOGEE.load(core::sync::atomic::Ordering::Relaxed)
                {
                    DETECTED_APOGEE
                        .store(max_altitude as u32, core::sync::atomic::Ordering::Relaxed);
                };

                // If our velocity is negative, we must be descending
                let velocities = altitudes.windows(2).map(|w| w[1] - w[0]);

                if velocities.filter(|v| *v < 0.015).count() > 10 {
                    get_logger().log_str("stage,detected apogee").await;
                    if role() == Role::Avionics {
                        get_logger().log_str("stage,firing drogue!").await;
                        // If we're the avionics, fire the pyro
                        fire_pyro(PyroPin::One, 1000).await;
                    }
                    cortex_m::interrupt::free(|cs| {
                        STAGE.borrow(cs).set(MissionStage::DescentDrogue)
                    });
                }
            }
            MissionStage::DescentDrogue => {
                // At 300m, fire main
                if altitudes
                    .iter()
                    .filter(|a| **a < MAIN_DEPLOYMENT_HEIGHT)
                    .count()
                    > 12
                {
                    get_logger().log_str("stage,detected main").await;
                    if role() == Role::Avionics {
                        get_logger().log_str("stage,firing main!").await;
                        // If we're the avionics, fire the pyro
                        fire_pyro(PyroPin::Two, 1000).await;
                    }
                    cortex_m::interrupt::free(|cs| STAGE.borrow(cs).set(MissionStage::DescentMain));
                }
            }
            MissionStage::DescentMain => {
                // If our average velocity is less than 1m/s, we must have landed
                let velocities_abs = altitudes.windows(2).map(|w| libm::fabsf(w[1] - w[0]));
                if velocities_abs.clone().filter(|x| *x < 0.02).count() > 12 {
                    get_logger()
                        .log_str("stage,detected entering landed stage")
                        .await;
                    if role() == Role::Cansat {
                        get_logger().log_str("stage,landed! firing pyro2!").await;
                        fire_pyro(PyroPin::Two, 7000).await;
                    }

                    cortex_m::interrupt::free(|cs| STAGE.borrow(cs).set(MissionStage::Landed));
                }
            }
            MissionStage::Landed => {
                // Nothing to do
            }
        };

        update_pyro_state().await;
        YieldFuture::new().await;
    }
}

pub fn current_stage() -> MissionStage {
    cortex_m::interrupt::free(|cs| STAGE.borrow(cs).get())
}

fn parse_role(s: &[u8]) -> Option<Role> {
    if s.starts_with(b"cansat") {
        Some(Role::Cansat)
    } else if s.starts_with(b"avionics") {
        Some(Role::Avionics)
    } else if s.starts_with(b"ground") {
        Some(Role::GroundMain)
    } else {
        None
    }
}

pub fn radio_ctxt() -> RadioCtxt {
    RadioCtxt {
        source: role(),
        stage: current_stage(),
        counter: radio::next_counter(),
    }
}

pub async fn usb_handler() -> ! {
    let mut buf = [0u8; 256];
    loop {
        let Ok(b) = get_serial().read_no_block(&mut buf) else {
            YieldFuture::new().await;
            continue;
        };
        let bytes = &buf[..b];

        let split: Vec<_, 32> = bytes.split(|b| *b == b',').collect();
        if split.len() > 1 && split[0].starts_with(b"disarm") {
            match parse_role(split[1]) {
                Some(role) => {
                    cortex_m::interrupt::free(|cs| {
                        STAGE.borrow(cs).set(MissionStage::Disarmed);
                    });
                    radio::queue_packet(MessageType::new_disarm(role).into_message(radio_ctxt()));
                }
                None => writeln!(get_serial(), "Invalid disarm role").unwrap(),
            }
        } else if split.len() > 1 && split[0].starts_with(b"arm") {
            match parse_role(split[1]) {
                Some(role) => {
                    radio::queue_packet(MessageType::new_arm(role).into_message(radio_ctxt()));
                    writeln!(get_serial(), "Sent arm packet!").unwrap();
                }
                None => writeln!(get_serial(), "Invalid arm role").unwrap(),
            }
        } else if split.len() > 2 && split[0].starts_with(b"fire") {
            if let Some(role) = parse_role(split[1]) {
                let pin = if split[2].starts_with(b"1") {
                    PyroPin::One
                } else if split[2].starts_with(b"2") {
                    PyroPin::Two
                } else if split[2].starts_with(b"both") {
                    PyroPin::Both
                } else {
                    writeln!(get_serial(), "Invalid test-pyro pin").unwrap();
                    continue;
                };

                radio::queue_packet(
                    MessageType::new_test_pyro(
                        role,
                        pin,
                        core::str::from_utf8(split[3])
                            .unwrap()
                            .trim()
                            .parse()
                            .unwrap(),
                    )
                    .into_message(radio_ctxt()),
                );
            } else {
                writeln!(get_serial(), "Invalid test-pyro role").unwrap();
            }
        } else if split.len() > 1 && split[0].starts_with(b"launch") {
            let new_state = if split[1].starts_with(b"true") {
                MissionStage::Ascent
            } else {
                MissionStage::Disarmed
            };

            cortex_m::interrupt::free(|cs| STAGE.borrow(cs).set(new_state));
        } else if split.len() > 1 && split[0].starts_with(b"setstage") {
            let role = match parse_role(split[1]) {
                Some(r) => r,
                None => continue,
            };

            let stage = if split[2].starts_with(b"ascent") {
                MissionStage::Ascent
            } else if split[1].starts_with(b"descent-drogue") {
                MissionStage::DescentDrogue
            } else if split[2].starts_with(b"descent-main") {
                MissionStage::DescentMain
            } else if split[2].starts_with(b"landed") {
                MissionStage::Landed
            } else {
                writeln!(get_serial(), "Invalid stage").unwrap();
                continue;
            };

            radio::queue_packet(MessageType::new_set_stage(role, stage).into_message(radio_ctxt()));
        } else if split[0].starts_with(b"ping") {
            writeln!(get_serial(), "pong").unwrap();
        } else if split[0].starts_with(b"uarm") {
            arm().await;
            writeln!(get_serial(), "armed").unwrap();
        } else if split[0].starts_with(b"ufire") {
            let pin = if split[2].starts_with(b"1") {
                PyroPin::One
            } else if split[2].starts_with(b"2") {
                PyroPin::Two
            } else if split[2].starts_with(b"both") {
                PyroPin::Both
            } else {
                writeln!(get_serial(), "Invalid test-pyro pin").unwrap();
                continue;
            };

            fire_pyro(pin, 100u32).await;
            writeln!(get_serial(), "fired").unwrap();
        } else if split.len() >= 3 && split[0].starts_with(b"config") {
            let key = core::str::from_utf8(split[1]).unwrap().trim();
            let value = core::str::from_utf8(split[2]).unwrap().trim();

            // if CONFIG_KEYS.iter().map(|(k, _)| *k).any(|k| k != key) {
            //     writeln!(get_serial(), "err, invalid config key {}", key).unwrap();
            //     continue;
            // }

            let value = match CONFIG_KEYS.iter().find(|(k, _)| *k == key).unwrap().1 {
                ValueType::U64 => value.parse::<u64>().unwrap(),
            };
            let key = ConfigKey::try_from(key).unwrap();

            get_logger().edit_config(&key, value).await;
            get_serial().log("ok\n").await;
        } else if split.len() >= 1 && split[0].starts_with(b"erase") {
            if let Err(msg) = get_logger().erase_logs().await {
                writeln!(get_serial(), "err, {}", msg).unwrap();
            }
            get_serial().log("ok\n").await;
        } else if split.len() >= 1 && split[0].starts_with(b"free") {
            let free = get_logger().space_left().await;
            if let Ok(free) = free {
                writeln!(
                    get_serial(),
                    "free,{:.02}%",
                    free as f32 / (16.0 * 1024.0 * 1024.0) * 100.0
                )
                .unwrap();
            } else {
                writeln!(get_serial(), "err, {}", free.unwrap_err()).unwrap();
            }
        } else if split.len() >= 1 && split[0].starts_with(b"logs") {
            if let Err(msg) = get_logger()
                .get_logs(async |s| match &s.message {
                    MessageType::Log { timestamp, message } => {
                        writeln!(
                            get_serial(),
                            "[{}] [Local log] {message}",
                            Into::<String<12>>::into(EpochTime(*timestamp))
                        )
                        .unwrap();
                    }
                    MessageType::Accelerometer(accel_compressed) => {
                        for sample in accel_compressed.decompress() {
                            if let Ok(AccelerometerSample {
                                timestamp,
                                acceleration,
                            }) = sample
                            {
                                writeln!(
                                    get_serial(),
                                    "[{}] [Local acc] {:?}",
                                    Into::<String<12>>::into(EpochTime(timestamp)),
                                    acceleration
                                )
                                .unwrap();
                            }
                        }
                    }
                    MessageType::Gps(gps_compressed) => {
                        for sample in gps_compressed.decompress() {
                            if let Ok(GPSSample {
                                timestamp,
                                latitude,
                                longitude,
                                altitude,
                            }) = sample
                            {
                                writeln!(
                                    get_serial(),
                                    "[{}] [Local gps] {latitude},{longitude},{altitude}",
                                    Into::<String<12>>::into(EpochTime(timestamp))
                                )
                                .unwrap();
                            }
                        }
                    }
                    MessageType::PressureTemp(pressure_temp_compressed) => {
                        for sample in pressure_temp_compressed.decompress() {
                            if let Ok(PressureTempSample {
                                timestamp,
                                pressure,
                                temperature,
                            }) = sample
                            {
                                writeln!(
                                    get_serial(),
                                    "[{}] [Local prt] {pressure},{temperature}",
                                    Into::<String<12>>::into(EpochTime(timestamp))
                                )
                                .unwrap();
                            }
                        }
                    }
                    MessageType::Imu(imu_compressed) => {
                        for sample in imu_compressed.decompress() {
                            if let Ok(IMUSample {
                                acceleration,
                                angular_velocity,
                                timestamp,
                            }) = sample
                            {
                                writeln!(
                                    get_serial(),
                                    "[{}] [Local imu] {:?},{:?}",
                                    Into::<String<12>>::into(EpochTime(timestamp)),
                                    acceleration,
                                    angular_velocity
                                )
                                .unwrap();
                            }
                        }
                    }
                    MessageType::Arm(role) => {
                        writeln!(
                            get_serial(),
                            "[{}] [Local log] Arm command from {role:?}",
                            current_rtc_time::<String<12>>()
                        )
                        .unwrap();
                    }
                    MessageType::Disarm(role) => {
                        writeln!(
                            get_serial(),
                            "[{}] [Local log] Disarm command from {role:?}",
                            current_rtc_time::<String<12>>()
                        )
                        .unwrap();
                    }
                    MessageType::TestPyro(role, pyro_pin, _) => {
                        writeln!(
                            get_serial(),
                            "[{}] [Local log] Test pyro command to {role:?} on pin {pyro_pin:?}",
                            current_rtc_time::<String<12>>()
                        )
                        .unwrap();
                    }
                    MessageType::SetStage(role, mission_stage) => {
                        writeln!(
                            get_serial(),
                            "[{}] [Local log] Set stage command to {role:?} to {mission_stage:?}",
                            current_rtc_time::<String<12>>()
                        )
                        .unwrap();
                    }
                    MessageType::Pyro(mv) => {
                        writeln!(
                            get_serial(),
                            "[{}] [Local pyro] {mv}",
                            current_rtc_time::<String<12>>()
                        )
                        .unwrap();
                    }
                })
                .await
            {
                writeln!(get_serial(), "err, {}", msg).unwrap();
            }
        } else {
            writeln!(get_serial(), "Invalid command").unwrap();
        }

        // writeln!(get_serial(), "Received: {:?}", core::str::from_utf8(bytes)).unwrap();
        YieldFuture::new().await;
    }
}

async fn gps_handler() -> ! {
    gps::poll_for_sentences().await
}

pub type RTCTime = (u8, u8, u8, u16);
#[derive(Debug, Serialize, Deserialize, PartialEq, Eq, Clone, Copy, From, Into)]
#[repr(transparent)]
pub struct EpochTime(u32);

impl From<EpochTime> for String<12> {
    fn from(value: EpochTime) -> Self {
        let mut s = String::new();
        let millis = value.0 % 1000;
        let seconds = (value.0 / 1000) % 60;
        let minutes = (value.0 / 60_000) % 60;
        let hours = (value.0 / 3_600_000) % 24;

        write!(s, "{hours:02}:{minutes:02}:{seconds:02}.{millis:03}").unwrap();
        s
    }
}

pub fn current_rtc_time<T: From<EpochTime>>() -> T {
    cortex_m::interrupt::free(|cs| {
        let mut rtc_ref = RTC.borrow(cs).borrow_mut();
        let rtc = rtc_ref.as_mut().unwrap();
        let date_time = rtc.get_datetime();
        EpochTime::from(date_time.as_hms_milli()).into()
    })
}

impl From<EpochTime> for LocalCtxt {
    fn from(value: EpochTime) -> Self {
        LocalCtxt { timestamp: value.0 }
    }
}

impl From<Time> for EpochTime {
    fn from(time: Time) -> Self {
        EpochTime(
            time.hours as u32 * 3_600_000
                + time.minutes as u32 * 60_000
                + time.seconds as u32 * 1000
                + ((time.seconds % 1.0) * 1000.0) as u32,
        )
    }
}

impl From<RTCTime> for EpochTime {
    fn from(time: RTCTime) -> Self {
        EpochTime(
            time.0 as u32 * 3_600_000
                + time.1 as u32 * 60_000
                + time.2 as u32 * 1000
                + time.3 as u32,
        )
    }
}

async fn gps_broadcast() -> ! {
    loop {
        let mut samples = [GPSSample::default(); 80];

        for sample in samples.iter_mut() {
            loop {
                let fix = gps::next_sentence().await;
                if let ParseResult::GGA(Some(GGA {
                    fix: Some(gga),
                    time,
                    ..
                })) = fix
                {
                    *sample = GPSSample {
                        timestamp: EpochTime::from(time).0,
                        latitude: gga.latitude.as_f64() as f32,
                        longitude: gga.longitude.as_f64() as f32,
                        altitude: gga.altitude.meters as f32,
                    };
                    break;
                }
            }
        }
        let radio_msg = MessageType::new_gps(samples.iter().cloned().enumerate().filter_map(
            |(i, sample)| {
                if i % 2 == 0 { Some(sample) } else { None }
            },
        ))
        .into_message(radio_ctxt());

        get_logger()
            .log(
                MessageType::new_gps(samples)
                    .clone()
                    .into_message(current_rtc_time::<LocalCtxt>()),
            )
            .await;

        radio::queue_packet(radio_msg)
    }
}

#[cfg(not(any(feature = "target-ultra", feature = "ultra-dev")))]
async fn bmp_altimeter_handler(
    mut sensor: Altimeter,
    mut timer: Counter<TIM12, 10000>,
    pressure_sender: thingbuf::mpsc::StaticSender<FifoFrames>,
) {
    // Wait for FIFO to fill up.

    timer.clear_flags(timer::Flag::Update);
    timer.start(550.millis()).unwrap();
    NbFuture::new(|| timer.wait()).await.unwrap();
    loop {
        const NTH: usize = 4;
        let mut samples = [PressureTempSample::default(); 40 * NTH];

        for sample in samples.iter_mut() {
            let (frames, returned_sensor) = crate::altimeter::read_altimeter_fifo(sensor).await;
            sensor = returned_sensor;
            for frame in &frames {
                let pressure = frame.pressure;
                let temperature = frame.temperature;

                *sample = PressureTempSample {
                    timestamp: current_rtc_time(),
                    pressure,
                    temperature,
                };
            }
            pressure_sender
                .send(Vec::from_slice(&frames).unwrap())
                .await
                .unwrap();
            timer.clear_flags(timer::Flag::Update);
            timer.start(400.millis()).unwrap();
            // FIXME: Use interrupt to wait for FIFO to fill up.
            NbFuture::new(|| timer.wait()).await.unwrap();
        }

        let radio_msg = MessageType::new_pressure_temp(samples.into_iter().enumerate().filter_map(
            |(i, sample)| {
                if i % NTH == 0 { Some(sample) } else { None }
            },
        ))
        .into_message(radio_ctxt());

        get_logger()
            .log(
                MessageType::new_pressure_temp(samples)
                    .clone()
                    .into_message(current_rtc_time()),
            )
            .await;
        radio::queue_packet(radio_msg);
    }
}

async fn handle_incoming_packets() -> ! {
    loop {
        if let Some(packet) = cortex_m::interrupt::free(|cs| {
            RECEIVED_MESSAGE_QUEUE.borrow(cs).borrow_mut().pop_front()
        }) {
            let RadioCtxt {
                source,
                counter: _counter,
                stage: _stage,
            } = packet.context;
            match role() {
                Role::GroundMain | Role::GroundBackup => match packet.message {
                    MessageType::Log { timestamp, message } => {
                        writeln!(
                            get_serial(),
                            "[{}] [{source:?} log] {message}",
                            Into::<String<12>>::into(EpochTime(timestamp))
                        )
                        .unwrap();
                    }
                    MessageType::Accelerometer(bit_buffer) => {
                        for sample in bit_buffer.decompress() {
                            if let Ok(AccelerometerSample {
                                timestamp,
                                acceleration,
                            }) = sample
                            {
                                writeln!(
                                    get_serial(),
                                    "[{}] [{source:?} acc] {:?}",
                                    Into::<String<12>>::into(EpochTime(timestamp)),
                                    acceleration
                                )
                                .unwrap();
                            }
                        }
                    }
                    MessageType::Gps(bit_buffer) => {
                        for sample in bit_buffer.decompress() {
                            if let Ok(GPSSample {
                                timestamp,
                                latitude,
                                longitude,
                                altitude,
                            }) = sample
                            {
                                writeln!(
                                    get_serial(),
                                    "[{}] [{source:?} gps] {latitude},{longitude},{altitude}",
                                    Into::<String<12>>::into(EpochTime(timestamp))
                                )
                                .unwrap();
                            }
                        }
                    }
                    MessageType::PressureTemp(bit_buffer) => {
                        for sample in bit_buffer.decompress() {
                            if let Ok(PressureTempSample {
                                timestamp,
                                pressure,
                                temperature,
                            }) = sample
                            {
                                writeln!(
                                    get_serial(),
                                    "[{}] [{source:?} prt] {pressure},{temperature}",
                                    Into::<String<12>>::into(EpochTime(timestamp))
                                )
                                .unwrap();
                            }
                        }
                    }
                    MessageType::Imu(bit_buffer) => {
                        for sample in bit_buffer.decompress() {
                            if let Ok(IMUSample {
                                acceleration,
                                angular_velocity,
                                timestamp,
                            }) = sample
                            {
                                writeln!(
                                    get_serial(),
                                    "[{}] [{source:?} imu] {:?},{:?}",
                                    Into::<String<12>>::into(EpochTime(timestamp)),
                                    acceleration,
                                    angular_velocity
                                )
                                .unwrap();
                            }
                        }
                    }
                    MessageType::Arm(_) => {
                        writeln!(
                            get_serial(),
                            "[{}] [{:?} log] Out of turn arm command from {source:?}",
                            <EpochTime as Into<String<12>>>::into(current_rtc_time()),
                            role()
                        )
                        .unwrap();
                    }
                    MessageType::Disarm(role) => {
                        writeln!(
                            get_serial(),
                            "[{}] [{:?} log] Out of turn disarm command from {source:?}",
                            <EpochTime as Into<String<12>>>::into(current_rtc_time()),
                            role
                        )
                        .unwrap();
                    }
                    MessageType::TestPyro(role, pyro_pin, _) => {
                        writeln!(
                                            get_serial(),
                                            "[{}] [{:?} log] Out of turn test pyro command from {source:?} on pin {pyro_pin:?}",
                                            <EpochTime as Into<String<12>>>::into(current_rtc_time()),
                                            role
                                        )
                                        .unwrap();
                    }
                    MessageType::SetStage(role, mission_stage) => {
                        writeln!(
                                            get_serial(),
                                            "[{}] [{:?} log] Out of turn set stage command from {source:?} to {mission_stage:?}",
                                            <EpochTime as Into<String<12>>>::into(current_rtc_time()),
                                            role
                                        )
                                        .unwrap();
                    }
                    MessageType::Pyro(mv) => {
                        writeln!(
                            get_serial(),
                            "[{}] [{source:?} pyro] {mv}",
                            current_rtc_time::<String<12>>()
                        )
                        .unwrap();
                    }
                },
                _ => match packet.message {
                    MessageType::Arm(r) if role() == r => {
                        arm().await;
                    }
                    MessageType::Disarm(r) if role() == r => {
                        disarm().await;
                    }
                    MessageType::TestPyro(r, pyro_pin, duration) if r == role() => {
                        fire_pyro(pyro_pin, duration).await;
                    }
                    _ => {}
                },
            }
        }

        YieldFuture::new().await;
    }
}

#[allow(unreachable_code)]
pub async fn fire_pyro(pin: PyroPin, duration: u32) {
    #[cfg(feature = "target-mini")]
    {
        let _ = pin;
        let _ = duration;
        panic!("Pyro board is modified. Mini cannot fire pyro.");
    }

    let buzzer = unsafe { BUZZER.as_mut().unwrap() };
    let timer = unsafe { PYRO_TIMER.as_mut().unwrap() };

    let mut pyro_fires = {
        let mut vec: Vec<&mut dyn OutputPin<Error = Infallible>, 2> = Vec::new();
        let _ = match pin {
            PyroPin::One => {
                vec.push(unsafe { PYRO_FIRE1.as_mut().unwrap() })
                    .map_err(|_| "Pyro fire 1 pin not initialized")
                    .unwrap();
            }
            PyroPin::Two => {
                vec.push(unsafe { PYRO_FIRE2.as_mut().unwrap() })
                    .map_err(|_| "Pyro fire 2 pin not initialized")
                    .unwrap();
            }
            PyroPin::Both => {
                vec.push(unsafe { PYRO_FIRE1.as_mut().unwrap() })
                    .map_err(|_| "Pyro fire 1 pin not initialized")
                    .unwrap();
                vec.push(unsafe { PYRO_FIRE2.as_mut().unwrap() })
                    .map_err(|_| "Pyro fire 2 pin not initialized")
                    .unwrap();
            }
        };
        vec
    };

    let mut time = 200i32;
    while time > 0 {
        buzz(200u32.millis()).await;
        buzzer.set_low();
        timer.start((time as u32).millis()).unwrap();
        NbFuture::new(|| timer.wait()).await.unwrap();
        time = (time as f32 * 0.8 - 5.0) as i32;
    }

    buzz(200u32.millis()).await;

    timer.start(500u32.millis()).unwrap();
    NbFuture::new(|| timer.wait()).await.unwrap();
    buzzer.set_low();
    for fire in &mut pyro_fires {
        fire.set_high().unwrap();
    }
    timer.start(duration.millis()).unwrap();
    NbFuture::new(|| timer.wait()).await.unwrap();
    for fire in &mut pyro_fires {
        fire.set_low().unwrap();
    }
}

pub async fn disarm() {
    let buzz = unsafe { BUZZER.as_mut().unwrap() };
    let timer = unsafe { BUZZER_TIMER.as_mut().unwrap() };
    let pyro_enable = unsafe { PYRO_ENABLE_PIN.as_mut().unwrap() };
    buzz.set_high();
    pyro_enable.set_low();
    cortex_m::interrupt::free(|cs| {
        STAGE.borrow(cs).replace(MissionStage::Disarmed);
    });
    update_pyro_state().await;

    timer.clear_flags(timer::Flag::Update);
    timer.start(300u32.millis()).unwrap();
    NbFuture::new(|| timer.wait()).await.unwrap();
    buzz.set_low();
}

#[allow(unreachable_code)]
pub async fn arm() {
    #[cfg(feature = "target-mini")]
    {
        panic!("Pyro board is modified. Mini cannot be armed.");
    }

    let buzz = unsafe { BUZZER.as_mut().unwrap() };
    let timer = unsafe { BUZZER_TIMER.as_mut().unwrap() };
    let pyro_enable = unsafe { PYRO_ENABLE_PIN.as_mut().unwrap() };
    buzz.set_high();

    pyro_enable.set_high();
    cortex_m::interrupt::free(|cs| {
        STAGE.borrow(cs).replace(MissionStage::Armed);
    });
    update_pyro_state().await;
    timer.clear_flags(timer::Flag::Update);
    timer.start(300u32.millis()).unwrap();
    NbFuture::new(|| timer.wait()).await.unwrap();
    buzz.set_low()
}

pub async fn buzz(duration: Duration<u32, 1, 10000>) {
    let buzz = unsafe { BUZZER.as_mut().unwrap() };
    let timer = unsafe { BUZZER_TIMER.as_mut().unwrap() };

    // hack: the buzzer pin is not hooked up to a PWM channel, so we have to do this shittily
    for i in 0..(2 * (duration / 250u32.micros::<1, 10000>())) {
        if i % 2 == 0 {
            buzz.set_high();
        } else {
            buzz.set_low();
        }
        timer.start(250u32.micros()).unwrap();
        NbFuture::new(|| timer.wait()).await.unwrap();
    }
    buzz.set_low();
}

pub async fn buzz_number(number: u32) {
    let buzzer = unsafe { BUZZER.as_mut().unwrap() };
    let timer = unsafe { BUZZER_TIMER.as_mut().unwrap() };

    let short = 100u32.millis();
    let long = 1000u32.millis();

    let mut number = number;
    let digits = core::iter::from_fn(|| {
        if number == 0 {
            None
        } else {
            let digit = number % 10;
            number /= 10;
            Some(digit as u8)
        }
    })
    .collect::<Vec<_, 10>>(); // 10 = log10(u32::MAX) + 1 (I wish it were a const fn)

    for &digit in digits.iter().rev() {
        for _ in 0..digit {
            buzz(short).await;
            buzzer.set_low();
            timer.start(short).unwrap();
            NbFuture::new(|| timer.wait()).await.unwrap();
        }
        timer.start(long).unwrap();
        NbFuture::new(|| timer.wait()).await.unwrap();
    }
}

async fn buzzer_controller() -> ! {
    // Buzz 1s on startup
    {
        buzz(1000u32.millis()).await;
        let buzzer = unsafe { BUZZER.as_mut().unwrap() };
        buzzer.set_low();
    }

    while !matches!(current_stage(), MissionStage::Landed) {
        YieldFuture::new().await;
    }

    // Buzz on landing
    loop {
        buzz_number(DETECTED_APOGEE.load(core::sync::atomic::Ordering::Relaxed)).await;
        let timer = unsafe { BUZZER_TIMER.as_mut().unwrap() };
        timer.start(3000u32.millis()).unwrap();
        NbFuture::new(|| timer.wait()).await.unwrap();
    }
}

#[cfg(any(feature = "target-ultra", feature = "ultra-dev"))]
async fn adxl_imu_handler(
    mut imu: adxl375::spi::ADXL375<impl SpiDevice<u8>, impl embedded_hal_async::delay::DelayNs>,
) {
    use storage_types::logs::AccelerometerSample;

    loop {
        let mut samples = [AccelerometerSample::default(); 32];

        for (store, sample) in samples.iter_mut().zip(imu.read_fifo().await.unwrap()) {
            store.timestamp = current_rtc_time();
            store.acceleration = sample;
        }

        let message = MessageType::new_accel(samples.into_iter());
        let radio_msg = message.clone().into_message(radio_ctxt());
        if role() == Role::Avionics {
            radio::queue_packet(radio_msg);
        }

        get_logger()
            .log(message.into_message(current_rtc_time()))
            .await;
        embedded_hal_async::delay::DelayNs::delay_ms(&mut imu, 32 * 10).await;
    }
}

async fn icm_imu_handler(
    mut imu: IcmImu<impl I2c, NoDmp>,
    mut imu_timer: impl embedded_hal_async::delay::DelayNs,
) -> ! {
    let mut samples = [IMUSample::default(); 20];

    loop {
        for sample in samples.iter_mut() {
            let (acc_data, gyro_data) = loop {
                if let Ok(x) = imu.read_acc().and_then(|a| imu.read_gyro().map(|g| (a, g))) {
                    break x;
                };
                YieldFuture::new().await;
            };

            *sample = IMUSample {
                timestamp: current_rtc_time(),
                acceleration: acc_data,
                angular_velocity: gyro_data,
            };
            imu_timer.delay_ms(10).await;
        }

        let message = MessageType::new_imu(samples.into_iter());
        if role() == Role::Avionics {
            radio::queue_packet(message.clone().into_message(radio_ctxt()));
        }

        get_logger()
            .log(message.into_message(current_rtc_time()))
            .await;
    }
}

/// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_code_2
#[allow(unused)]
fn quaternion_to_euler(quat: [f32; 4]) -> [f32; 3] {
    let [w, x, y, z] = quat;
    let mut angles = [0.0f32; 3];
    let [roll, pitch, yaw] = &mut angles;

    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    *roll = libm::atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    let sinp = libm::sqrtf(1.0 + 2.0 * (w * y - x * z));
    let cosp = libm::sqrtf(1.0 - 2.0 * (w * y - x * z));
    *pitch = 2.0 * libm::atan2f(sinp, cosp) - PI / 2.0;

    // yaw (z-axis rotation)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    *yaw = libm::atan2f(siny_cosp, cosy_cosp);

    return angles;
}

#[cfg(all(feature = "target-maxi", not(feature = "ultra-dev")))]
async fn bno_imu_handler<SI, SE>(mut bno: BNO080<SI>, delay: Counter<impl Instance, 100000>) -> !
where
    SI: SensorInterface<SensorError = SE>,
    SE: core::fmt::Debug,
{
    let mut delay = delay.release().delay();
    loop {
        let mut samples = [IMUSample::default(); 20];

        for i in 0..(12 * 8) {
            let (acc, quat) = loop {
                YieldFuture::new().await;
                bno.handle_all_messages(&mut delay, 1);
                if let (Ok(acc), Ok(rot)) = (bno.linear_accel(), bno.rotation_quaternion()) {
                    if !acc.iter().all(|&x| x == 0.0) && !rot.iter().all(|&x| x == 0.0) {
                        break (acc, rot);
                    }
                }
            };
            if i % 8 == 0 {
                samples[i / 8] = IMUSample {
                    timestamp: current_rtc_time(),
                    acceleration: acc,
                    angular_velocity: quaternion_to_euler(quat),
                };
            }
            let mut timer = delay.release().counter();
            timer.start(10u32.millis()).unwrap();
            NbFuture::new(|| timer.wait()).await.unwrap();

            delay = timer.release().delay();
        }

        let message = MessageType::new_imu(samples.into_iter());

        if role() == Role::Avionics {
            radio::queue_packet(message.clone().into_message(radio_ctxt()));
        }

        get_logger()
            .log(message.into_message(current_rtc_time()))
            .await;
    }
}

#[cfg(any(feature = "target-ultra", feature = "ultra-dev"))]
pub async fn bmi323_imu_handler(
    mut bmi323: Bmi323<SpiInterface<impl SpiDevice>, impl DelayNs>,
    mut timer: impl embedded_hal_async::delay::DelayNs,
) -> ! {
    loop {
        let mut samples = [IMUSample::default(); 20];

        for sample in samples.iter_mut() {
            sample.timestamp = current_rtc_time();
            sample.acceleration.copy_from_slice(
                &bmi323
                    .read_accel_data_scaled()
                    .map(|it| [it.x, it.y, it.z])
                    .unwrap(),
            );

            sample.angular_velocity.copy_from_slice(
                &bmi323
                    .read_gyro_data_scaled()
                    .map(|it| [it.x, it.y, it.z])
                    .unwrap(),
            );
            timer.delay_ms(10).await;
        }

        let message = MessageType::new_imu(samples.into_iter());
        let radio_msg = message.clone().into_message(radio_ctxt());
        if role() == Role::Avionics {
            radio::queue_packet(radio_msg);
        }

        get_logger()
            .log(message.into_message(current_rtc_time()))
            .await;
    }
}

#[cfg(any(feature = "target-ultra", feature = "ultra-dev"))]
pub async fn ms5607_altimeter_handler(
    mut ms5607: crate::ms5607::MS5607<
        impl I2c,
        impl embedded_hal_async::delay::DelayNs,
        crate::ms5607::Calibrated,
    >,
    pressure_sender: thingbuf::mpsc::StaticSender<FifoFrames>,
) {
    loop {
        let mut samples = [PressureTempSample::default(); 40];

        for i in 0..samples.len() {
            let crate::altimeter::PressureTemp {
                pressure,
                temperature,
            } = ms5607
                .read(crate::ms5607::Oversampling::OSR1024)
                .await
                .unwrap();
            samples[i] = PressureTempSample {
                timestamp: current_rtc_time(),
                pressure,
                temperature,
            };
            embedded_hal_async::delay::DelayNs::delay_ms(&mut ms5607, 10).await;
        }

        pressure_sender
            .send(
                samples
                    .iter()
                    .take(ALTIMETER_FRAME_COUNT)
                    .map(
                        |&PressureTempSample {
                             pressure,
                             temperature,
                             ..
                         }| crate::altimeter::PressureTemp {
                            pressure,
                            temperature,
                        },
                    )
                    .collect(),
            )
            .await
            .unwrap();

        let message = MessageType::new_pressure_temp(samples.into_iter());
        let radio_msg = message.clone().into_message(radio_ctxt());
        radio::queue_packet(radio_msg);

        get_logger()
            .log(message.into_message(current_rtc_time()))
            .await;

        YieldFuture::new().await;
    }
}

pub async fn begin(
    altimeter: Option<Altimeter>,
    pr_timer: Counter<TIM12, 10000>,
    icm_imu: Option<IcmImu<impl I2c, NoDmp>>,
    imu_timer: Counter<impl Instance, 100000>,
    bno_imu: Option<BNO080<impl SensorInterface<SensorError = impl core::fmt::Debug>>>,
    bmi323_imu: Option<Bmi323<SpiInterface<impl SpiDevice>, impl DelayNs>>,
    adxl_imu: Option<
        adxl375::spi::ADXL375<impl SpiDevice<u8>, impl embedded_hal_async::delay::DelayNs>,
    >,
    clocks: Clocks,
) -> ! {
    static PRESSURE_CHANNEL: StaticChannel<FifoFrames, 10> = StaticChannel::new();
    let (pressure_sender, pressure_receiver) = PRESSURE_CHANNEL.split();
    let altimeter_task: core::pin::Pin<&mut dyn Future<Output = ()>> = if role() != Role::GroundMain
    {
        #[cfg(any(
            feature = "target-mini",
            all(feature = "target-maxi", not(feature = "ultra-dev"))
        ))]
        {
            core::pin::pin!(bmp_altimeter_handler(
                altimeter.unwrap(),
                pr_timer,
                pressure_sender
            ))
        }
        #[cfg(any(feature = "target-ultra", feature = "ultra-dev"))]
        {
            let _ = pr_timer;
            core::pin::pin!(ms5607_altimeter_handler(
                altimeter.unwrap(),
                pressure_sender
            ))
        }
    } else {
        core::pin::pin!(core::future::ready(()))
    };

    match unsafe { ROLE } {
        Role::GroundMain | Role::GroundBackup =>
        {
            #[allow(unreachable_code)]
            join!(usb_handler(), gps_handler(), handle_incoming_packets()).0
        }
        Role::Avionics => {
            let imu_task = {
                #[cfg(feature = "target-mini")]
                {
                    let _ = bmi323_imu;
                    let _ = bno_imu;
                    icm_imu_handler(
                        icm_imu.unwrap(),
                        TimerDelay::new(imu_timer.release().release(), clocks),
                    )
                }
                #[cfg(all(feature = "target-maxi", not(feature = "ultra-dev")))]
                {
                    let _ = icm_imu;
                    let _ = bmi323_imu;
                    bno_imu_handler(bno_imu.unwrap(), imu_timer)
                }
                #[cfg(any(feature = "target-ultra", feature = "ultra-dev"))]
                {
                    let _ = bno_imu;
                    let _ = icm_imu;
                    ::futures::future::join(
                        bmi323_imu_handler(
                            bmi323_imu.unwrap(),
                            TimerDelay::new(imu_timer.release().release(), clocks),
                        ),
                        adxl_imu_handler(adxl_imu.unwrap()),
                    )
                }
            };

            #[allow(unreachable_code)]
            join!(
                usb_handler(),
                // buzzer_controller(),
                imu_task,
                gps_handler(),
                gps_broadcast(),
                altimeter_task,
                handle_incoming_packets(),
                stage_update_handler(pressure_receiver),
                // imu_task
            )
            .0
        }
        Role::Cansat => {
            #[allow(unreachable_code)]
            join!(
                usb_handler(),
                buzzer_controller(),
                gps_handler(),
                gps_broadcast(),
                handle_incoming_packets(),
                altimeter_task,
                stage_update_handler(pressure_receiver),
                // bno_handler(bno_imu.unwrap(), imu_timer),
            )
            .0
        }
        Role::CansatBackup =>
        {
            #[allow(unreachable_code)]
            join!(
                usb_handler(),
                buzzer_controller(),
                gps_handler(),
                gps_broadcast(),
                handle_incoming_packets(),
                altimeter_task,
                icm_imu_handler(
                    icm_imu.unwrap(),
                    TimerDelay::new(imu_timer.release().release(), clocks)
                ),
                stage_update_handler(pressure_receiver),
            )
            .0
        }
    }
}
