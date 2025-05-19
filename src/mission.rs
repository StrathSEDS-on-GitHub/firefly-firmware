use crate::{
    altimeter::PressureTemp,
    ms5607::{Calibrated, MS5607},
    pins::{PyroEnable, PyroFire1, PyroFire2},
    radio::{BNO_BROADCAST_BUF_LEN, BNO_BROADCAST_DECIMATION},
};
use bmi323::{Bmi323, interface::SpiInterface};
use bno080::{interface::SensorInterface, wrapper::BNO080};
use core::{cell::Cell, convert::Infallible, fmt::Write};
use cortex_m::interrupt::Mutex;
use embassy_futures::block_on;
use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    i2c::I2c,
    spi::SpiDevice,
};
use embedded_hal_async::delay::DelayNs as _;
use fugit::ExtU32;
use futures::join;
use heapless::Vec;
use icm20948_driver::icm20948::{NoDmp, i2c::IcmImu};
use nmea0183::{GGA, ParseResult};
use serde::{Deserialize, Serialize};
use stm32f4xx_hal::{
    ClearFlags,
    adc::{Adc, config::SampleTime},
    gpio::{Analog, Pin},
    pac::{ADC1, TIM12},
    timer::{self, Counter, Instance},
};
use storage_types::{CONFIG_KEYS, ConfigKey, ValueType};
use thingbuf::mpsc::{StaticChannel, StaticReceiver};

use crate::{
    Altimeter, BUZZER, BUZZER_TIMER, PYRO_TIMER, RTC,
    altimeter::{ALTIMETER_FRAME_COUNT, FifoFrames, read_altimeter_fifo},
    futures::{NbFuture, YieldFuture},
    gps,
    radio::{self, Message, RECEIVED_MESSAGE_QUEUE},
    logs::get_logger,
    usb_logger::get_serial,
};

pub static mut ROLE: Role = Role::Cansat;
pub static mut PYRO_ADC: Option<Adc<ADC1>> = None;
pub static mut PYRO_MEASURE_PIN: Option<Pin<'C', 0, Analog>> = None;
pub static mut PYRO_ENABLE_PIN: Option<PyroEnable> = None;
pub static mut PYRO_FIRE2: Option<PyroFire2> = None;
pub static mut PYRO_FIRE1: Option<PyroFire1> = None;

#[derive(Debug, PartialEq, Clone, Copy, Serialize, Deserialize)]
pub enum Role {
    Cansat,
    Avionics,
    CansatBackup,
    GroundMain,
    GroundBackup,
}

#[derive(Debug, PartialEq, Clone, Copy, Serialize, Deserialize)]
#[repr(u8)]
pub enum MissionStage {
    Disarmed(u16),
    Armed(u16),
    Ascent(u16),
    DescentDrogue(u16),
    DescentMain(u16),
    Landed(u16),
}

static STAGE: Mutex<Cell<MissionStage>> = Mutex::new(Cell::new(MissionStage::Disarmed(0)));

pub fn role() -> Role {
    // SAFETY: Role is mutated once by main prior to mission begin.
    unsafe { ROLE }
}

pub fn is_launched() -> bool {
    match current_stage() {
        MissionStage::Disarmed(_) => false,
        MissionStage::Armed(_) => false,
        _ => true,
    }
}

fn modify_pyro_state(state: u16) {
    cortex_m::interrupt::free(|cs| {
        let stage = STAGE.borrow(cs).get();
        STAGE.borrow(cs).set(match stage {
            MissionStage::Disarmed(_) => MissionStage::Disarmed(state),
            MissionStage::Armed(_) => MissionStage::Armed(state),
            MissionStage::Ascent(_) => MissionStage::Ascent(state),
            MissionStage::DescentDrogue(_) => MissionStage::DescentDrogue(state),
            MissionStage::DescentMain(_) => MissionStage::DescentMain(state),
            MissionStage::Landed(_) => MissionStage::Landed(state),
        });
    });
}

pub fn update_pyro_state() {
    let mv = unsafe {
        // Need to disarm pyro to read voltage correctly
        let pyro_enable = PYRO_ENABLE_PIN.as_mut().unwrap();
        let previous = pyro_enable.is_set_high();
        pyro_enable.set_low();
        let pyro = PYRO_MEASURE_PIN.as_mut().unwrap();
        let adc = PYRO_ADC.as_mut().unwrap();
        let sample = adc.convert(pyro, SampleTime::Cycles_56);

        if previous {
            pyro_enable.set_high();
        }
        adc.sample_to_millivolts(sample)
    };

    modify_pyro_state(mv);
    // FIXME: unfortunate. we should be able to submit this to the executor to avoid blocking.
    block_on(get_logger().log(format_args!("pyro,{}", mv)));
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
            MissionStage::Disarmed(s) => {
                /* Nothing to update, just wait for arm command */
                cortex_m::interrupt::free(|cs| STAGE.borrow(cs).set(MissionStage::Disarmed(s)));
            }
            MissionStage::Armed(s) => {
                // If we're above 10m, we must be ascending
                if altitudes
                    .iter()
                    .filter(|a| **a > 15.0 + start_altitude)
                    .count()
                    > 12
                {
                    get_logger().log_str("stage,detected ascent!").await;
                    cortex_m::interrupt::free(|cs| STAGE.borrow(cs).set(MissionStage::Ascent(s)));
                }
            }
            MissionStage::Ascent(s) => {
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
                        STAGE.borrow(cs).set(MissionStage::DescentDrogue(s))
                    });
                }
            }
            MissionStage::DescentDrogue(s) => {
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
                    cortex_m::interrupt::free(|cs| {
                        STAGE.borrow(cs).set(MissionStage::DescentMain(s))
                    });
                }
            }
            MissionStage::DescentMain(s) => {
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

                    cortex_m::interrupt::free(|cs| STAGE.borrow(cs).set(MissionStage::Landed(s)));
                }
            }
            MissionStage::Landed(_) => {
                // Nothing to do
            }
        };

        update_pyro_state();
        YieldFuture::new().await;
    }
}

pub fn current_stage() -> MissionStage {
    update_pyro_state();
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
                        STAGE.borrow(cs).set(MissionStage::Disarmed(0));
                    });
                    radio::queue_packet(Message::Disarm(role, 0));
                }
                None => writeln!(get_serial(), "Invalid disarm role").unwrap(),
            }
        } else if split.len() > 1 && split[0].starts_with(b"arm") {
            match parse_role(split[1]) {
                Some(role) => {
                    radio::queue_packet(Message::Arm(role, 0));
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

                radio::queue_packet(Message::TestPyro(
                    0,
                    role,
                    pin,
                    core::str::from_utf8(split[3])
                        .unwrap()
                        .trim()
                        .parse()
                        .unwrap(),
                ));
            } else {
                writeln!(get_serial(), "Invalid test-pyro role").unwrap();
            }
        } else if split.len() > 1 && split[0].starts_with(b"launch") {
            let new_state = if split[1].starts_with(b"true") {
                MissionStage::Ascent(0)
            } else {
                MissionStage::Disarmed(0)
            };

            cortex_m::interrupt::free(|cs| STAGE.borrow(cs).set(new_state));
        } else if split.len() > 1 && split[0].starts_with(b"setstage") {
            let role = match parse_role(split[1]) {
                Some(r) => r,
                None => continue,
            };

            let stage = if split[2].starts_with(b"ascent") {
                MissionStage::Ascent(0)
            } else if split[1].starts_with(b"descent-drogue") {
                MissionStage::DescentDrogue(0)
            } else if split[2].starts_with(b"descent-main") {
                MissionStage::DescentMain(0)
            } else if split[2].starts_with(b"landed") {
                MissionStage::Landed(0)
            } else {
                writeln!(get_serial(), "Invalid stage").unwrap();
                continue;
            };

            radio::queue_packet(Message::SetStage(role, stage, 0));
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
        } else if split.len() >= 1 && split[0].starts_with(b"logs") {
            if let Err(msg) = get_logger()
                .get_logs(async |s| {
                    get_serial().log(s).await;
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
fn current_rtc_time() -> RTCTime {
    cortex_m::interrupt::free(|cs| {
        let mut rtc_ref = RTC.borrow(cs).borrow_mut();
        let rtc = rtc_ref.as_mut().unwrap();
        let date_time = rtc.get_datetime();
        date_time.as_hms_milli()
    })
}

#[derive(Debug, Serialize, Deserialize, PartialEq, Eq, Clone, Copy)]
#[repr(transparent)]
pub struct EpochTime(u32);

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
        let mut latitudes = [0.0; 10];
        let mut longitudes = [0.0; 10];
        let mut altitudes = [0.0; 10];

        let mut i = 0;
        let mut start_time = None;
        while i < 10 * 2 {
            if i == 0 {
                let time = current_rtc_time();
                start_time = Some(time.into());
            }
            let fix = gps::next_sentence().await;
            if let ParseResult::GGA(Some(GGA { fix: Some(gga), .. })) = fix {
                if i % 2 == 0 {
                    // Send every other fix (5Hz).
                    latitudes[i / 2] = gga.latitude.as_f64() as f32;
                    longitudes[i / 2] = gga.longitude.as_f64() as f32;
                    altitudes[i / 2] = gga.altitude.meters;
                }

                get_logger()
                    .log(format_args!(
                        "gps,{},{},{}",
                        gga.latitude.as_f64(),
                        gga.longitude.as_f64(),
                        gga.altitude.meters
                    ))
                    .await;
                i += 1;
            }
        }
        let message = Message::GpsBroadCast {
            counter: radio::next_counter(),
            stage: current_stage(),
            role: role(),
            time_of_first_packet: start_time.unwrap(),
            latitudes,
            longitudes,
            altitudes,
        };

        radio::queue_packet(message);
    }
}

async fn pressure_temp_handler(
    mut sensor: Altimeter,
    mut timer: Counter<TIM12, 10000>,
    pressure_sender: thingbuf::mpsc::StaticSender<FifoFrames>,
) {
    // Wait for FIFO to fill up.
    timer.clear_flags(timer::Flag::Update);
    timer.start(550.millis()).unwrap();
    NbFuture::new(|| timer.wait()).await.unwrap();
    loop {
        let mut pressures = [0.0; 8];
        let mut temperatures = [0.0; 8];

        let start_time = current_rtc_time().into();

        let mut i = 0;

        let nth = if role() == Role::CansatBackup { 8 } else { 4 };

        while i < nth * 8 {
            let frames;
            (frames, sensor) = read_altimeter_fifo(sensor).await;
            for frame in &frames {
                let pressure = frame.pressure;
                let temperature = frame.temperature;

                // Send every fourth frame (10Hz).
                if i % nth == 0 && i < nth * 8 {
                    pressures[i / nth] = pressure;
                    temperatures[i / nth] = temperature;
                }

                get_logger()
                    .log(format_args!("pressure,{},{}", pressure, temperature))
                    .await;

                i += 1;
            }
            pressure_sender.send(frames.into()).await.unwrap();
            timer.clear_flags(timer::Flag::Update);
            timer.start(400.millis()).unwrap();
            // FIXME: Use interrupt to wait for FIFO to fill up.
            NbFuture::new(|| timer.wait()).await.unwrap();
        }

        let message = Message::PressureTempBroadCast {
            counter: radio::next_counter(),
            stage: current_stage(),
            role: role(),
            time_of_first_packet: start_time,
            pressures,
            temperatures,
        };

        // writeln!(get_serial(), "Sending: {:?}", message).unwrap();

        radio::queue_packet(message);
    }
}

async fn strain_handler(
    mut ads_dout: impl InputPin,
    mut ads_sclk: impl OutputPin,
    mut ads_delay: impl DelayNs,
) {
    loop {
        let mut buffer = [0i32; 16];

        for i in 0..16 {
            while ads_dout.is_high().unwrap() {
                YieldFuture::new().await;
            }

            let mut data = 0u32;
            for i in 0..24usize {
                ads_sclk.set_high().unwrap();
                ads_delay.delay_us(1u32);
                data |= (ads_dout.is_high().unwrap() as u32) << (23 - i);
                ads_sclk.set_low().unwrap();
                ads_delay.delay_us(1u32);
            }

            // Sign extend the 24-bit value to 32 bits
            // https://stackoverflow.com/a/42536138
            let sx_24_32 = |i: u32| {
                let m = 1 << 23;
                return ((i ^ m).wrapping_sub(m)) as i32;
            };

            let data = sx_24_32(data);
            buffer[i] = data;
        }

        get_logger().log(format_args!("strain,{:?}", buffer)).await;

        let message = Message::Strain {
            counter: radio::next_counter(),
            stage: current_stage(),
            role: role(),
            time_of_first_packet: current_rtc_time().into(),
            strain: buffer,
        };

        radio::queue_packet(message);
    }
}
async fn handle_incoming_packets() -> ! {
    loop {
        if let Some(packet) = cortex_m::interrupt::free(|cs| {
            RECEIVED_MESSAGE_QUEUE.borrow(cs).borrow_mut().pop_front()
        }) {
            match role() {
                Role::GroundMain | Role::GroundBackup => match packet {
                    Message::GpsBroadCast {
                        counter,
                        stage,
                        role,
                        time_of_first_packet,
                        latitudes,
                        longitudes,
                        altitudes,
                    } => {
                        writeln!(
                            get_serial(),
                            "gps,{:?},{:?},{},{:?},{:?},{:?},{:?}",
                            role,
                            stage,
                            counter,
                            time_of_first_packet,
                            latitudes,
                            longitudes,
                            altitudes
                        )
                        .unwrap();

                        let rssi = radio::get_packet_status().rssi_pkt();

                        writeln!(get_serial(), "rssi,{:?},{}", role, rssi).unwrap();
                    }
                    Message::PressureTempBroadCast {
                        counter,
                        stage,
                        role,
                        time_of_first_packet,
                        pressures,
                        temperatures,
                    } => {
                        writeln!(
                            get_serial(),
                            "pressuretemp,{:?},{:?},{},{:?},{:?},{:?}",
                            role,
                            stage,
                            counter,
                            time_of_first_packet,
                            pressures,
                            temperatures
                        )
                        .unwrap();
                        let rssi = radio::get_packet_status().rssi_pkt();

                        writeln!(get_serial(), "rssi,{:?},{}", role, rssi).unwrap();
                    }
                    Message::Arm(..) => {}
                    Message::Disarm(..) => {}
                    Message::TestPyro(..) => {}
                    Message::SetStage(..) => {}
                    Message::IMUBroadcast {
                        counter,
                        stage,
                        role,
                        time_of_first_packet,
                        accels,
                        gyros,
                    } => {
                        writeln!(
                            get_serial(),
                            "imu,{:?},{:?},{},{:?},{:?},{:?}",
                            role,
                            stage,
                            counter,
                            time_of_first_packet,
                            accels,
                            gyros
                        )
                        .unwrap();
                        let rssi = radio::get_packet_status().rssi_pkt();

                        writeln!(get_serial(), "rssi,{:?},{}", role, rssi).unwrap();
                    }
                    Message::Strain {
                        counter,
                        stage,
                        role,
                        time_of_first_packet,
                        strain,
                    } => {
                        writeln!(
                            get_serial(),
                            "strain,{:?},{:?},{},{:?},{:?}",
                            role,
                            stage,
                            counter,
                            time_of_first_packet,
                            strain
                        )
                        .unwrap();
                        let rssi = radio::get_packet_status().rssi_pkt();

                        writeln!(get_serial(), "rssi,{:?},{}", role, rssi).unwrap();
                    }
                    Message::BNOBroadcast {
                        counter,
                        stage,
                        role,
                        time_of_first_packet,
                        accels,
                        rots,
                    } => {
                        writeln!(
                            get_serial(),
                            "bno,{:?},{:?},{},{:?},{:?},{:?}",
                            role,
                            stage,
                            counter,
                            time_of_first_packet,
                            accels,
                            rots
                        )
                        .unwrap();
                        let rssi = radio::get_packet_status().rssi_pkt();

                        writeln!(get_serial(), "rssi,{:?},{}", role, rssi).unwrap();
                    }
                    Message::SpeedSound {
                        counter,
                        detected_offset,
                    } => {
                        writeln!(get_serial(), "sound,{},{:?}", counter, detected_offset).unwrap();
                        let rssi = radio::get_packet_status().rssi_pkt();

                        writeln!(get_serial(), "rssi,{}", rssi).unwrap();
                    }
                },
                _ => match packet {
                    Message::Disarm(r, _) if r == role() => {
                        disarm().await;
                    }
                    Message::Arm(r, _) if r == role() => {
                        arm().await;
                    }
                    Message::TestPyro(_, r, pin, duration) if r == role() => {
                        fire_pyro(pin, duration).await;
                    }
                    Message::SetStage(r, stage, _) if r == role() => {
                        cortex_m::interrupt::free(|cs| {
                            STAGE.borrow(cs).replace(stage);
                            update_pyro_state();
                        });
                    }
                    _ => {}
                },
            }
        }

        YieldFuture::new().await;
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PyroPin {
    One,
    Two,
    Both,
}

pub async fn fire_pyro(pin: PyroPin, duration: u32) {
    let buzz = unsafe { BUZZER.as_mut().unwrap() };
    let timer = unsafe { PYRO_TIMER.as_mut().unwrap() };
    trait Pin: OutputPin<Error = Infallible> + core::fmt::Debug {}
    impl<T: OutputPin<Error = Infallible> + core::fmt::Debug> Pin for T {}

    let mut pyro_fires = {
        let mut vec: Vec<&mut dyn Pin, 2> = Vec::new();
        let _ = match pin {
            PyroPin::One => {
                vec.push(unsafe { PYRO_FIRE1.as_mut().unwrap() }).unwrap();
            }
            PyroPin::Two => {
                vec.push(unsafe { PYRO_FIRE2.as_mut().unwrap() }).unwrap();
            }
            PyroPin::Both => {
                vec.push(unsafe { PYRO_FIRE1.as_mut().unwrap() }).unwrap();
                vec.push(unsafe { PYRO_FIRE2.as_mut().unwrap() }).unwrap();
            }
        };
        vec
    };

    let mut time = 200i32;
    while time > 0 {
        buzz.set_high();
        timer.start(200u32.millis()).unwrap();
        nb::block!(timer.wait()).unwrap();
        buzz.set_low();
        timer.start((time as u32).millis()).unwrap();
        nb::block!(timer.wait()).unwrap();
        time = (time as f32 * 0.8 - 5.0) as i32;
    }

    buzz.set_high();

    timer.start(500u32.millis()).unwrap();
    nb::block!(timer.wait()).unwrap();
    buzz.set_low();
    for fire in &mut pyro_fires {
        fire.set_high().unwrap();
    }
    timer.start(duration.millis()).unwrap();
    nb::block!(timer.wait()).unwrap();
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
        STAGE.borrow(cs).replace(MissionStage::Disarmed(0));
        update_pyro_state();
    });

    timer.clear_flags(timer::Flag::Update);
    timer.start(300u32.millis()).unwrap();
    NbFuture::new(|| timer.wait()).await.unwrap();
    buzz.set_low();
}

pub async fn arm() {
    let buzz = unsafe { BUZZER.as_mut().unwrap() };
    let timer = unsafe { BUZZER_TIMER.as_mut().unwrap() };
    let pyro_enable = unsafe { PYRO_ENABLE_PIN.as_mut().unwrap() };
    buzz.set_high();
    pyro_enable.set_high();
    cortex_m::interrupt::free(|cs| {
        STAGE.borrow(cs).replace(MissionStage::Armed(0));
        update_pyro_state();
    });
    timer.clear_flags(timer::Flag::Update);
    timer.start(300u32.millis()).unwrap();
    NbFuture::new(|| timer.wait()).await.unwrap();
    buzz.set_low()
}

async fn buzzer_controller() -> ! {
    // Buzz 1s on startup
    let buzz = unsafe { BUZZER.as_mut().unwrap() };
    let timer = unsafe { BUZZER_TIMER.as_mut().unwrap() };
    buzz.set_high();
    timer.start(1000u32.millis()).unwrap();
    NbFuture::new(|| timer.wait()).await.unwrap();
    buzz.set_low();

    while !matches!(
        cortex_m::interrupt::free(|cs| STAGE.borrow(cs).get()),
        MissionStage::Landed(_)
    ) {
        YieldFuture::new().await;
    }

    let buzz = unsafe { BUZZER.as_mut().unwrap() };
    let timer = unsafe { BUZZER_TIMER.as_mut().unwrap() };

    // Buzz on landing
    loop {
        buzz.set_high();
        timer.start(500u32.millis()).unwrap();
        NbFuture::new(|| timer.wait()).await.unwrap();
        buzz.set_low();
        timer.start(500u32.millis()).unwrap();
        NbFuture::new(|| timer.wait()).await.unwrap();
    }
}

async fn imu_handler(
    mut imu: IcmImu<impl I2c, NoDmp>,
    mut imu_timer: impl embedded_hal_async::delay::DelayNs,
) -> ! {
    let mut acc_buffer = [[0.0f32; 3]; 8];
    let mut gyro_buffer = [[0.0f32; 3]; 8];

    loop {
        // SAFETY: We're in a loop, so we're guaranteed to get a value.
        let mut start_time = None;

        for (acc, gyro) in acc_buffer.iter_mut().zip(gyro_buffer.iter_mut()) {
            let (acc_data, gyro_data) = loop {
                if let Ok(x) = imu.read_acc().and_then(|a| imu.read_gyro().map(|g| (a, g))) {
                    start_time.get_or_insert(current_rtc_time().into());
                    get_logger()
                        .log(format_args!("imu,{:?},{:?}", x.0, x.1))
                        .await;
                    break x;
                };
                YieldFuture::new().await;
            };

            *acc = acc_data;
            *gyro = gyro_data;
            imu_timer.delay_ms(10).await;
        }

        let message = Message::IMUBroadcast {
            counter: radio::next_counter(),
            stage: current_stage(),
            role: role(),
            time_of_first_packet: start_time.unwrap(),
            accels: acc_buffer,
            gyros: gyro_buffer,
        };
        if role() == Role::Avionics {
            radio::queue_packet(message);
        }
    }
}

async fn bno_handler<SI, SE>(mut bno: BNO080<SI>, delay: Counter<impl Instance, 100000>) -> !
where
    SI: SensorInterface<SensorError = SE>,
    SE: core::fmt::Debug,
{
    let mut delay = delay.release().delay();
    loop {
        let mut acc_buffer = [[0.0f32; 3]; BNO_BROADCAST_BUF_LEN];
        let mut quat_buffer = [[0.0f32; 4]; BNO_BROADCAST_BUF_LEN];

        for i in 0..(BNO_BROADCAST_BUF_LEN * BNO_BROADCAST_DECIMATION) {
            let (acc, quat) = loop {
                YieldFuture::new().await;
                bno.handle_all_messages(&mut delay, 1);
                if let (Ok(acc), Ok(rot)) = (bno.linear_accel(), bno.rotation_quaternion()) {
                    if !acc.iter().all(|&x| x == 0.0) && !rot.iter().all(|&x| x == 0.0) {
                        break (acc, rot);
                    }
                }
            };
            if i % BNO_BROADCAST_DECIMATION == 0 {
                acc_buffer[i / BNO_BROADCAST_DECIMATION] = acc;
                quat_buffer[i / BNO_BROADCAST_DECIMATION] = quat;
            }
            get_logger()
                .log(format_args!("bno,{:?},{:?}", acc, quat))
                .await;
            let mut timer = delay.release().counter();
            timer.start(10u32.millis()).unwrap();
            NbFuture::new(|| timer.wait()).await.unwrap();

            delay = timer.release().delay();
        }

        let message = Message::BNOBroadcast {
            counter: radio::next_counter(),
            stage: current_stage(),
            role: role(),
            time_of_first_packet: current_rtc_time().into(),
            accels: acc_buffer,
            rots: quat_buffer,
        };

        if role() == Role::Avionics {
            radio::queue_packet(message);
        }
    }
}

pub async fn bmi323_handler(
    mut bmi323: Bmi323<SpiInterface<impl SpiDevice>, impl DelayNs>,
    mut timer: impl embedded_hal_async::delay::DelayNs,
) -> ! {
    loop {
        let mut acc_buffer = [[0.0f32; 3]; 8];
        let mut gyro_buffer = [[0.0f32; 3]; 8];

        for (acc, gyro) in acc_buffer.iter_mut().zip(gyro_buffer.iter_mut()) {
            acc.copy_from_slice(
                &bmi323
                    .read_accel_data_scaled()
                    .map(|it| [it.x, it.y, it.z])
                    .unwrap(),
            );

            gyro.copy_from_slice(
                &bmi323
                    .read_gyro_data_scaled()
                    .map(|it| [it.x, it.y, it.z])
                    .unwrap(),
            );
            get_logger()
                .log(format_args!("bmi323,{:?},{:?}", acc, gyro))
                .await;
            timer.delay_ms(10).await;
        }

        let message = Message::IMUBroadcast {
            counter: radio::next_counter(),
            stage: current_stage(),
            role: role(),
            time_of_first_packet: current_rtc_time().into(),
            accels: acc_buffer,
            gyros: gyro_buffer,
        };
        if role() == Role::Avionics {
            radio::queue_packet(message);
        }
    }
}

pub async fn ms5607_handler(
    mut ms5607: MS5607<impl I2c, impl embedded_hal_async::delay::DelayNs, Calibrated>,
) -> ! {
    loop {
        let mut pressures = [0.0; 8];
        let mut temperatures = [0.0; 8];

        for i in 0..8 {
            let PressureTemp {
                pressure,
                temperature,
            } = ms5607
                .read(crate::ms5607::Oversampling::OSR1024)
                .await
                .unwrap();
            pressures[i] = pressure;
            temperatures[i] = temperature;

            ms5607.delay_ms(10).await;
        }

        get_logger()
            .log(format_args!("pressure,{:?},{:?}", pressures, temperatures))
            .await;
        let message = Message::PressureTempBroadCast {
            counter: radio::next_counter(),
            stage: current_stage(),
            role: role(),
            time_of_first_packet: current_rtc_time().into(),
            pressures,
            temperatures,
        };
        radio::queue_packet(message);

        YieldFuture::new().await;
    }
}

pub async fn begin(
    pressure_sensor: Option<Altimeter>,
    pr_timer: Counter<TIM12, 10000>,
    imu: Option<IcmImu<impl I2c, NoDmp>>,
    imu_timer: impl embedded_hal_async::delay::DelayNs,
    bno_imu: Option<BNO080<impl SensorInterface<SensorError = impl core::fmt::Debug>>>,
    ms5607: Option<MS5607<impl I2c, impl embedded_hal_async::delay::DelayNs, Calibrated>>,
    bmi323: Option<Bmi323<SpiInterface<impl SpiDevice>, impl DelayNs>>,
) -> ! {
    static PRESSURE_CHANNEL: StaticChannel<FifoFrames, 10> = StaticChannel::new();
    let (pressure_sender, pressure_receiver) = PRESSURE_CHANNEL.split();
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
                    imu_handler(imu.unwrap(), imu_timer)
                }
                #[cfg(all(feature = "target-maxi", not(feature = "ultra-dev")))]
                {
                    let _ = imu;
                    bno_handler(bno_imu.unwrap(), imu_timer)
                }
                #[cfg(all(feature = "target-maxi", feature = "ultra-dev"))]
                {
                    bmi323_handler(bmi323.unwrap(), imu_timer)
                }
            };

            #[allow(unreachable_code)]
            join!(
                usb_handler(),
                // buzzer_controller(),
                imu_task,
                gps_handler(),
                gps_broadcast(),
                // pressure_temp_handler(pressure_sensor.unwrap(), pr_timer, pressure_sender),
                handle_incoming_packets(),
                // stage_update_handler(pressure_receiver),
                ms5607_handler(ms5607.unwrap(),),
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
                pressure_temp_handler(pressure_sensor.unwrap(), pr_timer, pressure_sender),
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
                pressure_temp_handler(pressure_sensor.unwrap(), pr_timer, pressure_sender),
                imu_handler(imu.unwrap(), imu_timer),
                stage_update_handler(pressure_receiver),
            )
            .0
        }
    }
}
