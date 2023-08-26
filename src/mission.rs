use core::{borrow::BorrowMut, cell::Cell, convert::Infallible, fmt::Write};
use cortex_m::interrupt::Mutex;
use embedded_hal::blocking::i2c::WriteRead;
use embedded_hal::digital::v2::OutputPin;
use fugit::ExtU32;
use futures::join;
use heapless::Vec;
use nmea0183::{ParseResult, GGA};
use serde::{Deserialize, Serialize};
use stm32f4xx_hal::{
    adc::{config::SampleTime, Adc},
    gpio::{Analog, Output, Pin},
    pac::{ADC1, TIM12, TIM4},
    timer::{CounterMs, Event},
};
use thingbuf::mpsc::{StaticChannel, StaticReceiver, StaticSender};

use crate::{
    bmp581::{PressureTemp, BMP581},
    futures::{NbFuture, YieldFuture},
    gps,
    ina219::INA219,
    logger::get_serial,
    radio::{self, Message, RECEIVED_MESSAGE_QUEUE},
    sdio::get_logger,
    BUZZER, BUZZER_TIMER, PYRO_TIMER, RTC,
};

pub static mut ROLE: Role = Role::Cansat;
pub static mut PYRO_ADC: Option<Adc<ADC1>> = None;
pub static mut PYRO_MEASURE_PIN: Option<Pin<'C', 0, Analog>> = None;
pub static mut PYRO_ENABLE_PIN: Option<Pin<'D', 7, Output>> = None;
pub static mut PYRO_FIRE2: Option<Pin<'D', 6, Output>> = None;
pub static mut PYRO_FIRE1: Option<Pin<'D', 5, Output>> = None;

#[derive(Debug, PartialEq, Clone, Copy, Serialize, Deserialize)]
pub enum Role {
    Cansat,
    Avionics,
    Ground,
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

static STAGE: Mutex<Cell<MissionStage>> = Mutex::new(Cell::new(MissionStage::Armed(0)));

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

    get_logger().log(format_args!("pyro,{}", mv));
}

pub async fn stage_update_handler(channel: StaticReceiver<[PressureTemp; 15]>) {
    let mut start_altitude: f32 = 0.0;
    let mut sea_level_pressure: f32 = 0.0;

    loop {
        let frames = channel.recv().await;
        if let None = frames {
            YieldFuture::new().await;
            continue;
        }
        let frames = frames.unwrap();
        if sea_level_pressure == 0.0 {
            let mut vec: Vec<f32, 15> = frames
                .map(|frame| {
                    let pressure = frame.pressure as f32 / libm::powf(2.0, 6.0);
                    pressure
                })
                .into_iter()
                .collect();
            vec.sort_unstable_by(f32::total_cmp);
            // Take the average of the middle 5 values
            sea_level_pressure = vec[7..12].iter().sum::<f32>() / 5.0;
        }

        let stage = current_stage();
        let altitudes = frames.map(|frame| {
            let pressure = frame.pressure as f32 / libm::powf(2.0, 6.0);
            let temperature = frame.temperature as f32 / libm::powf(2.0, 16.0);

            let altitude = (libm::powf(sea_level_pressure / pressure, 1.0 / 5.257) - 1.0)
                * (temperature + 273.15)
                / 0.0065;
            altitude
        });

        if start_altitude == 0.0 {
            // If we don't have a start pressure, we can't do anything
            let mut vec: Vec<f32, 15> = altitudes.clone().into_iter().collect();
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
                    get_logger().log_str("stage,detected ascent!");
                    cortex_m::interrupt::free(|cs| STAGE.borrow(cs).set(MissionStage::Ascent(s)));
                }
            }
            MissionStage::Ascent(s) => {
                // If our velocity is negative, we must be descending
                let velocities = altitudes.windows(3).map(|w| w[2] - w[0]);

                if velocities.filter(|v| *v < 0.05).count() > 10 {
                    get_logger().log_str("stage,detected apogee");
                    if role() == Role::Avionics {
                        get_logger().log_str("stage,firing drogue!");
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
                if altitudes.iter().filter(|a| **a < 320.0).count() > 12 {
                    get_logger().log_str("stage,detected main");
                    if role() == Role::Avionics {
                        get_logger().log_str("stage,firing main!");
                        // If we're the avionics, fire the pyro
                        fire_pyro(PyroPin::Two, 1000).await;
                    } else if role() == Role::Cansat {
                        get_logger().log_str("stage,firing pyro1!");
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
                    get_logger().log_str("stage,detected entering landed stage");
                    if role() == Role::Cansat {
                        get_logger().log_str("stage,landed! firing pyro2!");
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
        Some(Role::Ground)
    } else {
        None
    }
}

pub async fn usb_handler() -> ! {
    let mut buf = [0u8; 256];
    let mut idempotency_counter = 0;
    loop {
        let bytes = get_serial().read(&mut buf).await.unwrap();

        let split: Vec<_, 32> = bytes.split(|b| *b == b',').collect();
        if split.len() > 1 && split[0].starts_with(b"disarm") {
            match parse_role(split[1]) {
                Some(role) => {
                    cortex_m::interrupt::free(|cs| {
                        STAGE.borrow(cs).set(MissionStage::Disarmed(0));
                    });
                    for _ in 0..10 {
                        radio::queue_packet(Message::Disarm(role, idempotency_counter));
                    }
                    idempotency_counter += 1;
                }
                None => writeln!(get_serial(), "Invalid disarm role").unwrap(),
            }
        } else if split.len() > 1 && split[0].starts_with(b"arm") {
            match parse_role(split[1]) {
                Some(role) => {
                    for _ in 0..10 {
                        radio::queue_packet(Message::Arm(role, idempotency_counter));
                    }
                    writeln!(get_serial(), "Sent arm packet!").unwrap();
                    idempotency_counter += 1;
                }
                None => writeln!(get_serial(), "Invalid arm role").unwrap(),
            }
        } else if split.len() > 3 && split[0].starts_with(b"fire") {
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

            for _ in 0..10 {
                radio::queue_packet(Message::SetStage(role, stage, idempotency_counter));
            }
            idempotency_counter += 1;
        } else {
            writeln!(get_serial(), "Invalid command").unwrap();
        }

        writeln!(get_serial(), "Received: {:?}", core::str::from_utf8(bytes)).unwrap();
        YieldFuture::new().await;
    }
}

async fn gps_handler() -> ! {
    gps::poll_for_sentences().await
}

fn current_rtc_time() -> (u8, u8, u8, u16) {
    cortex_m::interrupt::free(|cs| {
        let mut rtc_ref = RTC.borrow(cs).borrow_mut();
        let rtc = rtc_ref.as_mut().unwrap();
        let date_time = rtc.get_datetime();
        date_time.as_hms_milli()
    })
}

async fn gps_broadcast() -> ! {
    loop {
        let mut latitudes = [0.0; 10];
        let mut longitudes = [0.0; 10];
        let mut altitudes = [0.0; 10];

        let mut i = 0;
        let mut start_time = None;
        while i < 20 {
            if i == 0 {
                let time = current_rtc_time();
                start_time = Some(
                    time.0 as u32 * 3_600_000
                        + time.1 as u32 * 60_000
                        + time.2 as u32 * 1000
                        + time.3 as u32,
                );
            }
            let fix = gps::next_sentence().await;
            if let ParseResult::GGA(Some(GGA { fix: Some(gga), .. })) = fix {
                if i % 2 == 0 {
                    // Send every other fix (5Hz).
                    latitudes[i / 2] = gga.latitude.as_f64() as f32;
                    longitudes[i / 2] = gga.longitude.as_f64() as f32;
                    altitudes[i / 2] = gga.altitude.meters;
                }

                get_logger().log(format_args!(
                    "gps,{},{},{}",
                    gga.latitude.as_f64(),
                    gga.longitude.as_f64(),
                    gga.altitude.meters
                ));
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
    mut sensor: BMP581,
    mut timer: CounterMs<TIM12>,
    pressure_sender: StaticSender<[PressureTemp; 15]>,
) {
    // Wait for FIFO to fill up.
    timer.clear_interrupt(Event::Update);
    timer.start(550.millis()).unwrap();
    NbFuture::new(|| timer.wait()).await.unwrap();
    loop {
        let mut pressures = [0.0; 8];
        let mut temperatures = [0.0; 8];

        let time = current_rtc_time();
        let start_time = time.0 as u32 * 3_600_000
            + time.1 as u32 * 60_000
            + time.2 as u32 * 1000
            + time.3 as u32;

        let mut i = 0;

        while i < 4 * 8 {
            if let Ok(frames) = sensor.read_fifo() {
                for frame in frames {
                    let pressure = frame.pressure as f32 / libm::powf(2.0, 6.0);
                    let temperature = frame.temperature as f32 / libm::powf(2.0, 16.0);

                    // Send every fourth frame (10Hz).
                    if i % 4 == 0 && i < 4 * 8 {
                        pressures[i / 4] = pressure;
                        temperatures[i / 4] = temperature;
                    }

                    get_logger().log(format_args!("pressure,{},{}", pressure, temperature));

                    i += 1;
                }
                pressure_sender.send(frames).await.unwrap();
            }
            timer.clear_interrupt(Event::Update);
            timer.start(400.millis()).unwrap();
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

async fn handle_incoming_packets() -> ! {
    let mut idempotency_counter = 0;
    loop {
        if let Some(packet) = cortex_m::interrupt::free(|cs| {
            RECEIVED_MESSAGE_QUEUE.borrow(cs).borrow_mut().pop_front()
        }) {
            match role() {
                Role::Ground => match packet {
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

                        let rssi = radio::get_rssi();

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
                        let rssi = radio::get_rssi();

                        writeln!(get_serial(), "rssi,{:?},{}", role, rssi).unwrap();
                    }
                    Message::Arm(..) => {}
                    Message::Disarm(..) => {}
                    Message::TestPyro(..) => {}
                    Message::SetStage(..) => {}
                    Message::CurrentSensorBroadcast {
                        counter,
                        stage,
                        role,
                        time_of_first_packet,
                        currents,
                        voltages,
                    } => {
                        writeln!(
                            get_serial(),
                            "current,{:?},{:?},{},{:?},{:?},{:?}",
                            role,
                            stage,
                            counter,
                            time_of_first_packet,
                            currents,
                            voltages
                        )
                        .unwrap();
                        let rssi = radio::get_rssi();

                        writeln!(get_serial(), "rssi,{:?},{}", role, rssi).unwrap();
                    }
                },
                _ => match packet {
                    Message::Disarm(r, i) if r == role() && i > idempotency_counter => {
                        idempotency_counter = i;
                        disarm().await;
                    }
                    Message::Arm(r, i) if r == role() && i > idempotency_counter => {
                        idempotency_counter = i;
                        arm().await;
                    }
                    Message::TestPyro(r, pin, duration) if r == role() => {
                        fire_pyro(pin, duration).await;
                    }
                    Message::SetStage(r, stage, i) if r == role() && i > idempotency_counter => {
                        idempotency_counter = i;
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
        NbFuture::new(|| timer.wait()).await.unwrap();
        buzz.set_low();
        timer.start((time as u32).millis()).unwrap();
        NbFuture::new(|| timer.wait()).await.unwrap();
        time = (time as f32 * 0.8 - 5.0) as i32;
    }

    buzz.set_high();

    timer.start(500u32.millis()).unwrap();
    NbFuture::new(|| timer.wait()).await.unwrap();
    buzz.set_low();
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
        STAGE.borrow(cs).replace(MissionStage::Disarmed(0));
        update_pyro_state();
    });

    timer.clear_interrupt(Event::Update);
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
    timer.clear_interrupt(Event::Update);
    timer.start(300u32.millis()).unwrap();
    NbFuture::new(|| timer.wait()).await.unwrap();
    buzz.set_low()
}

async fn current_sensor_handler<E, I2C>(mut sensor: INA219<'_, E, I2C>, mut timer: CounterMs<TIM4>)
where
    I2C: WriteRead<Error = E> + embedded_hal::blocking::i2c::Write<Error = E>,
    E: core::fmt::Debug,
{
    // Wait for FIFO to fill up.
    timer.clear_interrupt(Event::Update);
    timer.start(550.millis()).unwrap();
    NbFuture::new(|| timer.wait()).await.unwrap();
    let mut voltages = [0; 8];
    let mut currents = [0; 8];

    let mut i = 0;
    let mut start_time;
    loop {
        let time = current_rtc_time();
        start_time = time.0 as u32 * 3_600_000
            + time.1 as u32 * 60_000
            + time.2 as u32 * 1000
            + time.3 as u32;

        let current = sensor.current();
        let voltage = sensor.voltage();

        if let (Ok(current), Ok(voltage)) = (current, voltage) {
            voltages[i % 8] = voltage;
            currents[i % 8] = current;

            get_logger().log(format_args!("current,{},{}", current, voltage));
            i = i.wrapping_add(1);
        }

        if i % 8 == 0 && matches!(current_stage(), MissionStage::Landed(_)) {
            let message = Message::CurrentSensorBroadcast {
                counter: radio::next_counter(),
                stage: current_stage(),
                role: role(),
                time_of_first_packet: start_time,
                currents,
                voltages,
            };

            radio::queue_packet(message);
        }
        timer.clear_interrupt(Event::Update);
        timer.start(1000.millis()).unwrap();
        NbFuture::new(|| timer.wait()).await.unwrap();
    }
}

async fn buzzer_controller() -> ! {
    // Buzz 1s on startup
    let buzz = unsafe { BUZZER.as_mut().unwrap() };
    let timer = unsafe { BUZZER_TIMER.as_mut().unwrap() };
    buzz.set_high();
    timer.start(1000u32.millis()).unwrap();
    NbFuture::new(|| timer.wait()).await.unwrap();
    buzz.set_low();

    drop(timer);
    drop(buzz);

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

pub async fn begin<E, I2C>(
    pressure_sensor: Option<BMP581>,
    current_sensor: Option<INA219<'_, E, I2C>>,
    pr_timer: CounterMs<TIM12>,
    i_timer: CounterMs<TIM4>,
) -> !
where
    I2C: WriteRead<Error = E> + embedded_hal::blocking::i2c::Write<Error = E>,
    E: core::fmt::Debug,
{
    radio::update_timer(0.0);
    match unsafe { ROLE } {
        Role::Ground =>
        {
            #[allow(unreachable_code)]
            join!(usb_handler(), gps_handler(), handle_incoming_packets()).0
        }
        Role::Avionics => {
            static PRESSURE_CHANNEL: StaticChannel<[PressureTemp; 15], 10> = StaticChannel::new();
            let (pressure_sender, pressure_receiver) = PRESSURE_CHANNEL.split();
            #[allow(unreachable_code)]
            join!(
                buzzer_controller(),
                gps_handler(),
                gps_broadcast(),
                pressure_temp_handler(pressure_sensor.unwrap(), pr_timer, pressure_sender),
                handle_incoming_packets(),
                stage_update_handler(pressure_receiver)
            )
            .0
        }
        Role::Cansat => {
            static PRESSURE_CHANNEL: StaticChannel<[PressureTemp; 15], 10> = StaticChannel::new();
            let (pressure_sender, pressure_receiver) = PRESSURE_CHANNEL.split();
            #[allow(unreachable_code)]
            join!(
                buzzer_controller(),
                gps_handler(),
                gps_broadcast(),
                pressure_temp_handler(pressure_sensor.unwrap(), pr_timer, pressure_sender),
                handle_incoming_packets(),
                stage_update_handler(pressure_receiver),
                current_sensor_handler(current_sensor.unwrap(), i_timer)
            )
            .0
        }
    }
}
