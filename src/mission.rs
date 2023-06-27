use core::{
    cell::{Cell},
    convert::Infallible,
    fmt::Write,
};
use cortex_m::interrupt::Mutex;
use embedded_hal::digital::v2::OutputPin;
use fugit::{ExtU32};
use futures::join;
use heapless::{Vec};
use nmea0183::{ParseResult, GGA};
use serde::{Deserialize, Serialize};
use stm32f4xx_hal::{
    adc::{config::SampleTime, Adc},
    gpio::{Analog, Output, Pin},
    pac::{ADC1, TIM12},
    timer::{CounterMs, Event},
};

use crate::{
    bmp581::{PressureTemp, BMP581},
    futures::{NbFuture, YieldFuture},
    gps,
    logger::get_serial,
    radio::{self, Message, RECEIVED_MESSAGE_QUEUE},
    sdio::get_logger,
    BUZZER, BUZZER_TIMER, RTC,
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
pub enum PyroState {
    BothFired,
    OneFired,
    NoneFired,
}

#[derive(Debug, PartialEq, Clone, Copy, Serialize, Deserialize)]
#[repr(u8)]
pub enum MissionStage {
    Disarmed(PyroState),
    Armed(PyroState),
    Ascent(PyroState),
    DescentDrogue(PyroState),
    DescentMain(PyroState),
    Landed(PyroState),
}

static STAGE: Mutex<Cell<MissionStage>> =
    Mutex::new(Cell::new(MissionStage::Disarmed(PyroState::NoneFired)));

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

fn modify_pyro_state(state: PyroState) {
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
        pyro_enable.set_low();

        let pyro = PYRO_MEASURE_PIN.as_mut().unwrap();
        let adc = PYRO_ADC.as_mut().unwrap();
        let sample = adc.convert(pyro, SampleTime::Cycles_56);
        adc.sample_to_millivolts(sample)
    };
    match mv {
        0..=2550 => modify_pyro_state(PyroState::NoneFired),
        2551..=2950 => modify_pyro_state(PyroState::OneFired),
        _ => modify_pyro_state(PyroState::BothFired),
    }
}

pub async fn update_stage(frames: [PressureTemp; 15]) {
    static mut START_ALTITUDE: f32 = 0.0;
    static mut SEA_LEVEL_PRESSURE: f32 = 0.0;

    if unsafe { SEA_LEVEL_PRESSURE } == 0.0 {
        unsafe {
            SEA_LEVEL_PRESSURE = frames
                .map(|frame| {
                    let pressure = frame.pressure as f32 / libm::powf(2.0, 6.0);
                    pressure
                })
                .iter()
                .sum::<f32>()
                / frames.len() as f32;
        }
    }

    let stage = current_stage();
    let altitudes = frames.map(|frame| {
        let pressure = frame.pressure as f32 / libm::powf(2.0, 6.0);
        let temperature = frame.temperature as f32 / libm::powf(2.0, 16.0);

        let altitude = (libm::powf(unsafe { SEA_LEVEL_PRESSURE } / pressure, 1.0 / 5.257) - 1.0)
            * (temperature + 273.15)
            / 0.0065;
        altitude
    });

    if unsafe { START_ALTITUDE } == 0.0 {
        // If we don't have a start pressure, we can't do anything
        unsafe {
            START_ALTITUDE = altitudes.clone().iter().sum::<f32>() / altitudes.len() as f32;
        }
        return;
    }

    let new_stage = match stage {
        MissionStage::Disarmed(s) => {
            /* Nothing to update, just wait for arm command */
            MissionStage::Disarmed(s)
        }
        MissionStage::Armed(s) => {
            // If we're above 15m, we must be ascending
            if altitudes
                .iter()
                .all(|a| *a > 15.0 + unsafe { START_ALTITUDE })
            {
                MissionStage::Ascent(s)
            } else {
                MissionStage::Armed(s)
            }
        }
        MissionStage::Ascent(s) => {
            // If our velocity is negative, we must be descending
            let velocities = altitudes.windows(3).map(|w| w[2] - w[0]);

            if velocities.filter(|v| *v < 0.05).count() > 10 {
                if role() == Role::Avionics {
                    // If we're the avionics, fire the pyro
                    fire_pyro(PyroPin::One, 1000).await;
                }
                MissionStage::DescentDrogue(s)
            } else {
                MissionStage::Ascent(s)
            }
        }
        MissionStage::DescentDrogue(s) => {
            // At 300m, fire main
            if altitudes.iter().all(|a| *a < 320.0) {
                if role() == Role::Avionics {
                    // If we're the avionics, fire the pyro
                    fire_pyro(PyroPin::Two, 1000).await;
                }
                MissionStage::DescentMain(s)
            } else {
                MissionStage::DescentDrogue(s)
            }
        }
        MissionStage::DescentMain(s) => {
            // If our average velocity is less than 1m/s, we must have landed
            let velocities_abs = altitudes.windows(2).map(|w| libm::fabsf(w[1] - w[0]));
            if velocities_abs.clone().sum::<f32>() / (velocities_abs.len() as f32) < 0.01 {
                if role() == Role::Avionics {
                    // If we're the avionics, fire the pyro
                    fire_pyro(PyroPin::One, 1000).await;
                    fire_pyro(PyroPin::Two, 1000).await;
                }

                MissionStage::Landed(s)
            } else {
                MissionStage::DescentMain(s)
            }
        }
        MissionStage::Landed(s) => {
            // Nothing to do
            MissionStage::Landed(s)
        }
    };

    cortex_m::interrupt::free(|cs| STAGE.borrow(cs).set(new_stage));
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
                        STAGE
                            .borrow(cs)
                            .set(MissionStage::Disarmed(PyroState::NoneFired));
                    });
                    for _ in 0..5 {
                        radio::queue_packet(Message::Disarm(role, idempotency_counter));
                    }
                    idempotency_counter += 1;
                }
                None => writeln!(get_serial(), "Invalid disarm role").unwrap(),
            }
        } else if split.len() > 1 && split[0].starts_with(b"arm") {
            match parse_role(split[1]) {
                Some(role) => {
                    for _ in 0..5 {
                        radio::queue_packet(Message::Arm(role, idempotency_counter));
                    }
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
                MissionStage::Ascent(PyroState::NoneFired)
            } else {
                MissionStage::Disarmed(PyroState::NoneFired)
            };

            cortex_m::interrupt::free(|cs| STAGE.borrow(cs).set(new_state));
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

async fn pressure_temp_handler(mut sensor: BMP581, mut timer: CounterMs<TIM12>) {
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
                update_stage(frames).await;
            } else {
            }
            timer.clear_interrupt(Event::Update);
            timer.start(350.millis()).unwrap();
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

                        writeln!(get_serial(), "rssi,{}", rssi).unwrap();
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

                        writeln!(get_serial(), "rssi,{}", rssi).unwrap();
                    }
                    Message::Arm(..) => {}
                    Message::Disarm(..) => {}
                    Message::TestPyro(..) => {}
                },
                _ => match packet {
                    Message::Disarm(r, i) if r == role() && i > idempotency_counter => {
                        idempotency_counter += 1;
                        disarm().await;
                    }
                    Message::Arm(r, i) if r == role() && i > idempotency_counter => {
                        idempotency_counter += 1;
                        arm().await;
                    }
                    Message::TestPyro(r, pin, duration) if r == role() => {
                        fire_pyro(pin, duration).await;
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
    let timer = unsafe { BUZZER_TIMER.as_mut().unwrap() };
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
        NbFuture::new(||timer.wait()).await.unwrap();
        buzz.set_low();
        timer.start((time as u32).millis()).unwrap();
        NbFuture::new(||timer.wait()).await.unwrap();
        time = (time as f32 * 0.8 - 5.0) as i32;
    }

    buzz.set_high();

    timer.start(500u32.millis()).unwrap();
    NbFuture::new(||timer.wait()).await.unwrap();
    buzz.set_low();
    for fire in &mut pyro_fires {
        fire.set_high().unwrap();
    }
    timer.start(duration.millis()).unwrap();
    NbFuture::new(||timer.wait()).await.unwrap();
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
        STAGE
            .borrow(cs)
            .replace(MissionStage::Disarmed(PyroState::NoneFired));
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
        STAGE
            .borrow(cs)
            .replace(MissionStage::Armed(PyroState::NoneFired));
        update_pyro_state();
    });
    timer.clear_interrupt(Event::Update);
    timer.start(300u32.millis()).unwrap();
    NbFuture::new(|| timer.wait()).await.unwrap();
    buzz.set_low()
}

pub async fn begin(pressure_sensor: Option<BMP581>, pr_timer: CounterMs<TIM12>) -> ! {
    match unsafe { ROLE } {
        Role::Ground =>
        {
            #[allow(unreachable_code)]
            join!(usb_handler(), gps_handler(), handle_incoming_packets()).0
        }
        _ =>
        {
            #[allow(unreachable_code)]
            join!(
                gps_handler(),
                gps_broadcast(),
                pressure_temp_handler(pressure_sensor.unwrap(), pr_timer),
                handle_incoming_packets()
            )
            .0
        }
    }
}
