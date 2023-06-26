use core::{
    borrow::BorrowMut,
    cell::{Cell, RefCell},
    convert::Infallible,
    fmt::Write,
    sync::atomic::{AtomicBool, Ordering},
};
use cortex_m::interrupt::Mutex;
use cortex_m_semihosting::hprintln;
use embedded_hal::{blocking::i2c, prelude::_embedded_hal_blocking_i2c_WriteRead};
use embedded_hal::{blocking::i2c::WriteRead, digital::v2::OutputPin};
use fugit::{ExtU32, RateExtU32};
use futures::join;
use heapless::{String, Vec};
use nmea0183::{ParseResult, GGA};
use serde::{Deserialize, Serialize, __private::from_utf8_lossy};
use smart_leds::SmartLedsWrite;
use stm32f4xx_hal::{
    adc::{config::SampleTime, Adc},
    gpio::{Analog, Output, Pin, PushPull},
    i2c::{
        dma::{DMATransfer, Error},
        Instance,
    },
    pac::{ADC1, TIM12},
    timer::{CounterMs, Event},
};

use crate::{
    bmp581::BMP581,
    futures::{NbFuture, YieldFuture},
    gps,
    logger::get_serial,
    radio::{self, Message, QUEUED_PACKETS, RECEIVED_MESSAGE_QUEUE},
    sdio::get_logger,
    BUZZER, BUZZER_TIMER, NEOPIXEL, RTC,
};
use stm32f4xx_hal::i2c::dma::I2CMasterWriteReadDMA;

pub static mut ROLE: Role = Role::Cansat;
pub static mut PYRO_ADC: Option<Adc<ADC1>> = None;
pub static mut PYRO_MEASURE_PIN: Option<Pin<'C', 0, Analog>> = None;
pub static mut PYRO_ENABLE_PIN: Option<Pin<'D', 7, Output>> = None;
pub static mut PYRO_FIRE2: Option<Pin<'D', 6, Output>> = None;
pub static mut PYRO_FIRE1: Option<Pin<'D', 5, Output>> = None;

static LAUNCHED: AtomicBool = AtomicBool::new(false);

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
    Accelerating(PyroState),
    Coast(PyroState),
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
    return LAUNCHED.load(Ordering::Relaxed);
}

fn modify_pyro_state(state: PyroState) {
    cortex_m::interrupt::free(|cs| {
        let stage = STAGE.borrow(cs).get();
        STAGE.borrow(cs).set(match stage {
            MissionStage::Disarmed(_) => MissionStage::Disarmed(state),
            MissionStage::Armed(_) => MissionStage::Armed(state),
            MissionStage::Accelerating(_) => MissionStage::Accelerating(state),
            MissionStage::Coast(_) => MissionStage::Coast(state),
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
        pyro_enable.set_high();
        adc.sample_to_millivolts(sample)
    };
    match mv {
        0..=2550 => {
            modify_pyro_state(PyroState::NoneFired)
        }
        2551..=2950 => {
            modify_pyro_state(PyroState::OneFired)
        }
        _ => {
            modify_pyro_state(PyroState::BothFired)
        }
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
    loop {
        let bytes = get_serial().read(&mut buf).await.unwrap();

        let split: Vec<_, 32> = bytes.split(|b| *b == b',').collect();
        if split.len() > 1 && split[0].starts_with(b"disarm") {
            match parse_role(split[1]) {
                Some(role) => {
                    radio::queue_packet(Message::Disarm(role));
                }
                None => writeln!(get_serial(), "Invalid disarm role").unwrap(),
            }
        } else if split.len() > 1 && split[0].starts_with(b"arm") {
            match parse_role(split[1]) {
                Some(role) => {
                    radio::queue_packet(Message::Arm(role));
                }
                None => writeln!(get_serial(), "Invalid arm role").unwrap(),
            }
        } else if split.len() > 3 && split[0].starts_with(b"test-pyro") {
            if let Some(role) = parse_role(split[1]) {
                let pin = if split[2].starts_with(b"1") {
                    1
                } else if split[2].starts_with(b"2") {
                    2
                } else if split[2].starts_with(b"3") {
                    3
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
        }

        writeln!(get_serial(), "Received: {:?}", core::str::from_utf8(bytes)).unwrap();
        YieldFuture::new().await;
    }
}

async fn gps_handler() -> ! {
    gps::poll_for_sentences().await
}

fn current_rtc_time() -> (u8, u8, u8) {
    cortex_m::interrupt::free(|cs| {
        let mut rtc_ref = RTC.borrow(cs).borrow_mut();
        let rtc = rtc_ref.as_mut().unwrap();
        let date_time = rtc.get_datetime();
        date_time.as_hms()
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
                start_time = Some(time.0 as u32 * 3600 + time.1 as u32 * 60 + time.2 as u32);
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
    loop {
        let mut pressures = [0.0; 8];
        let mut temperatures = [0.0; 8];

        let time = current_rtc_time();
        let start_time = Some(time.0 as u32 * 3600 + time.1 as u32 * 60 + time.2 as u32);

        let mut i = 0;

        while i < 4 * 8 {
            if let Ok(frames) = sensor.read_fifo() {
                for frame in frames {
                    let pressure = frame.pressure as f32 / libm::powf(2.0, 6.0);
                    let temperature = frame.temperature as f32 / libm::powf(2.0, 16.0);

                    // Send every fourth frame (10Hz).
                    if i % 4 == 0 {
                        pressures[i / 4] = pressure;
                        temperatures[i / 4] = temperature;
                    }

                    get_logger().log(format_args!("pressure,{},{}", pressure, temperature));

                    i += 1;
                }
            } else {
            }
            timer.clear_interrupt(Event::Update);
            timer.start(100.millis()).unwrap();
            NbFuture::new(|| timer.wait()).await.unwrap();
        }

        let message = Message::PressureTempBroadCast {
            counter: radio::next_counter(),
            stage: current_stage(),
            role: role(),
            time_of_first_packet: start_time.unwrap(),
            pressures,
            temperatures,
        };

        // writeln!(get_serial(), "Sending: {:?}", message).unwrap();

        radio::queue_packet(message);
    }
}

async fn handle_incoming_packets() -> ! {
    loop {
        if let Some(packet) = cortex_m::interrupt::free(|cs| {
            RECEIVED_MESSAGE_QUEUE.borrow(cs).borrow_mut().pop_front()
        }) {
            match role() {
                Role::Ground => {
                    writeln!(get_serial(), "Received: {:?}", packet).unwrap();
                }
                _ => match packet {
                    Message::Disarm(r) if r == role() => disarm().await,
                    Message::Arm(r) if r == role() => arm().await,
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

pub async fn fire_pyro(pin: u8, duration: u32) {
    let buzz = unsafe { BUZZER.as_mut().unwrap() };
    let timer = unsafe { BUZZER_TIMER.as_mut().unwrap() };
    let mut pyro_fires = {
        let mut vec: Vec<&mut dyn OutputPin<Error = Infallible>, 2> = Vec::new();
        let _ = match pin {
            1 => {
                vec.push(unsafe { PYRO_FIRE1.as_mut().unwrap() });
            }
            2 => {
                vec.push(unsafe { PYRO_FIRE2.as_mut().unwrap() });
            }
            3 => {
                vec.push(unsafe { PYRO_FIRE1.as_mut().unwrap() });
                vec.push(unsafe { PYRO_FIRE2.as_mut().unwrap() });
            }
            _ => return,
        };
        vec
    };

    let mut time = 200i32;
    while time > 0 {
        buzz.set_high();
        timer.start(200u32.millis());
        nb::block!(timer.wait());
        buzz.set_low();
        timer.start((time as u32).millis());
        nb::block!(timer.wait());
        time = (time as f32 * 0.8 - 5.0) as i32;
    }

    buzz.set_high();

    timer.start(1000u32.millis());
    nb::block!(timer.wait());
    buzz.set_low();
    for fire in &mut pyro_fires {
        fire.set_high();
    }
    timer.start(duration.millis());
    nb::block!(timer.wait());
    for fire in &mut pyro_fires {
        fire.set_low();
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

pub async fn test_launch() {
    let mut count = 0;
    unsafe {
        BUZZER_TIMER
            .as_mut()
            .unwrap()
            .start(1000u32.millis())
            .unwrap();
    }

    while count < 10 {
        unsafe {
            NbFuture::new(|| BUZZER_TIMER.as_mut().unwrap().wait())
                .await
                .unwrap()
        };
        count += 1;
    }

    LAUNCHED.store(true, Ordering::Relaxed);
    cortex_m::interrupt::free(|cs| {
        NEOPIXEL
            .borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .write([[0, 50, 0]; 2].into_iter())
            .unwrap();
    });
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
