use core::{
    borrow::BorrowMut,
    cell::{Cell, RefCell},
    fmt::Write,
    sync::atomic::{AtomicBool, Ordering},
};
use cortex_m::interrupt::Mutex;
use embedded_hal::{blocking::i2c, prelude::_embedded_hal_blocking_i2c_WriteRead};
use fugit::RateExtU32;
use futures::join;
use heapless::String;
use nmea0183::{ParseResult, GGA};
use serde::{Deserialize, Serialize, __private::from_utf8_lossy};
use smart_leds::SmartLedsWrite;
use stm32f4xx_hal::{
    adc::{config::SampleTime, Adc},
    gpio::{Analog, Pin},
    pac::ADC1,
};

use crate::{
    bmp581::BMP581,
    futures::{NbFuture, YieldFuture},
    gps,
    logger::get_serial,
    radio::{self, Message, QUEUED_PACKETS, RECEIVED_MESSAGE_QUEUE},
    BUZZER, BUZZER_TIMER, NEOPIXEL, RTC,
};

pub static mut ROLE: Role = Role::Cansat;
pub static mut PYRO_ADC: Option<Adc<ADC1>> = None;
pub static mut PYRO_MEASURE_PIN: Option<Pin<'C', 0, Analog>> = None;
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

pub fn update_pyro_state() {
    let mv = unsafe {
        let pyro = PYRO_MEASURE_PIN.as_mut().unwrap();
        let adc = PYRO_ADC.as_mut().unwrap();
        let sample = adc.convert(pyro, SampleTime::Cycles_56);
        adc.sample_to_millivolts(sample)
    };
    match mv {
        0..=2550 => {
            cortex_m::interrupt::free(|cs| {
                STAGE
                    .borrow(cs)
                    .set(MissionStage::Disarmed(PyroState::NoneFired));
            });
        }
        2551..=2950 => {
            cortex_m::interrupt::free(|cs| {
                STAGE
                    .borrow(cs)
                    .set(MissionStage::Disarmed(PyroState::OneFired));
            });
        }
        _ => {
            cortex_m::interrupt::free(|cs| {
                STAGE
                    .borrow(cs)
                    .set(MissionStage::Disarmed(PyroState::BothFired));
            });
        }
    }
}

pub fn current_stage() -> MissionStage {
    update_pyro_state();
    cortex_m::interrupt::free(|cs| STAGE.borrow(cs).get())
}

pub async fn usb_handler() -> ! {
    let mut buf = [0u8; 256];
    loop {
        let bytes = get_serial().read(&mut buf).await.unwrap();
        if bytes.starts_with(b"disarm") {
            for i in 0..10 {
                radio::queue_packet(Message::Disarm);
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
        while i < 10 {
            if i == 0 {
                let time = current_rtc_time();
                start_time = Some(time.0 as u32 * 3600 + time.1 as u32 * 60 + time.2 as u32);
            }
            let fix = gps::next_sentence().await;
            if let ParseResult::GGA(Some(GGA { fix: Some(gga), .. })) = fix {
                latitudes[i] = gga.latitude.as_f64() as f32;
                longitudes[i] = gga.longitude.as_f64() as f32;
                altitudes[i] = gga.altitude.meters;
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

        let time = current_rtc_time();
        let mut time = time.0 as u32 * 3600 + time.1 as u32 * 60 + time.2 as u32;
        // GPS provides packets at 10Hz and we're targeting 5Hz so we need to wait 2 seconds.
        while time - start_time.unwrap() < 2 {
            gps::next_sentence().await;
            let time_ = current_rtc_time();
            time = time_.0 as u32 * 3600 + time_.1 as u32 * 60 + time_.2 as u32;
        }
    }
}

async fn pressure_temp_broadcast<I2C, E>(mut sensor: BMP581<'_, E, I2C>)
where
    I2C: i2c::Write<u8, Error = E> + i2c::WriteRead<u8, Error = E>,
{
    loop {
        let mut pressures = [0.0; 10];
        let mut temperatures = [0.0; 10];

        let mut i = 0;
        let mut start_time = None;
        while i < 10 {
            if i == 0 {
                let time = current_rtc_time();
                start_time = Some(time.0 as u32 * 3600 + time.1 as u32 * 60 + time.2 as u32);
            }
            let pressure = sensor.pressure();
            let temperature = sensor.temperature();
            if let (Ok(pressure), Ok(temperature)) = (pressure, temperature) {
                pressures[i] = pressure as f32 / libm::powf(2.0, 6.0);
                temperatures[i] = temperature as f32 / libm::powf(2.0, 16.0);
                i += 1;
            }
        }
        let message = Message::PressureTempBroadCast {
            counter: radio::next_counter(),
            stage: current_stage(),
            role: role(),
            time_of_first_packet: start_time.unwrap(),
            pressures,
            temperatures,
        };

        radio::queue_packet(message);

        let time = current_rtc_time();
        let mut time = time.0 as u32 * 3600 + time.1 as u32 * 60 + time.2 as u32;
        // GPS provides packets at 10Hz and we're targeting 5Hz so we need to wait 2 seconds.
        while time - start_time.unwrap() < 2 {
            YieldFuture::new().await;
            let time_ = current_rtc_time();
            time = time_.0 as u32 * 3600 + time_.1 as u32 * 60 + time_.2 as u32;
        }
    }
}

async fn handle_incoming_packets() -> ! {
    loop {
        cortex_m::interrupt::free(|cs| {
            if let Some(packet) = RECEIVED_MESSAGE_QUEUE.borrow(cs).borrow_mut().pop_front() {
                writeln!(get_serial(), "Received: {:?}", packet).unwrap();
            }
        });
        YieldFuture::new().await;
    }
}

pub fn disarm() {
    unsafe { BUZZER.as_mut().unwrap().set_high() }
    unsafe { BUZZER_TIMER.as_mut().unwrap().start(1u32.Hz()).unwrap() };
    unsafe { nb::block!(BUZZER_TIMER.as_mut().unwrap().wait()).unwrap() };
    unsafe { BUZZER.as_mut().unwrap().set_low() };
}

pub async fn test_launch() {
    let mut count = 0;
    unsafe {
        BUZZER_TIMER.as_mut().unwrap().start(1.Hz()).unwrap();
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

pub async fn begin<E, I2C>(pressure_sensor: BMP581<'_, E, I2C>) -> !
where
    I2C: i2c::Write<u8, Error = E> + i2c::WriteRead<u8, Error = E>,
{
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
                pressure_temp_broadcast(pressure_sensor)
            )
            .0
        }
    }
}
