use core::{borrow::BorrowMut, fmt::Write};
use fugit::RateExtU32;
use futures::join;
use heapless::String;
use nmea0183::{ParseResult, GGA};
use serde::__private::from_utf8_lossy;

use crate::{
    futures::YieldFuture,
    gps,
    logger::get_serial,
    radio::{self, Message, QUEUED_PACKETS, RECEIVED_MESSAGE_QUEUE},
    BUZZER, BUZZER_TIMER,
};

pub static mut ROLE: Role = Role::Cansat;

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum Role {
    Cansat,
    Avionics,
    Ground,
}

pub fn role() -> Role {
    // SAFETY: Role is mutated once by main prior to mission begin.
    unsafe { ROLE }
}

pub fn is_launched() -> bool {
    return false;
}

async fn usb_handler() -> ! {
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

async fn gps_broadcast() -> ! {
    let mut counter = 0;
    loop {
        let mut latitudes = [0.0; 8];
        let mut longitudes = [0.0; 8];
        let mut altitudes = [0.0; 8];

        let mut i = 0;
        while i < 8 {
            let fix = gps::next_sentence().await;
            if let ParseResult::GGA(Some(GGA { fix: Some(gga), .. })) = fix {
                latitudes[i] = gga.latitude.as_f64() as f32;
                longitudes[i] = gga.longitude.as_f64() as f32;
                altitudes[i] = gga.altitude.meters;
                i += 1;
            }
        }
        let message = Message::GpsBroadCast {
            counter,
            latitudes,
            longitudes,
            altitudes,
        };

        radio::queue_packet(message);
        counter = counter.wrapping_add(1);
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

pub async fn begin() {
    match unsafe { ROLE } {
        Role::Ground => {
            join!(usb_handler(), gps_handler(), handle_incoming_packets());
        }
        _ => {
            join!(gps_handler(), gps_broadcast());
        }
    }
}
