
use futures::join;
use nmea0183::{ParseResult, GGA};

use crate::{logger::get_serial, gps, radio::{Message, self}};

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

async fn usb_handler() {
    let mut buf = [0u8; 256];
    loop {
        get_serial().read(&mut buf).await;
    }
}

async fn gps_handler() {
    gps::poll_for_sentences().await;
}

async fn gps_broadcast() {
    loop {
        let mut latitudes = [0.0; 8];
        let mut longitudes = [0.0; 8];
        let mut altitudes = [0.0; 8];

        for i in 0..8 {
            let fix = gps::next_sentence().await;
            // FIXME: This broadcasts even if there is no fix (useful for testing but not for real flights)
            if let ParseResult::GGA(Some(GGA { fix: Some(gga), .. })) = fix {
                latitudes[i] = gga.latitude.as_f64() as f32;
                longitudes[i] = gga.longitude.as_f64() as f32;
                altitudes[i] = gga.altitude.meters;
            }
        }
        let message = Message::GpsBroadCast {
            latitudes,
            longitudes,
            altitudes,
        };
        for i in 0..5 {
            radio::queue_packet(message);
        }
    }
}

pub async fn begin() {
    match unsafe { ROLE } {
        Role::Ground => {
            radio::setup_ground_radio();
            join!(usb_handler());
        }
        _ => {
            join!(usb_handler(), gps_handler(), gps_broadcast());
        }
    }
}