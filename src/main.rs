//! Demonstrate the use of a blocking `Delay` using the SYST (sysclock) timer.

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use hal::gpio::Output;
use hal::gpio::Pin;
use hal::gpio::PushPull;
use hal::gpio::Speed;
use logger::get_serial;
use logger::setup_usb;
use logger::write_to::show;
use core::panic::PanicInfo;

use cortex_m_rt::entry;
use smart_leds::SmartLedsWrite;
use smart_leds::RGB8;
use stm32f4xx_hal as hal;
use ws2812_timer_delay::Ws2812;
use crate::hal::{pac, prelude::*};

mod logger;

static mut ERROR_LED: Option<Pin<Output<PushPull>, 'C', 1>> = None;

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {

        let gpioc = dp.GPIOC.split();
        let mut led = gpioc.pc1.into_push_pull_output();
        led.set_high();
        // SAFETY: error_led is only mutated once at initialization.
        unsafe { ERROR_LED = Some(led); }

        // Set up the system clock.
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(12.MHz())
            .sysclk(168.MHz())
            .require_pll48clk()
            .freeze();


        let gpioa = dp.GPIOA.split();

        setup_usb(
            dp.OTG_FS_GLOBAL,
            dp.OTG_FS_DEVICE,
            dp.OTG_FS_PWRCLK,
            gpioa.pa11,
            gpioa.pa12,
            &clocks,
        );

        let mut timer = dp.TIM2.counter_hz(&clocks);

        timer.start(3.MHz()).unwrap();

        let neo_pin = gpioc.pc0.into_push_pull_output().set_speed(Speed::High);
        let mut neopixel = Ws2812::new(timer, neo_pin);

        loop {
            if !get_serial().poll() {
                continue;
            }

            let mut buf = [0u8; 64];
            match get_serial().read(&mut buf) {
                Ok(count) if count > 0 => {
                    // Skips newlines.
                    // FIXME: This is a bit of a hack.     
                    if count == 2 && buf[0] == b'\r' && buf[1] == b'\n' {
                        continue;
                    }

                    if buf[0..count].starts_with(b"panic!") {
                        panic!("at the disco");
                    }

                    let rgb = parse_rgb_hex(&buf[0..count]).expect("invalid rgb");
                    neopixel.write([rgb].iter().cloned()).unwrap();
                }
                _ => {}
            }
        }
    }

    loop {}
}

/// Parses a hex #rrggbb string into an RGB8 color.
fn parse_rgb_hex(hex: &[u8]) -> Result<RGB8, ()> {
    let mut rgb = RGB8 { r: 0, g: 0, b: 0 };
    if hex.len() != 6 {
        return Err(());
    }

    let hex = core::str::from_utf8(&hex[0..6]).map_err(|_| ())?;

    rgb.r = u8::from_str_radix(&hex[0..2], 16).map_err(|_| ())?;
    rgb.g = u8::from_str_radix(&hex[2..4], 16).map_err(|_| ())?;
    rgb.b = u8::from_str_radix(&hex[4..6], 16).map_err(|_| ())?;

    get_serial().log(
        show(
            &mut [0u8; 64],
            format_args!("{} {} {}\n", rgb.r, rgb.g, rgb.b),
        )
        .unwrap(),
    );        
    Ok(rgb)
}

#[panic_handler]
pub fn panic(info: &PanicInfo) -> ! {
    if let Some(serial) = logger::try_get_serial() {
        serial.log("A panic! occured! The system has been halted!\n");
        let mut buffer = [0u8; 1024];
        serial.log(show(&mut buffer, format_args!("{}\n", info)).unwrap_or("Formatting error occured!\n"));
    }

    // SAFETY: error_led is only mutated once at initialization.
    unsafe {
        if let Some(led) = ERROR_LED.as_mut() {
            led.set_low();
        }
    }

    
    loop {}
}