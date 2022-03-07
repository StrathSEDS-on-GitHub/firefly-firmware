#![feature(async_closure)]
//! Demonstrate the use of a blocking `Delay` using the SYST (sysclock) timer.

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use crate::futures::NbFuture;
use cassette::pin_mut;
use cassette::Cassette;
use cortex_m::interrupt::Mutex;
use hal::timer::CounterHz;

use core::cell::RefCell;
use core::panic::PanicInfo;
use hal::gpio;
use hal::gpio::Output;
use hal::gpio::PushPull;
use hal::gpio::Speed;
use logger::get_serial;
use logger::setup_usb;
use logger::write_to::show;

use crate::hal::{pac, prelude::*};
use cortex_m_rt::entry;
use smart_leds::SmartLedsWrite;
use smart_leds::RGB8;
use stm32f4xx_hal as hal;
use ws2812_timer_delay::Ws2812;

mod futures;
mod logger;

// SAFETY: It's properly Mutex'd and RefCell'd, so everything should be fine
static mut ERROR_LED: Option<Mutex<RefCell<gpio::Pin<Output<PushPull>, 'C', 1>>>> = None;

#[entry]
fn main() -> ! {
    let x = prog_main();
    pin_mut!(x);
    let mut cm = Cassette::new(x);
    loop {
        if let Some(_) = cm.poll_on() {
            break;
        }
    }
    cortex_m::interrupt::free(|cs| unsafe {
        ERROR_LED
            .as_ref()
            .unwrap()
            .borrow(cs)
            .borrow_mut()
            .set_low();
    });
    loop {}
}

async fn blink_led(timer: &mut CounterHz<pac::TIM3>) {
    timer.start(1.Hz()).unwrap();
    loop {
        NbFuture::new(|| timer.wait()).await.unwrap();

        cortex_m::interrupt::free(|cs| unsafe {
            ERROR_LED
                .as_ref()
                .unwrap()
                .borrow(cs)
                .borrow_mut()
                .set_low();
        });
        NbFuture::new(|| timer.wait()).await.unwrap();
        cortex_m::interrupt::free(|cs| unsafe {
            ERROR_LED
                .as_ref()
                .unwrap()
                .borrow(cs)
                .borrow_mut()
                .set_high();
        });
        //show("Hello, world!");
    }
}

async fn prog_main() {
    if let (Some(dp), Some(cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        let gpioc = dp.GPIOC.split();
        let mut led = gpioc.pc1.into_push_pull_output();
        led.set_high();
        // SAFETY: error_led is only mutated once at initialization.
        unsafe {
            ERROR_LED = Some(Mutex::new(RefCell::new(led)));
        }

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
        let neopixel = Ws2812::new(timer, neo_pin);

        let f = run_usb(neopixel);
        let mut timer2 = dp.TIM3.counter_hz(&clocks);
        let f2 = blink_led(&mut timer2);

        ::futures::future::join(f, f2).await;
    }
}

async fn run_usb(
    mut neopixel: Ws2812<hal::timer::CounterHz<pac::TIM2>, gpio::Pin<Output<PushPull>, 'C', 0>>,
) -> ! {
    loop {
        let mut buf = [0u8; 64];
        match get_serial().read(&mut buf).await {
            Ok(count) if count > 0 => {
                // Skips newlines.
                // FIXME: This is a bit of a hack.
                if count == 2 && buf[0] == b'\r' && buf[1] == b'\n' {
                    continue;
                }

                if buf[0..count].starts_with(b"panic!") {
                    panic!("at the disco");
                }

                let mut buf2 = [0u8; 128];
                let rgb = parse_rgb_hex(&buf[0..count]).await;
                if let Ok(rgb) = rgb {
                    neopixel.write([rgb].iter().cloned()).unwrap();
                } else {
                    get_serial()
                        .log(
                            show(
                                &mut buf2,
                                format_args!("invalid rgb {:?}\n", &buf[0..count]),
                            )
                            .unwrap(),
                        )
                        .await;
                }
            }
            _ => {}
        }
    }
}

/// Parses a hex #rrggbb string into an RGB8 color.
async fn parse_rgb_hex(hex: &[u8]) -> Result<RGB8, ()> {
    let mut rgb = RGB8 { r: 0, g: 0, b: 0 };
    if hex.len() != 6 {
        return Err(());
    }

    let hex = core::str::from_utf8(&hex[0..6]).map_err(|_| ())?;

    rgb.r = u8::from_str_radix(&hex[0..2], 16).map_err(|_| ())?;
    rgb.g = u8::from_str_radix(&hex[2..4], 16).map_err(|_| ())?;
    rgb.b = u8::from_str_radix(&hex[4..6], 16).map_err(|_| ())?;

    get_serial()
        .log(
            show(
                &mut [0u8; 64],
                format_args!("{} {} {}\n", rgb.r, rgb.g, rgb.b),
            )
            .unwrap(),
        )
        .await;
    Ok(rgb)
}

#[panic_handler]
pub fn panic(info: &PanicInfo) -> ! {
    if let Some(serial) = logger::try_get_serial() {
        {
            let f = serial.log("A panic! occured! The system has been halted!\n");
            pin_mut!(f);
            let mut cm = Cassette::new(f);
            while cm.poll_on().is_none() {}
        }

        let mut buffer = [0u8; 1024];

        {
            let f = serial.log(
                show(&mut buffer, format_args!("{}\n", info))
                    .unwrap_or("Formatting error occured!\n"),
            );
            pin_mut!(f);
            let mut cm = Cassette::new(f);
            while cm.poll_on().is_none() {}
        }
    }

    // SAFETY: error_led is only mutated once at initialization.
    cortex_m::interrupt::free(|cs| unsafe {
        ERROR_LED
            .as_ref()
            .unwrap()
            .borrow(cs)
            .borrow_mut()
            .set_low();
    });

    loop {}
}
