#![feature(async_closure)]
//! Demonstrate the use of a blocking `Delay` using the SYST (sysclock) timer.

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use crate::futures::NbFuture;
use crate::hal::timer::TimerExt;
use cassette::pin_mut;
use cassette::Cassette;
use cortex_m::interrupt::Mutex;
use embedded_hal::digital::v2::OutputPin;
use hal::gpio::Pin;
use hal::i2c::I2c;
use hal::pac::SDIO;
use hal::sdio::ClockFreq;
use hal::sdio::SdCard;
use hal::sdio::Sdio;
use hal::timer::CounterHz;
use hal::timer::CounterMs;

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
static mut ERROR_LED: Option<Mutex<RefCell<gpio::Pin<'C', 1, Output<PushPull>>>>> = None;

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
        let gpiod = dp.GPIOD.split();
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

        let mut neo_pin = gpioc.pc0.into_push_pull_output();
        neo_pin.set_speed(Speed::High);
        let neopixel = Ws2812::new(timer, neo_pin);

        let mut delay = cp.SYST.delay(&clocks);

        let d0 = gpioc.pc8.into_alternate().internal_pull_up(true);
        let d1 = gpioc.pc9.into_alternate().internal_pull_up(true);
        let d2 = gpioc.pc10.into_alternate().internal_pull_up(true);
        let d3 = gpioc.pc11.into_alternate().internal_pull_up(true);
        let clk = gpioc.pc12.into_alternate().internal_pull_up(false);
        let cmd = gpiod.pd2.into_alternate().internal_pull_up(true);


        let f = run_usb(neopixel, clk, cmd, d0, d1, d2, d3, &clocks, delay, dp.SDIO);
        let mut timer2 = dp.TIM3.counter_hz(&clocks);
        let f2 = blink_led(&mut timer2);

        ::futures::future::join(f, f2).await;
    }
}

async fn sd_card_shit(
    clk: Pin<'C', 12, gpio::Alternate<12>>,
    cmd: Pin<'D', 2, gpio::Alternate<12>>,
    d0: Pin<'C', 8, gpio::Alternate<12>>,
    d1: Pin<'C', 9, gpio::Alternate<12>>,
    d2: Pin<'C', 10, gpio::Alternate<12>>,
    d3: Pin<'C', 11, gpio::Alternate<12>>,
    clocks: &hal::rcc::Clocks,
    mut delay: hal::timer::SysDelay,
    sdio_port: SDIO,
) {
    let mut sdio: Sdio<SdCard> = Sdio::new(sdio_port, (clk, cmd, d0, d1, d2, d3), clocks);
    get_serial().log("Waiting for card...\n").await;
    // Wait for card to be ready
    loop {
        match sdio.init(ClockFreq::F24Mhz) {
            Ok(_) => break,
            Err(_err) => (),
        }

        delay.delay_ms(1000u32);
    }
    let mut buffer = [0; 256];
    let nblocks = sdio.card().map(|c| c.block_count()).unwrap_or(0);
    get_serial()
        .log(
            show(
                &mut buffer,
                format_args!("Card detected: nbr of blocks: {:?}\n", nblocks),
            )
            .unwrap(),
        )
        .await;
    // Read a block from the card and print the data
    let mut block = [0u8; 512];
    match sdio.read_block(0, &mut block) {
        Ok(()) => (),
        Err(err) => {
            panic!("Error reading block: {:?}\n", err);
        }
    }
    for b in block.iter() {
        get_serial()
            .log(show(&mut buffer, format_args!("{:?}", b)).unwrap())
            .await;
    }

    get_serial().log("\n").await;
}

async fn run_usb(
    mut neopixel: Ws2812<hal::timer::CounterHz<pac::TIM2>, gpio::Pin<'C', 0, Output<PushPull>>>,

    clk: Pin<'C', 12, gpio::Alternate<12>>,
    cmd: Pin<'D', 2, gpio::Alternate<12>>,
    d0: Pin<'C', 8, gpio::Alternate<12>>,
    d1: Pin<'C', 9, gpio::Alternate<12>>,
    d2: Pin<'C', 10, gpio::Alternate<12>>,
    d3: Pin<'C', 11, gpio::Alternate<12>>,
    clocks: &hal::rcc::Clocks,
    delay: hal::timer::SysDelay,
    sdio_port: SDIO,
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

                if buf[0..count].starts_with(b"sd") {
                    sd_card_shit(clk, cmd, d0, d1, d2, d3, clocks, delay, sdio_port).await;
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
