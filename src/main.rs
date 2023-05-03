#![feature(async_closure)]
//! Demonstrate the use of a blocking `Delay` using the SYST (sysclock) timer.

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use core::fmt::Write;

use crate::hal::timer::TimerExt;
use cassette::pin_mut;
use cassette::Cassette;
use cortex_m::interrupt::Mutex;
use cortex_m_semihosting::hprintln;
use logger::setup_usb;

use core::cell::RefCell;
use core::panic::PanicInfo;
use logger::get_serial;
use logger::write_to::show;

use crate::hal::{pac, prelude::*};
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;

mod bmp581;
mod bno085;
mod futures;
mod gps;
mod logger;
mod radio;

static mut EP_MEMORY: [u32; 1024] = [0; 1024];
static CLOCKS: Mutex<RefCell<Option<hal::rcc::Clocks>>> = Mutex::new(RefCell::new(None));

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
    loop {}
}

async fn prog_main() {
    if let (Some(dp), Some(cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        let gpioe = dp.GPIOE.split();
        let gpiob = dp.GPIOB.split();
        let gpioa = dp.GPIOA.split();
        let mut pin = gpioe.pe10.into_push_pull_output();
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .sysclk(48.MHz())
            .use_hse(16.MHz())
            .require_pll48clk()
            .freeze();

        let mut delay = cp.SYST.delay(&clocks);
        let mut timer = dp.TIM2.counter_hz(&clocks);
        timer.start(4.MHz()).unwrap();

        hprintln!("Setting up USB");

        setup_usb(
            dp.OTG_FS_GLOBAL,
            dp.OTG_FS_DEVICE,
            dp.OTG_FS_PWRCLK,
            gpioa.pa11,
            gpioa.pa12,
            &clocks,
        );

        cortex_m::interrupt::free(|cs| {
            CLOCKS.borrow(cs).borrow_mut().replace(clocks);
        });

        /*let gps_serial = dp.USART1
        .serial(
            (gpioa.pa15.into_alternate(), gpioa.pa10.into_alternate()),
            hal::serial::config::Config {
                baudrate: 9600.bps(),
                dma: hal::serial::config::DmaConfig::TxRx,
                ..Default::default()
            },
            &clocks,
        )
        .unwrap();*/
        run_usb().await;
    }
}

async fn run_usb() -> ! {
    loop {
        let mut buf = [0u8; 64];
        match get_serial().read(&mut buf).await {
            Ok(count) if count > 0 => {
                writeln!(get_serial(), "Hi").unwrap();
            }
            _ => {}
        }
    }
}

/// Parses a hex #rrggbb string into an RGB8 color.
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

    loop {}
}
