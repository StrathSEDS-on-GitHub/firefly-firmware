#![feature(async_closure)]
//! Demonstrate the use of a blocking `Delay` using the SYST (sysclock) timer.

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use core::fmt::Write;
use core::sync::atomic;

use crate::futures::NbFuture;
use crate::hal::timer::TimerExt;
use crate::logger::write_to;
use ::futures::join;
use cassette::pin_mut;
use cassette::Cassette;
use cortex_m::interrupt::Mutex;
use cortex_m_semihosting::hprintln;
use dummy_pin::DummyPin;
use hal::gpio;
use hal::gpio::Input;
use hal::gpio::gpioc;
use hal::interrupt;
use hal::spi;
use logger::setup_usb;
use smart_leds::RGB8;
use smart_leds::SmartLedsWrite;
use smart_leds::gamma;
use smart_leds::hsv::Hsv;
use smart_leds::hsv::hsv2rgb;
use sx126x::op::packet;
use sx126x::op::CalibParam;
use sx126x::op::IrqMask;
use sx126x::op::PaConfig;
use sx126x::op::RampTime;
use sx126x::op::RxTxTimeout;
use sx126x::op::TxParams;
use sx126x::SX126x;
use ws2812_timer_delay::Ws2812;

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

type Dio1Pin = gpio::gpioc::PC4<gpio::Input>;
static mut DIO1_PIN: Option<Dio1Pin> = None;
static DIO1_RISEN: atomic::AtomicBool = atomic::AtomicBool::new(false);

const RF_FREQUENCY: u32 = 868_000_000; // 868MHz (EU)
const F_XTAL: u32 = 32_000_000; // 32MHz

#[interrupt]
fn EXTI4() {
    hprintln!("EXTI4");
    let dio1_pin = unsafe { DIO1_PIN.as_mut().unwrap() };
    if dio1_pin.check_interrupt() {
        DIO1_RISEN.store(true, atomic::Ordering::Relaxed);

        dio1_pin.clear_interrupt_pending_bit();
    }
}

#[interrupt] 
fn EXTI9_5() {
    let mut busy: gpioc::PC5 = unsafe { core::mem::transmute(()) };
    if busy.check_interrupt() {
        busy.clear_interrupt_pending_bit();
    }
}

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
    if let (Some(mut dp), Some(cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();
        let gpioe = dp.GPIOE.split();

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

        const LED_NUM: usize = 4;
        let mut data = [RGB8::default(); LED_NUM];
        let mut neopixel = Ws2812::new(timer, gpioa.pa9.into_push_pull_output());

        loop {
            for j in 0..256 {
                for i in 0..LED_NUM {
                    // rainbow cycle using HSV, where hue goes through all colors in circle
                    // value sets the brightness
                    let hsv = Hsv{hue: ((i * 3 + j) % 256) as u8, sat: 255, val: 100};

                    data[i] = hsv2rgb(hsv);
                }
                // before writing, apply gamma correction for nicer rainbow
                neopixel.write(gamma(data.iter().cloned())).unwrap();
                delay.delay_ms(10u8);
            }
        }

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

        // ===== Init SPI1 =====
        let spi1_sck = gpioa.pa5.into_alternate();
        let spi1_miso = gpioa.pa6.into_floating_input();
        let spi1_mosi = gpioa.pa7.into_alternate();
        let spi1_mode = spi::Mode {
            polarity: spi::Polarity::IdleLow,
            phase: spi::Phase::CaptureOnFirstTransition,
        };
        let spi1_freq = 100.kHz();

        let spi1_pins = (spi1_sck, spi1_miso, spi1_mosi);

        let mut spi1 = &mut spi::Spi::new(dp.SPI1, spi1_pins, spi1_mode, spi1_freq, &clocks);

        let lora_nreset = gpiob
            .pb0
            .into_push_pull_output_in_state(gpio::PinState::High);
        let lora_nss = gpioa
            .pa4
            .into_push_pull_output_in_state(gpio::PinState::High);
        let mut lora_busy = gpioc.pc5.into_floating_input();
        let lora_ant = DummyPin::new_high();

        // This is safe as long as the only other place we use DIO1_PIN is in the ISR
        let lora_dio1 = unsafe {
            DIO1_PIN.replace(gpioc.pc4.into_floating_input());
            DIO1_PIN.as_mut().unwrap()
        };

        let mut syscfg = dp.SYSCFG.constrain();
        lora_dio1.make_interrupt_source(&mut syscfg);
        lora_dio1.trigger_on_edge(&mut dp.EXTI, gpio::Edge::Rising);
        lora_dio1.enable_interrupt(&mut dp.EXTI);

        lora_busy.make_interrupt_source(&mut syscfg);
        lora_busy.trigger_on_edge(&mut dp.EXTI, gpio::Edge::RisingFalling);
        lora_busy.enable_interrupt(&mut dp.EXTI);


        // Wrap DIO1 pin in Dio1PinRefMut newtype, as mutable refences to
        // pins do not implement the `embedded_hal::digital::v2::InputPin` trait.
        let lora_dio1 = Dio1PinRefMut(lora_dio1);

        let lora_pins = (
            lora_nss,    // D7
            lora_nreset, // A0
            lora_busy,   // D4
            lora_ant,    // D8
            lora_dio1,   // D6
        );

        let conf = build_config();

        delay.delay_ms(1000u32);

        let mut lora = SX126x::new(lora_pins);
        lora.init(spi1, &mut delay, conf).unwrap();

        let crc_type = packet::lora::LoRaCrcType::CrcOn;

        unsafe {
            pac::NVIC::unmask(pac::Interrupt::EXTI4);
            pac::NVIC::unpend(pac::Interrupt::EXTI4);

            pac::NVIC::unmask(pac::Interrupt::EXTI9_5);
            pac::NVIC::unpend(pac::Interrupt::EXTI9_5);
        }

        hprintln!("Sending message");
        lora.set_rx(&mut spi1, &mut delay, RxTxTimeout::from_ms(5000))
            .unwrap();
        let mut timer2 = dp.TIM3.counter_ms(&clocks);
        timer2.start(1000u32.millis()).unwrap();

        let f1 = async {
            loop {
                NbFuture::new(|| timer2.wait()).await.unwrap();
                let status = lora.get_status(&mut spi1, &mut delay);
                hprintln!("Status: {:?}", status); 
                lora.set_buffer_base_address(&mut spi1, &mut delay, 0, 0);


                //get_serial().log(write_to::show(&mut buf, format_args!("Status: {:?}\n", status)).unwrap()).await;

                if DIO1_RISEN.swap(false, atomic::Ordering::Relaxed) {
                    hprintln!("DI01_RISEN");
                    loop {}
                }
            }
        };
        let f2 = async {
            loop {
                get_serial().read(&mut [0; 32]).await.unwrap();
            }
        };

        join!(f1, f2);
    }
}

fn build_config() -> sx126x::conf::Config {
    use sx126x::op::{
        irq::IrqMaskBit::*, modulation::lora::*, packet::lora::LoRaPacketParams,
        rxtx::DeviceSel::SX1262, PacketType::LoRa,
    };

    let mod_params = LoraModParams::default().into();
    let tx_params = TxParams::default()
        .set_power_dbm(14)
        .set_ramp_time(RampTime::Ramp200u);
    let pa_config = PaConfig::default()
        .set_device_sel(SX1262)
        .set_pa_duty_cycle(0x04);

    let dio1_irq_mask = IrqMask::all();

    let packet_params = LoRaPacketParams {
        payload_len: 0xFF,
        ..Default::default()
    }
    .into();

    let rf_freq = sx126x::calc_rf_freq(RF_FREQUENCY as f32, F_XTAL as f32);

    sx126x::conf::Config {
        packet_type: LoRa,
        sync_word: 0x1424, // Private networks
        calib_param: CalibParam::from(0x7F),
        mod_params,
        tx_params,
        pa_config,
        packet_params: Some(packet_params),
        dio1_irq_mask,
        dio2_irq_mask: IrqMask::none(),
        dio3_irq_mask: IrqMask::none(),
        rf_frequency: RF_FREQUENCY,
        rf_freq,
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

struct Dio1PinRefMut<'dio1>(&'dio1 mut Dio1Pin);

impl<'dio1> embedded_hal::digital::v2::InputPin for Dio1PinRefMut<'dio1> {
    type Error = core::convert::Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.0.is_high())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.0.is_low())
    }
}
