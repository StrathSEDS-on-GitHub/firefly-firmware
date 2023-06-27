#![feature(async_closure)]
#![feature(type_alias_impl_trait)]
//! Demonstrate the use of a blocking `Delay` using the SYST (sysclock) timer.

#![allow(clippy::empty_loop)]
#![allow(dead_code)]
#![no_main]
#![no_std]

use crate::bmp581::BMP581;
use crate::hal::timer::TimerExt;
use crate::ina219::INA219;
use crate::mission::PYRO_ENABLE_PIN;
use crate::mission::PYRO_FIRE1;
use crate::mission::PYRO_FIRE2;
use crate::mission::Role;
use crate::mission::PYRO_ADC;
use crate::mission::PYRO_MEASURE_PIN;
use crate::radio::Radio;
use crate::sdio::setup_logger;
use cassette::pin_mut;
use cassette::Cassette;
use hal::pac::TIM13;
use core::fmt::Write;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::exception;
use cortex_m_rt::ExceptionFrame;
use cortex_m_semihosting::hio;
use dummy_pin::DummyPin;
use f4_w25q::w25q::W25Q;
use hal::adc::config::AdcConfig;
use hal::adc::Adc;
use hal::dma::StreamsTuple;
use hal::gpio;
use hal::gpio::alt::quadspi::Bank1;
use hal::gpio::Output;
use hal::gpio::Pin;
use hal::gpio::PushPull;
use hal::i2c::I2c;
use hal::pac::TIM2;
use hal::pac::TIM9;
use hal::qspi::FlashSize;
use hal::qspi::Qspi;
use hal::qspi::QspiConfig;
use hal::rtc::Rtc;
use hal::spi;
use hal::timer::CounterHz;
use hal::timer::CounterMs;
use littlefs2::fs::Allocation;
use littlefs2::fs::Filesystem;
use littlefs2::path;
use littlefs2::path::Path;
use logger::setup_usb_serial;
use smart_leds::SmartLedsWrite;
use sx126x::op::CalibParam;
use sx126x::op::IrqMask;
use sx126x::op::IrqMaskBit;
use sx126x::op::PaConfig;
use sx126x::op::RampTime;
use sx126x::op::TcxoVoltage;
use sx126x::op::TxParams;
use sx126x::SX126x;
use ws2812_timer_delay::Ws2812;

use core::cell::RefCell;
use core::panic::PanicInfo;

use crate::hal::{pac, prelude::*};
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;

mod bmp581;
mod futures;
mod gps;
mod ina219;
mod logger;
mod mission;
mod radio;
mod sdio;
mod stepper;
mod usb_msc;

static mut EP_MEMORY: [u32; 1024] = [0; 1024];
static CLOCKS: Mutex<RefCell<Option<hal::rcc::Clocks>>> = Mutex::new(RefCell::new(None));

type Dio1Pin = gpio::gpioc::PC4<gpio::Input>;
static mut DIO1_PIN: Option<Dio1Pin> = None;

const RF_FREQUENCY: u32 = 868_000_000; // 868MHz (EU)
const F_XTAL: u32 = 32_000_000; // 32MHz

static NEOPIXEL: Mutex<RefCell<Option<Ws2812<CounterHz<TIM2>, Pin<'A', 9, Output>>>>> =
    Mutex::new(RefCell::new(None));

static mut PANIC_TIMER: Option<CounterHz<TIM9>> = None;
static mut BUZZER: Option<Pin<'E', 1, Output<PushPull>>> = None;
static mut BUZZER_TIMER: Option<CounterMs<TIM13>> = None;
static RTC: Mutex<RefCell<Option<Rtc>>> = Mutex::new(RefCell::new(None));
static mut FS_ALLOC: Option<Allocation<W25Q<Bank1>>> = None;
static mut FLASH: Option<W25Q<Bank1>> = None;

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

        dp.RCC
            .ahb3enr
            .modify(|r, w| unsafe { w.bits(r.bits()).qspien().enabled() });
        let rcc = dp.RCC.constrain();

        let clocks = rcc
            .cfgr
            .sysclk(48.MHz())
            .use_hse(16.MHz())
            .pclk1(48.MHz())
            .pclk2(48.MHz())
            .require_pll48clk()
            .freeze();

        // SAFETY: these are touched only in panic timer/buzzer code
        unsafe {
            PANIC_TIMER.replace(dp.TIM9.counter_hz(&clocks));
            BUZZER.replace(gpioe.pe1.into_push_pull_output());
            BUZZER_TIMER.replace(dp.TIM13.counter_ms(&clocks));
        }

        let mut delay = cp.SYST.delay(&clocks);
        let mut timer = dp.TIM2.counter_hz(&clocks);
        timer.start(7.MHz()).unwrap();

        const LED_NUM: usize = 4;
        let mut pa9 = gpioa.pa9.into_push_pull_output();
        pa9.set_speed(gpio::Speed::VeryHigh);
        let mut neopixel = Ws2812::new(timer, pa9);
        neopixel.write([[0, 0, 5]; 4].into_iter()).unwrap();

        let gpiod = dp.GPIOD.split();
        let rtc = hal::rtc::Rtc::new(dp.RTC, &mut dp.PWR);

        cortex_m::interrupt::free(|cs| {
            CLOCKS.borrow(cs).borrow_mut().replace(clocks);
            NEOPIXEL.borrow(cs).borrow_mut().replace(neopixel);
            radio::TIMER
                .borrow(cs)
                .borrow_mut()
                .replace(dp.TIM5.counter_us(&clocks));
            RTC.borrow(cs).borrow_mut().replace(rtc);
            unsafe {
                PYRO_MEASURE_PIN.replace(gpioc.pc0.into_analog());
                PYRO_ENABLE_PIN.replace(gpiod.pd7.into_push_pull_output_in_state(gpio::PinState::Low));
                PYRO_FIRE2.replace(gpiod.pd6.into_push_pull_output());
                PYRO_FIRE1.replace(gpiod.pd5.into_push_pull_output());
                PYRO_ADC.replace(Adc::adc1(dp.ADC1, false, AdcConfig::default()));
            }
        });

        #[cfg(not(feature = "msc"))]
        {
            setup_usb_serial(
                dp.OTG_FS_GLOBAL,
                dp.OTG_FS_DEVICE,
                dp.OTG_FS_PWRCLK,
                gpioa.pa11,
                gpioa.pa12,
                &clocks,
            );
        }
        let stepper_serial = dp
            .USART2
            .serial(
                (gpioa.pa2.into_alternate(), gpioa.pa3.into_alternate()),
                hal::serial::config::Config {
                    baudrate: 9600.bps(),
                    dma: hal::serial::config::DmaConfig::TxRx,
                    ..Default::default()
                },
                &clocks,
            )
            .unwrap();
        stepper::setup(dp.DMA1, stepper_serial);
        let mut gconf = tmc2209::reg::GCONF::default();
        let mut vactual = tmc2209::reg::VACTUAL::ENABLED_STOPPED;
        vactual.set(10);
        gconf.set_pdn_disable(true);
        gconf.set_internal_rsense(true);

        // let STEPPER_ADDR = 1;

        // loop {
        //     let req = tmc2209::write_request(STEPPER_ADDR, gconf);
        //     stepper::tx(req.bytes()).await;
        //     delay.delay_ms(500u32);
        //     let req = tmc2209::write_request(STEPPER_ADDR, vactual);
        //     stepper::tx(req.bytes()).await;
        //     cortex_m::interrupt::free(|cs| {
        //         NEOPIXEL
        //             .borrow(cs)
        //             .borrow_mut()
        //             .as_mut()
        //             .unwrap()
        //             .write([[0, 20, 0]; 4].into_iter())
        //             .unwrap();
        //     });
        //     YieldFuture::new().await;
        //     delay.delay_ms(1000u32);
        // }

        gpioe.pe3.into_floating_input();
        gpioe.pe4.into_floating_input();

        let i2c1 = I2c::new(dp.I2C1, (gpiob.pb8, gpiob.pb9), 1000u32.kHz(), &clocks);
        let streams = StreamsTuple::new(unsafe { core::mem::transmute::<_, pac::DMA1>(()) });
        let tx_stream = streams.1;
        let rx_stream = streams.0;

        unsafe {
            pac::NVIC::unmask(hal::interrupt::DMA1_STREAM1);
            pac::NVIC::unmask(hal::interrupt::DMA1_STREAM0);
        }

        let qspi = Qspi::<Bank1>::new(
            dp.QUADSPI,
            (
                gpiob.pb6, gpiod.pd11, gpiod.pd12, gpioe.pe2, gpioa.pa1, gpiob.pb1,
            ),
            QspiConfig::default()
                .address_size(hal::qspi::AddressSize::Addr24Bit)
                .flash_size(FlashSize::from_megabytes(16))
                .clock_prescaler(0)
                .sample_shift(hal::qspi::SampleShift::HalfACycle),
        );

        let flash = W25Q::new(qspi).unwrap();

        #[cfg(feature = "msc")]
        {
            setup_usb_msc(
                dp.OTG_FS_GLOBAL,
                dp.OTG_FS_DEVICE,
                dp.OTG_FS_PWRCLK,
                gpioa.pa11,
                gpioa.pa12,
                &clocks,
                flash,
            );
            loop {}
        }

        let alloc = Filesystem::allocate();
        let (flash, alloc) = unsafe {
            FS_ALLOC.replace(alloc);
            FLASH.replace(flash);
            (FLASH.as_mut().unwrap(), FS_ALLOC.as_mut().unwrap())
        };
        let fs = Filesystem::mount(alloc, flash).unwrap();
        // let mut step = gpioc.pc3.into_push_pull_output();
        // let mut dir = gpioc.pc2.into_push_pull_output();
        // let mut tim7 = dp.TIM7.counter_hz(&clocks);

        // dir.set_low();

        // loop {
        //     step.toggle();
        //     tim7.start(100.Hz());
        //     nb::block!(tim7.wait()).unwrap();
        // }

        let gps_serial = dp
            .USART1
            .serial(
                (gpioa.pa15.into_alternate(), gpioa.pa10.into_alternate()),
                hal::serial::config::Config {
                    baudrate: 9600.bps(),
                    dma: hal::serial::config::DmaConfig::TxRx,
                    ..Default::default()
                },
                &clocks,
            )
            .unwrap();

        gps::setup(dp.DMA2, gps_serial);
        gps::tx(b"$PMTK251,115200*1F\r\n").await;
        gps::change_baudrate(115200).await;
        gps::set_nmea_output().await;
        gps::tx(b"$PMTK220,100*2F\r\n").await;

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

        let mut spi1 = spi::Spi::new(dp.SPI1, spi1_pins, spi1_mode, spi1_freq, &clocks);

        let lora_nreset = gpiob
            .pb0
            .into_push_pull_output_in_state(gpio::PinState::High);
        let lora_nss = gpioa
            .pa4
            .into_push_pull_output_in_state(gpio::PinState::High);
        let lora_busy = gpioc.pc5.into_floating_input();
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

        let conf = build_config().await;

        delay.delay_ms(1000u32);

        let mut lora = SX126x::new(lora_pins);
        lora.init(&mut spi1, &mut delay, conf).unwrap();

        unsafe {
            pac::NVIC::unmask(pac::Interrupt::EXTI4);
            pac::NVIC::unpend(pac::Interrupt::EXTI4);
        }

        Radio::init(lora, spi1, delay);

        let board_id = fs.read::<1>(path!("/board_id")).unwrap()[0] - b'0';
        let role = match board_id {
            3 => Role::Avionics,
            4 => Role::Cansat,
            _ => Role::Ground,
        };

        unsafe { mission::ROLE = role };

        if role != Role::Ground {
            setup_logger(fs).unwrap();
        }


        let mut i2c2 = I2c::new(
            dp.I2C3,
            (gpioa.pa8.into_input(), gpiob.pb4.into_input()),
            100u32.kHz(),
            &clocks,
        );

        let mut ina = INA219::new(&mut i2c2).unwrap();
        ina.calibrate(87).unwrap();
        let bmp = if mission::role() != Role::Ground {
            let mut bmp = BMP581::new(i2c1.use_dma(tx_stream, rx_stream)).unwrap();
            bmp.enable_pressure_temperature().unwrap();
            bmp.setup_fifo().unwrap();
            Some(bmp)
        } else { None };

        mission::begin(bmp, dp.TIM12.counter_ms(&clocks)).await;
    }
}

async fn build_config() -> sx126x::conf::Config {
    use sx126x::op::{
        modulation::lora::*, packet::lora::LoRaPacketParams, rxtx::DeviceSel::SX1262,
        PacketType::LoRa,
    };

    let mod_params = LoraModParams::default()
        .set_spread_factor(LoRaSpreadFactor::SF7)
        .set_bandwidth(LoRaBandWidth::BW250)
        .set_coding_rate(LoraCodingRate::CR4_8)
        .into();

    let tx_params = TxParams::default()
        .set_power_dbm(22)
        .set_ramp_time(RampTime::Ramp200u);
    let pa_config = PaConfig::default()
        .set_device_sel(SX1262)
        .set_pa_duty_cycle(0x04)
        .set_hp_max(0x07);

    let dio1_irq_mask = IrqMask::none()
        .combine(IrqMaskBit::RxDone)
        .combine(IrqMaskBit::TxDone)
        .combine(IrqMaskBit::Timeout);

    let packet_params = LoRaPacketParams {
        payload_len: 255,
        preamble_len: 0x12,
        crc_type: sx126x::op::packet::lora::LoRaCrcType::CrcOn,
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
        tcxo_opts: Some((TcxoVoltage::Volt3_3, [0, 0, 0x64].into())),
    }
}

#[panic_handler]
pub fn panic(info: &PanicInfo) -> ! {
    if let Ok(mut stdout) = hio::hstdout() {
        writeln!(stdout, "A panic occured:\n {}", info).ok();
    }
    let mut timer = unsafe { PANIC_TIMER.take() }.unwrap();
    timer.start(4.Hz()).ok();
    cortex_m::interrupt::free(|cs| {
        let mut neo_ref = NEOPIXEL.borrow(cs).borrow_mut();
        let neo = neo_ref.as_mut().unwrap();
        loop {
            neo.write([[255, 0, 0]; 4].into_iter()).unwrap();
            nb::block!(timer.wait()).ok();
            neo.write([[0, 0, 0]; 4].into_iter()).unwrap();
            nb::block!(timer.wait()).ok();
        }
    });
    unreachable!();
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

#[exception]
unsafe fn HardFault(frame: &ExceptionFrame) -> ! {
    panic!("HardFault {:?}", frame);
}
