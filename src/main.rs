#![allow(clippy::empty_loop)]
#![allow(dead_code)]
#![no_main]
#![no_std]

use crate::mission::Role;
use crate::mission::PYRO_ADC;
use crate::mission::PYRO_ENABLE_PIN;
use crate::mission::PYRO_FIRE1;
use crate::mission::PYRO_FIRE2;
use crate::mission::PYRO_MEASURE_PIN;
use crate::pins::*;
use crate::radio::Radio;
use crate::sdio::setup_logger;
use core::convert::Infallible;
use core::fmt::Write;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::exception;
use cortex_m_rt::ExceptionFrame;
use cortex_m_semihosting::hio;
use dummy_pin::DummyPin;
use embassy_executor::Spawner;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::ErrorType;
use f4_w25q::embedded_storage::W25QSequentialStorage;
use f4_w25q::w25q::W25Q;
use hal::adc::config::AdcConfig;
use hal::adc::Adc;
use hal::dma::StreamsTuple;
use hal::gpio;
use hal::gpio::NoPin;
use hal::pac::TIM13;
use hal::pac::TIM14;
use hal::pac::TIM3;
use hal::pac::TIM9;
use hal::qspi::Bank1;
use hal::qspi::FlashSize;
use hal::qspi::Qspi;
use hal::qspi::QspiConfig;
use hal::rtc;
use hal::rtc::Rtc;
use hal::spi;
use hal::spi::Spi;
use hal::timer::Counter;
use hal::timer::CounterHz;
use hal::timer::PwmHz;
use sequential_storage::cache::NoCache;
use sequential_storage::map;
use stm32f4xx_hal::pac::SPI1;
use storage_types::U64Item;
use sx126x::op::CalibParam;
use sx126x::op::IrqMask;
use sx126x::op::IrqMaskBit;
use sx126x::op::PaConfig;
use sx126x::op::RampTime;
use sx126x::op::TcxoVoltage;
use sx126x::op::TxParams;
use sx126x::SX126x;
use usb_logger::setup_usb_serial;
use ws2812_spi::Ws2812;
use ws2812_spi::MODE;

use core::cell::RefCell;
use core::panic::PanicInfo;

use crate::hal::{pac, prelude::*};
use stm32f4xx_hal as hal;

mod altimeter;
mod bmp581;
mod futures;
mod gps;
mod mission;
mod neopixel;
mod pins;
mod radio;
mod sdio;
mod stepper;
mod usb_logger;
mod usb_msc;

static mut EP_MEMORY: [u32; 1024] = [0; 1024];
static CLOCKS: Mutex<RefCell<Option<hal::rcc::Clocks>>> = Mutex::new(RefCell::new(None));

type Dio1Pin = gpio::gpioc::PC4<gpio::Input>;
static mut DIO1_PIN: Option<Dio1Pin> = None;

const RF_FREQUENCY: u32 = 868_000_000; // 868MHz (EU)
const F_XTAL: u32 = 32_000_000; // 32MHz

static mut PANIC_TIMER: Option<CounterHz<TIM9>> = None;
static mut BUZZER: Option<BuzzerPin> = None;
static mut BUZZER_TIMER: Option<Counter<TIM13, 10000>> = None;
static mut PYRO_TIMER: Option<Counter<TIM14, 10000>> = None;
static RTC: Mutex<RefCell<Option<Rtc>>> = Mutex::new(RefCell::new(None));

pub const CAPACITY: usize = 16777216;
static mut FLASH: Option<W25QSequentialStorage<Bank1, CAPACITY>> = None;

const CONFIG_FLASH_RANGE: core::ops::Range<u32> = 0..8192;
const LOGS_FLASH_RANGE: core::ops::Range<u32> = 8192..CAPACITY as u32;

fn play_tone<P: hal::timer::Pins<TIM3>, DELAY: DelayNs>(
    channel: &mut PwmHz<TIM3, P>,
    delay: &mut DELAY,
    freq: fugit::HertzU32,
    duration: fugit::MillisDurationU32,
) {
    let max_duty = channel.get_max_duty();
    channel.set_duty(hal::timer::Channel::C2, max_duty / 2);
    channel.enable(hal::timer::Channel::C2);
    channel.set_period(freq);
    delay.delay_ms(duration.to_millis());
    channel.disable(hal::timer::Channel::C2);
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    if let (Some(mut dp), Some(cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();
        let gpiod = dp.GPIOD.split();
        let gpioe = dp.GPIOE.split();

        let gpio = GpioBuses {
            a: gpioa,
            b: gpiob,
            c: gpioc,
            d: gpiod,
            e: gpioe,
        };

        dp.RCC
            .ahb3enr
            .modify(|r, w| unsafe { w.bits(r.bits()).qspien().enabled() });
        let rcc = dp.RCC.constrain();

        let clocks = rcc
            .cfgr
            .sysclk(96.MHz())
            .use_hse(16.MHz())
            .pclk1(48.MHz())
            .pclk2(96.MHz())
            .require_pll48clk()
            .freeze();

        // SAFETY: these are touched only in panic timer/buzzer code
        unsafe {
            PANIC_TIMER.replace(dp.TIM9.counter_hz(&clocks));
            BUZZER.replace(buzzer_pin!(gpio));
            BUZZER_TIMER.replace(dp.TIM13.counter(&clocks));
            PYRO_TIMER.replace(dp.TIM14.counter(&clocks));
        }

        let mut delay = cp.SYST.delay(&clocks);

        const LED_NUM: usize = 3;
        let neopixel_spi = Spi::new(
            neopixel_spi!(dp),
            (NoPin::new(), NoPin::new(), {
                let mut pin = neopixel_pin!(gpio).into_alternate();
                pin.set_speed(gpio::Speed::VeryHigh);
                pin
            }),
            MODE,
            3.MHz(),
            &clocks,
        );

        neopixel::new_neopixel(Ws2812::new(neopixel_spi));
        neopixel::update_pixel(0, [0, 255, 0]);

        let mut rtc = hal::rtc::Rtc::new(dp.RTC, &mut dp.PWR);
        rtc.enable_wakeup(1000u32.millis().into());

        rtc.listen(&mut dp.EXTI, rtc::Event::Wakeup);

        cortex_m::interrupt::free(|cs| {
            CLOCKS.borrow(cs).borrow_mut().replace(clocks);
            RTC.borrow(cs).borrow_mut().replace(rtc);
            unsafe {
                PYRO_MEASURE_PIN.replace(gpio.c.pc0.into_analog());
                PYRO_ENABLE_PIN.replace(
                    gpio.d
                        .pd7
                        .into_push_pull_output_in_state(gpio::PinState::Low),
                );
                PYRO_FIRE2.replace(gpio.d.pd6.into_push_pull_output());
                PYRO_FIRE1.replace(gpio.d.pd5.into_push_pull_output());
                PYRO_ADC.replace(Adc::adc1(dp.ADC1, false, AdcConfig::default()));
            }
        });

        #[cfg(not(feature = "msc"))]
        {
            setup_usb_serial(
                dp.OTG_FS_GLOBAL,
                dp.OTG_FS_DEVICE,
                dp.OTG_FS_PWRCLK,
                gpio.a.pa11,
                gpio.a.pa12,
                &clocks,
            );
        }

        gpio.e.pe3.into_floating_input();
        gpio.e.pe4.into_floating_input();

        unsafe {
            pac::NVIC::unmask(hal::interrupt::DMA1_STREAM1);
            pac::NVIC::unmask(hal::interrupt::DMA1_STREAM0);
        }

        let qspi = Qspi::<Bank1>::new(
            dp.QUADSPI,
            qspi_pins!(gpio),
            QspiConfig::default()
                .address_size(hal::qspi::AddressSize::Addr24Bit)
                .flash_size(FlashSize::from_megabytes(16))
                .clock_prescaler(0)
                .sample_shift(hal::qspi::SampleShift::HalfACycle),
        );

        let flash = W25Q::new(qspi).unwrap();

        #[cfg(feature = "msc")]
        {
            crate::usb_msc::setup_usb_msc(
                dp.OTG_FS_GLOBAL,
                dp.OTG_FS_DEVICE,
                dp.OTG_FS_PWRCLK,
                gpio.a.pa11,
                gpio.a.pa12,
                &clocks,
                flash,
            );
            loop {}
        }
        let gps_serial = dp
            .USART1
            .serial(
                gps_pins!(gpio),
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

        let mut wrapper = W25QSequentialStorage::new(flash);

        let board_id: Result<Option<U64Item>, _> = map::fetch_item(
            &mut wrapper,
            CONFIG_FLASH_RANGE,
            NoCache::new(),
            &mut [0; 41],
            "id".try_into().unwrap(),
        )
        .await;

        let role = match board_id.unwrap().unwrap().1 {
            5 => Role::Ground,
            3 | 4 => Role::Avionics,
            _ => Role::Cansat,
        };

        unsafe { mission::ROLE = role };

        let bmp = if mission::role() != Role::Ground {
            let streams = StreamsTuple::new(dp.DMA1);
            let tx_stream = streams.1;
            let rx_stream = streams.0;
            let i2c = dp
                .I2C1
                .i2c((gpio.b.pb8, gpio.b.pb7), 400.kHz(), &clocks)
                .use_dma(tx_stream, rx_stream);

            #[cfg(feature = "target-mini")]
            {
                Some({
                    static I2C_BUS: Option<critical_section::Mutex<RefCell<I2c1Handle>>> = None;
                    critical_section::with(|cs| {
                        I2C_BUS.as_ref().unwrap().borrow(cs).replace(i2c);
                    });
                    let alt_proxy = crate::pins::i2c::CriticalSectionDevice::new(I2C_BUS.as_ref().unwrap());

                    altimeter::BMP388Wrapper::new(alt_proxy, &mut delay)
                    // let mut icm = IcmImu::new(i2c, 0x68).unwrap();
                    // icm.enable_acc().unwrap();
                    // icm.enable_gyro().unwrap();
                    // todo!()
                })
            }

            #[cfg(feature = "target-maxi")]
            {
                let mut bmp = crate::bmp581::BMP581::new(i2c).unwrap();
                bmp.enable_pressure_temperature().unwrap();
                bmp.setup_fifo().unwrap();
                Some(bmp)
            }
        } else {
            None
        };

        // ===== Init SPI1 =====
        let spi1_mode = spi::Mode {
            polarity: spi::Polarity::IdleLow,
            phase: spi::Phase::CaptureOnFirstTransition,
        };
        let spi1_freq = 100.kHz();

        let (nss, sck, miso, mosi, nreset, busy, dio) = radio_pins!(gpio);

        let spi1_pins = (sck, miso, mosi);

        let spi1 = spi::Spi::new(dp.SPI1, spi1_pins, spi1_mode, spi1_freq, &clocks);

        let lora_nreset = nreset.into_push_pull_output_in_state(gpio::PinState::High);
        let lora_nss = nss.into_push_pull_output_in_state(gpio::PinState::High);
        let lora_busy = busy.into_floating_input();
        let lora_ant = DummyPin::new_high();

        static SPI_BUS: Option<critical_section::Mutex<RefCell<Spi<SPI1>>>> = None;
        critical_section::with(|cs| {
            SPI_BUS.as_ref().unwrap().borrow(cs).replace(spi1);
        });

        let mut spi1_device = embedded_hal_bus::spi::CriticalSectionDevice::new(
            &SPI_BUS.as_ref().unwrap(),
            lora_nss,
            delay,
        )
        .unwrap();

        // This is safe as long as the only other place we use DIO1_PIN is in the ISR
        let lora_dio1 = unsafe {
            DIO1_PIN.replace(dio.into_floating_input());
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
            lora_nreset, // A0
            lora_busy,   // D4
            lora_ant,    // D8
            lora_dio1,   // D6
        );

        let conf = build_config(&mut wrapper).await;

        if role != Role::Ground {
            setup_logger(wrapper).unwrap();
        }

        let mut lora = SX126x::new(lora_pins);
        lora.init(&mut spi1_device, conf).unwrap();

        Radio::init(lora, spi1_device);
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::RTC_WKUP);
            pac::NVIC::unmask(pac::Interrupt::EXTI4);
            pac::NVIC::unpend(pac::Interrupt::RTC_WKUP);
            pac::NVIC::unpend(pac::Interrupt::EXTI4);
        }

        mission::begin(bmp, dp.TIM12.counter(&clocks)).await;
    }
}

async fn build_config(flash: &mut W25QSequentialStorage<Bank1, CAPACITY>) -> sx126x::conf::Config {
    use sx126x::op::{
        modulation::lora::*, packet::lora::LoRaPacketParams, rxtx::DeviceSel::SX1262,
        PacketType::LoRa,
    };

    let sf: U64Item = map::fetch_item(
        flash,
        CONFIG_FLASH_RANGE,
        NoCache::new(),
        &mut [0; 128],
        "sf".try_into().unwrap(),
    )
    .await
    .unwrap()
    .unwrap_or(U64Item("".try_into().unwrap(), 7));
    let sf = match sf.1 {
        7 => LoRaSpreadFactor::SF7,
        8 => LoRaSpreadFactor::SF8,
        9 => LoRaSpreadFactor::SF9,
        10 => LoRaSpreadFactor::SF10,
        11 => LoRaSpreadFactor::SF11,
        12 => LoRaSpreadFactor::SF12,
        _ => panic!("Invalid spread factor"),
    };

    let bw: U64Item = map::fetch_item(
        flash,
        CONFIG_FLASH_RANGE,
        NoCache::new(),
        &mut [0; 128],
        "bw".try_into().unwrap(),
    )
    .await
    .unwrap()
    .unwrap_or(U64Item("".try_into().unwrap(), 250));

    let bw = match bw.1 {
        7 => LoRaBandWidth::BW7,
        10 => LoRaBandWidth::BW10,
        15 => LoRaBandWidth::BW15,
        20 => LoRaBandWidth::BW20,
        31 => LoRaBandWidth::BW31,
        41 => LoRaBandWidth::BW41,
        62 => LoRaBandWidth::BW62,
        125 => LoRaBandWidth::BW125,
        250 => LoRaBandWidth::BW250,
        500 => LoRaBandWidth::BW500,
        _ => panic!("Invalid bandwidth"),
    };

    let cr: U64Item = map::fetch_item(
        flash,
        CONFIG_FLASH_RANGE,
        NoCache::new(),
        &mut [0; 128],
        "cr".try_into().unwrap(),
    )
    .await
    .unwrap()
    .unwrap_or(U64Item("".try_into().unwrap(), 8));

    let cr = match cr.1 {
        5 => LoraCodingRate::CR4_5,
        6 => LoraCodingRate::CR4_6,
        7 => LoraCodingRate::CR4_7,
        8 => LoraCodingRate::CR4_8,
        _ => panic!("Invalid coding rate"),
    };

    let power: U64Item = map::fetch_item(
        flash,
        CONFIG_FLASH_RANGE,
        NoCache::new(),
        &mut [0; 128],
        "power".try_into().unwrap(),
    )
    .await
    .unwrap()
    .unwrap_or(U64Item("".try_into().unwrap(), 22));
    let power = power.1;
    if power > 22 {
        panic!("Invalid power");
    }

    let rf_freq: U64Item = map::fetch_item(
        flash,
        CONFIG_FLASH_RANGE,
        NoCache::new(),
        &mut [0; 128],
        "rf_freq".try_into().unwrap(),
    )
    .await
    .unwrap()
    .unwrap_or(U64Item("".try_into().unwrap(), 868_000_000));

    let rf_freq = rf_freq.1;

    let mod_params = LoraModParams::default()
        .set_spread_factor(sf)
        .set_bandwidth(bw)
        .set_coding_rate(cr)
        .into();

    let tx_params = TxParams::default()
        .set_power_dbm(power as i8)
        .set_ramp_time(RampTime::Ramp200u);

    let pa_config = PaConfig::default()
        .set_device_sel(SX1262)
        .set_pa_duty_cycle(0x04)
        .set_hp_max(0x07);

    let dio1_irq_mask = IrqMask::none()
        .combine(IrqMaskBit::RxDone)
        .combine(IrqMaskBit::TxDone)
        .combine(IrqMaskBit::Timeout);

    let packet_params = LoRaPacketParams::default()
        .set_payload_len(255)
        .set_crc_type(sx126x::op::packet::lora::LoRaCrcType::CrcOn)
        .set_preamble_len(0x12)
        .into();

    let rf_freq = sx126x::calc_rf_freq(rf_freq as f32, F_XTAL as f32);

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

    loop {
        neopixel::update_pixel(1, [255, 0, 0]);
        nb::block!(timer.wait()).ok();
        neopixel::update_pixel(1, [0, 0, 0]);
        nb::block!(timer.wait()).ok();
    }
}

struct Dio1PinRefMut<'dio1>(&'dio1 mut Dio1Pin);
impl ErrorType for Dio1PinRefMut<'_> {
    type Error = Infallible;
}

impl<'dio1> embedded_hal::digital::InputPin for Dio1PinRefMut<'dio1> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        self.0.is_high()
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        self.0.is_low()
    }
}

#[exception]
unsafe fn HardFault(frame: &ExceptionFrame) -> ! {
    panic!("HardFault {:?}", frame);
}
