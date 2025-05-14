#![allow(clippy::empty_loop)]
#![allow(dead_code)]
#![allow(static_mut_refs)]
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
use bmi323::AccelConfig;
use bmi323::AccelerometerRange;
use bmi323::Bmi323;
use bmi323::GyroConfig;
use bmi323::GyroscopeRange;
use bmi323::OutputDataRate;
use bno080::interface::I2cInterface;
use bno080::wrapper::BNO080;
use core::cell::UnsafeCell;
use core::convert::Infallible;
use core::fmt::Write;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::exception;
use cortex_m_rt::ExceptionFrame;
use cortex_m_semihosting::hio;
use dummy_pin::DummyPin;
use embassy_executor::Spawner;
use embedded_hal::digital::ErrorType;
use embedded_hal_bus::spi::AtomicDevice;
use embedded_hal_bus::util::AtomicCell;
use f4_w25q::embedded_storage::W25QSequentialStorage;
use f4_w25q::w25q::W25Q;
use fugit::ExtU64;
use futures::TimerDelay;
use hal::adc::config::AdcConfig;
use hal::adc::Adc;
use hal::dma::StreamsTuple;
use hal::gpio;
use hal::gpio::NoPin;
use hal::pac::TIM13;
use hal::pac::TIM14;
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
use icm20948_driver::icm20948::i2c::IcmImu;
use ms5607::MS5607;
use sequential_storage::cache::NoCache;
use sequential_storage::map;
use stm32f4xx_hal::i2c::I2c;
use stm32f4xx_hal::pac::SPI1;
use stm32f4xx_hal::pac::SPI2;
use stm32f4xx_hal::pac::TIM6;
use storage_types::ConfigKey;
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
mod ms5607;
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

struct WriteAdapter<T: _embedded_hal_serial_nb_Write>(T);
impl<T: _embedded_hal_serial_nb_Write> usbd_serial::embedded_io::ErrorType for WriteAdapter<T> {
    type Error = Infallible;
}
impl<T: stm32f4xx_hal::prelude::_embedded_hal_serial_nb_Write> usbd_serial::embedded_io::Write
    for WriteAdapter<T>
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        for w in buf {
            nb::block!(self.0.write(*w)).unwrap();
        }

        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
struct ReadAdapter<T: _embedded_hal_serial_nb_Read>(T);
impl<T: _embedded_hal_serial_nb_Read> usbd_serial::embedded_io::ErrorType for ReadAdapter<T> {
    type Error = Infallible;
}
impl<T: stm32f4xx_hal::prelude::_embedded_hal_serial_nb_Read> usbd_serial::embedded_io::Read
    for ReadAdapter<T>
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut i = 0;
        while i < buf.len() {
            if let Ok(b) = self.0.read() {
                buf[i] = b;
                i += 1;
            } else {
                break;
            }
        }
        Ok(i)
    }
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

        let mut delay = cp.SYST.delay(&clocks);
        let buzzer_pin = buzzer_pin!(gpio);

        // SAFETY: these are touched only in panic timer/buzzer code
        unsafe {
            PANIC_TIMER.replace(dp.TIM9.counter_hz(&clocks));
            BUZZER.replace(buzzer_pin);
            BUZZER_TIMER.replace(dp.TIM13.counter(&clocks));
            PYRO_TIMER.replace(dp.TIM14.counter(&clocks));
        }

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
        neopixel::update_pixel(0, [255, 0, 0]);

        let mut rtc = hal::rtc::Rtc::new(dp.RTC, &mut dp.PWR);
        rtc.listen(&mut dp.EXTI, rtc::Event::Wakeup);
        cortex_m::interrupt::free(|cs| {
            CLOCKS.borrow(cs).borrow_mut().replace(clocks);
            RTC.borrow(cs).borrow_mut().replace(rtc);
            unsafe {
                let (enable, p2, p1) = pyro_pins!(gpio);
                PYRO_MEASURE_PIN.replace(gpio.c.pc0.into_analog());
                PYRO_ENABLE_PIN.replace(enable);
                PYRO_FIRE2.replace(p2);
                PYRO_FIRE1.replace(p1);
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
        delay.delay_ms(1000);

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
            .USART2
            .serial(
                gps_pins!(gpio),
                hal::serial::config::Config {
                    baudrate: 115200.bps(),
                    dma: hal::serial::config::DmaConfig::TxRx,
                    ..Default::default()
                },
                &clocks,
            )
            .unwrap();
        let streams = StreamsTuple::new(dp.DMA1);
        let tx_stream = streams.6;
        let rx_stream = streams.5;
        gps::setup(tx_stream, rx_stream, gps_serial);
        gps::init_teseo().await;
        gps::set_par(gps::ConfigBlock::ConfigCurrent, 201, &2u32.to_be_bytes(), None).await;
        
        // gps::tx(b"$PMTK251,115200*1F\r\n").await;
        // gps::change_baudrate(115200).await;
        // gps::set_nmea_output().await;
        // gps::tx(b"$PMTK220,100*2F\r\n").await;

        let mut wrapper = W25QSequentialStorage::new(flash);
        let total = sequential_storage::queue::space_left(
            &mut wrapper,
            LOGS_FLASH_RANGE,
            &mut NoCache::new(),
        )
        .await
        .unwrap();

        if total < 10240 {
            panic!("Not enough space for logs");
        }

        let board_id: Result<Option<u64>, _> = map::fetch_item(
            &mut wrapper,
            CONFIG_FLASH_RANGE,
            &mut NoCache::new(),
            &mut [0; 41],
            &ConfigKey::try_from("id").unwrap(),
        )
        .await;
        let board_id = board_id.unwrap().unwrap();
        let role = match board_id {
            2 | 0 => Role::GroundMain,
            3 => Role::GroundBackup,
            4 => Role::Cansat,
            1 => Role::Avionics,
            _ => panic!("Invalid board ID {}", 2)
        };

        unsafe { mission::ROLE = role };
        let tx_stream = streams.1;
        let rx_stream = streams.0;
        let i2c = dp
            .I2C1
            .i2c(i2c1_pins!(gpio), 400.kHz(), &clocks)
            .use_dma(tx_stream, rx_stream);

        static mut I2C_BUS: Option<UnsafeCell<I2c1Handle>> = None;
        static I2C_BUSY: i2c::AtomicBusyState = i2c::AtomicBusyState::new(i2c::BusyState::Free);
        // SAFETY: This is the only mutation of I2C_BUS so we have exclusive mutable access
        unsafe { I2C_BUS = Some(UnsafeCell::new(i2c)) };

        let bmp = if mission::role() != Role::GroundMain {
            #[cfg(feature = "target-mini")]
            {
                Some({
                    let alt_proxy = crate::pins::i2c::AtomicDevice::new(
                        unsafe { I2C_BUS.as_ref().unwrap() },
                        &I2C_BUSY,
                    );

                    altimeter::BMP388Wrapper::new(alt_proxy, &mut delay)
                })
            }

            #[cfg(feature = "target-maxi")]
            {
                let alt_proxy = crate::pins::i2c::AtomicDevice::new(
                    unsafe { I2C_BUS.as_ref().unwrap() },
                    &I2C_BUSY,
                );
                let mut bmp = crate::bmp581::BMP581::new(alt_proxy).unwrap();
                bmp.enable_pressure_temperature().unwrap();
                bmp.setup_fifo().unwrap();
                Some(bmp)
            }
        } else {
            None
        };
        let icm = if role != Role::GroundMain && cfg!(feature = "target-mini") {
            let icm_proxy = crate::pins::i2c::AtomicDevice::new(
                unsafe { I2C_BUS.as_ref().unwrap() },
                &I2C_BUSY,
            );
            let mut icm = IcmImu::new(icm_proxy, 0x68).unwrap();
            icm.enable_acc().unwrap();
            icm.enable_gyro().unwrap();
            Some(icm)
        } else {
            None
        };

        dp.I2C3.cr1.modify(|_, w| w.swrst().set_bit());
        delay.delay_ms(10);
        dp.I2C3.cr1.modify(|_, w| w.swrst().clear_bit());

        let i2c3 = dp.I2C3.i2c(
            (
                gpio.a
                    .pa8
                    .into_alternate()
                    .internal_pull_up(true)
                    .set_open_drain(),
                gpio.b
                    .pb4
                    .into_alternate()
                    .internal_pull_up(true)
                    .set_open_drain(),
            ),
            100.kHz(),
            &clocks,
        );

        let ms5607: Option<MS5607<I2c<pac::I2C3>, TimerDelay<TIM6>, _>> = {
            #[cfg(all(feature = "target-maxi", feature = "ultra-dev"))]
            {
                let delay = TimerDelay::new(dp.TIM6, clocks);
                let ms5607 = MS5607::new(i2c3, 0b1110110, delay).await.unwrap();
                let ms5607 = ms5607.calibrate().await.unwrap();
                Some(ms5607)
            }
            #[cfg(any(feature = "target-mini", not(feature = "ultra-dev")))]
            {
                None
            }
        };

        let bno080: Option<BNO080<I2cInterface<I2c<pac::I2C3>>>> = {
            #[cfg(all(feature = "target-maxi", not(feature = "ultra-dev")))]
            {
                let mut delay = dp.TIM6.delay::<100000>(&clocks);
                let mut bno = BNO080::new_with_interface(I2cInterface::alternate(i2c3));

                delay.delay_ms(10);

                bno.init(&mut delay).unwrap();
                delay.delay_ms(10);

                bno.enable_gyro(200).unwrap();
                bno.handle_all_messages(&mut delay, 5u8);
                bno.enable_linear_accel(200).unwrap();
                bno.handle_all_messages(&mut delay, 5u8);
                bno.enable_rotation_vector(10).unwrap();
                bno.handle_all_messages(&mut delay, 5u8);

                Some(bno)
            }

            #[cfg(any(feature = "target-mini", feature = "ultra-dev"))]
            {
                None
            }
        };

        let bmi323 = if cfg!(feature = "ultra-dev") {
            #[cfg(feature = "target-maxi")]
            {
                let bus = dp.SPI2.spi(
                    (gpio.b.pb13, gpio.c.pc2, gpio.c.pc3),
                    spi::Mode {
                        polarity: spi::Polarity::IdleLow,
                        phase: spi::Phase::CaptureOnFirstTransition,
                    },
                    100.kHz(),
                    &clocks,
                );
                static mut SPI2_BUS: Option<AtomicCell<Spi<SPI2>>> = None;
                unsafe {
                    SPI2_BUS.replace(AtomicCell::new(bus));
                }
                let device =
                    AtomicDevice::new_no_delay(unsafe { SPI2_BUS.as_ref().unwrap() }, gpio.e.pe12.into_push_pull_output()).unwrap();
                let mut bmi = Bmi323::new_with_spi(device, dp.TIM4.delay_us(&clocks));
                let accel_config = AccelConfig::builder()
                    .odr(OutputDataRate::Odr100hz)
                    .range(AccelerometerRange::G16)
                    .build();
                bmi.set_accel_config(accel_config).unwrap();

                // Configure gyroscope
                let gyro_config = GyroConfig::builder()
                    .odr(OutputDataRate::Odr100hz)
                    .range(GyroscopeRange::DPS2000)
                    .build();
                bmi.set_gyro_config(gyro_config).unwrap();

                Some(bmi)
            }

            #[cfg(feature = "target-mini")]
            {
                None
            }
        } else {
            None
        };

        // ===== Init SPI1 =====
        let spi1_mode = spi::Mode {
            polarity: spi::Polarity::IdleLow,
            phase: spi::Phase::CaptureOnFirstTransition,
        };
        let spi1_freq = 3.MHz();
        let (nss, sck, miso, mosi, nreset, busy, dio) = radio_pins!(gpio);
        let spi1_pins = (sck, miso, mosi);
        let spi1 = spi::Spi::new(dp.SPI1, spi1_pins, spi1_mode, spi1_freq, &clocks);

        let lora_nreset = nreset.into_push_pull_output_in_state(gpio::PinState::High);
        let lora_nss = nss.into_push_pull_output_in_state(gpio::PinState::High);
        let lora_busy = busy.into_floating_input();
        let lora_ant = DummyPin::new_high();

        static mut SPI_BUS: Option<AtomicCell<Spi<SPI1>>> = None;
        unsafe {
            SPI_BUS.replace(AtomicCell::new(spi1));
        }

        let spi1_device = embedded_hal_bus::spi::AtomicDevice::new(
            unsafe { SPI_BUS.as_ref().unwrap() },
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
        setup_logger(wrapper).unwrap();

        let mut lora = SX126x::new(spi1_device, lora_pins);
        lora.init(conf).unwrap();

        Radio::init(lora);
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::RTC_WKUP);
            pac::NVIC::unmask(pac::Interrupt::EXTI4);
            pac::NVIC::unpend(pac::Interrupt::RTC_WKUP);
            pac::NVIC::unpend(pac::Interrupt::EXTI4);
        }

        if matches!(role, Role::CansatBackup | Role::GroundBackup)
            || radio::TDM_CONFIG_MAIN.len() == 1
        {
            cortex_m::interrupt::free(|cs| {
                RTC.borrow(cs)
                    .borrow_mut()
                    .as_mut()
                    .unwrap()
                    .enable_wakeup(100u64.millis());
            });
        }

        neopixel::update_pixel(0, [0, 128, 0]);
        mission::begin(
            bmp,
            dp.TIM12.counter(&clocks),
            icm,
            TimerDelay::new(dp.TIM10, clocks),
            bno080,
            ms5607,
            bmi323,
        )
        .await;
    }
}

async fn build_config(flash: &mut W25QSequentialStorage<Bank1, CAPACITY>) -> sx126x::conf::Config {
    use sx126x::op::{
        modulation::lora::*, packet::lora::LoRaPacketParams, rxtx::DeviceSel::SX1262,
        PacketType::LoRa,
    };

    let sf: u64 = map::fetch_item(
        flash,
        CONFIG_FLASH_RANGE,
        &mut NoCache::new(),
        &mut [0; 128],
        &ConfigKey::try_from("sf").unwrap(),
    )
    .await
    .unwrap()
    .unwrap_or(7);
    let sf = match sf {
        5 => LoRaSpreadFactor::SF5,
        6 => LoRaSpreadFactor::SF6,
        7 => LoRaSpreadFactor::SF7,
        8 => LoRaSpreadFactor::SF8,
        9 => LoRaSpreadFactor::SF9,
        10 => LoRaSpreadFactor::SF10,
        11 => LoRaSpreadFactor::SF11,
        12 => LoRaSpreadFactor::SF12,
        _ => LoRaSpreadFactor::SF7,
    };

    let bw: u64 = map::fetch_item(
        flash,
        CONFIG_FLASH_RANGE,
        &mut NoCache::new(),
        &mut [0; 128],
        &ConfigKey::try_from("bw").unwrap(),
    )
    .await
    .unwrap()
    .unwrap_or(250);

    let bw = match bw {
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
        _ => LoRaBandWidth::BW250,
    };

    let cr: u64 = map::fetch_item(
        flash,
        CONFIG_FLASH_RANGE,
        &mut NoCache::new(),
        &mut [0; 128],
        &ConfigKey::try_from("cr").unwrap(),
    )
    .await
    .unwrap()
    .unwrap_or(8);

    let cr = match cr {
        5 => LoraCodingRate::CR4_5,
        6 => LoraCodingRate::CR4_6,
        7 => LoraCodingRate::CR4_7,
        8 => LoraCodingRate::CR4_8,
        _ => LoraCodingRate::CR4_5,
    };

    let mut power: u64 = map::fetch_item(
        flash,
        CONFIG_FLASH_RANGE,
        &mut NoCache::new(),
        &mut [0; 128],
        &ConfigKey::try_from("power").unwrap(),
    )
    .await
    .unwrap()
    .unwrap_or(22);
    if power > 22 {
        power = 22;
    }

    let rf_freq: u64 = map::fetch_item(
        flash,
        CONFIG_FLASH_RANGE,
        &mut NoCache::new(),
        &mut [0; 128],
        &ConfigKey::try_from("rf_freq").unwrap(),
    )
    .await
    .unwrap()
    .unwrap_or(868_000_000);

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
        rf_frequency: rf_freq,
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
    let mut buzzer = unsafe { BUZZER.take() }.unwrap();

    loop {
        buzzer.toggle();
        neopixel::update_pixel(1, [255, 0, 0]);
        nb::block!(timer.wait()).ok();

        buzzer.toggle();
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
