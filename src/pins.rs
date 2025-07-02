use stm32f4xx_hal::gpio::{Output, PushPull, gpioa, gpiob, gpioc, gpiod, gpioe};

pub struct GpioBuses {
    pub a: gpioa::Parts,
    pub b: gpiob::Parts,
    pub c: gpioc::Parts,
    pub d: gpiod::Parts,
    pub e: gpioe::Parts,
}

#[cfg(feature = "target-mini")]
pub const TARGET: &str = "Firefly Mini";
#[cfg(feature = "target-maxi")]
pub const TARGET: &str = "Firefly Maxi";
#[cfg(feature = "target-ultra")]
pub const TARGET: &str = "Firefly Ultra";

#[cfg(feature = "target-mini")]
pub type BuzzerPin = gpiob::PB5<Output<PushPull>>;
#[cfg(feature = "target-maxi")]
pub type BuzzerPin = gpioe::PE1<Output<PushPull>>;
#[cfg(feature = "target-ultra")]
pub type BuzzerPin = gpioc::PC1<Output<PushPull>>;

#[cfg(feature = "target-mini")]
pub type NeopixelSPI = stm32f4xx_hal::pac::SPI2;
#[cfg(feature = "target-maxi")]
pub type NeopixelSPI = stm32f4xx_hal::pac::SPI3;
#[cfg(feature = "target-ultra")]
pub type NeopixelSPI = stm32f4xx_hal::pac::SPI3;

#[allow(unused_imports)]
pub mod i2c {
    use core::{cell::UnsafeCell, sync::atomic::Ordering};
    use embedded_hal::i2c::{ErrorType, I2c};
    use embedded_hal_bus::i2c::AtomicError;
    use stm32f4xx_hal::{
        dma::{Stream0, Stream1},
        i2c::dma::{I2CMasterDma, I2CMasterHandleIT, I2CMasterWriteReadDMA, RxDMA, TxDMA},
        pac::{DMA1, I2C1, I2C3, TIM6},
    };
    pub type I2c1Handle =
        I2CMasterDma<I2C1, TxDMA<I2C1, Stream1<DMA1>, 0>, RxDMA<I2C1, Stream0<DMA1>, 1>>;

    #[cfg(not(feature = "target-ultra"))]
    pub type I2c1Proxy = DMAAtomicDevice<'static, I2c1Handle>;

    #[cfg(feature = "target-mini")]
    pub type Altimeter = crate::bmp388::BMP388Wrapper;

    #[cfg(all(feature = "target-maxi", not(feature = "ultra-dev")))]
    pub type Altimeter = crate::bmp581::BMP581<I2c1Proxy>;

    #[cfg(any(feature = "target-ultra", feature = "ultra-dev"))]
    pub type Altimeter = crate::ms5607::MS5607<
        stm32f4xx_hal::i2c::I2c<I2C3>,
        crate::futures::TimerDelay<TIM6>,
        crate::ms5607::Calibrated,
    >;

    #[macro_export]
    macro_rules! i2c_dma_streams {
        ($dp:ident, $gps_streams:ident) => {{
            #[cfg(any(
                feature = "target-mini",
                all(feature = "target-maxi", not(feature = "ultra-dev"))
            ))]
            {
                let streams = stm32f4xx_hal::dma::StreamsTuple::new($dp.DMA1);
                (streams.0, streams.1)
            }

            #[cfg(all(feature = "target-maxi", feature = "ultra-dev"))]
            {
                ($gps_streams.0, $gps_streams.1)
            }
        }};
    }

    /// Replacement for embedded-hal-bus AtomicDevice that also implements I2CMasterHandleIT
    pub struct DMAAtomicDevice<'a, T> {
        bus: &'a UnsafeCell<T>,
        busy: &'a AtomicBusyState,
    }
    unsafe impl<'a, T> Send for DMAAtomicDevice<'a, T> {}

    #[derive(PartialEq, Eq)]
    #[atomic_enum::atomic_enum]
    pub enum BusyState {
        Free,
        Busy,
        BusyDMA, // Only handle_dma_interrupt is allowed
    }

    impl<'a, T> DMAAtomicDevice<'a, T> {
        #[inline]
        pub fn new(bus: &'a UnsafeCell<T>, busy: &'a AtomicBusyState) -> Self {
            busy.store(BusyState::Free, Ordering::Relaxed);
            Self { bus, busy }
        }
    }

    impl<'a, T> ErrorType for DMAAtomicDevice<'a, T>
    where
        T: I2c,
    {
        type Error = AtomicError<T::Error>;
    }

    impl<'a, T: I2c> DMAAtomicDevice<'a, T> {
        fn lock<R, F>(&self, f: F) -> Result<R, AtomicError<T::Error>>
        where
            F: FnOnce(&mut T) -> Result<R, T::Error>,
        {
            self.busy
                .compare_exchange_weak(
                    BusyState::Free,
                    BusyState::Busy,
                    Ordering::Acquire,
                    Ordering::Relaxed,
                )
                .map_err(|_| embedded_hal_bus::i2c::AtomicError::Busy)?;
            let result = f(unsafe { &mut *self.bus.get() });

            self.busy.store(BusyState::Free, Ordering::SeqCst);

            result.map_err(AtomicError::Other)
        }

        #[allow(unused)]
        pub unsafe fn dma_complete(&self) -> Result<(), ()> {
            self.busy
                .compare_exchange(
                    BusyState::BusyDMA,
                    BusyState::Free,
                    Ordering::Acquire,
                    Ordering::Relaxed,
                )
                .map_err(|_| ())
                .map(|_| ())
        }
    }

    impl<'a, T> I2c for DMAAtomicDevice<'a, T>
    where
        T: I2c,
    {
        #[inline]
        fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
            self.lock(|bus| bus.read(address, read))
        }

        #[inline]
        fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
            self.lock(|bus| bus.write(address, write))
        }

        #[inline]
        fn write_read(
            &mut self,
            address: u8,
            write: &[u8],
            read: &mut [u8],
        ) -> Result<(), Self::Error> {
            self.lock(|bus| bus.write_read(address, write, read))
        }

        #[inline]
        fn transaction(
            &mut self,
            address: u8,
            operations: &mut [embedded_hal::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            self.lock(|bus| bus.transaction(address, operations))
        }
    }

    impl<'a, T: I2c + I2CMasterHandleIT> I2CMasterHandleIT for DMAAtomicDevice<'a, T> {
        fn handle_dma_interrupt(&mut self) {
            match self.busy.load(Ordering::Relaxed) {
                BusyState::BusyDMA => {
                    let bus = unsafe { &mut *self.bus.get() };
                    bus.handle_dma_interrupt();
                }
                s => panic!("Invalid state for DMA interrupt {:?}", s),
            }
        }

        fn handle_error_interrupt(&mut self) {
            match self.busy.load(Ordering::Relaxed) {
                BusyState::BusyDMA => {
                    let bus = unsafe { &mut *self.bus.get() };
                    bus.handle_error_interrupt();
                }
                _ => panic!("Invalid state for DMA interrupt"),
            }
        }
    }

    impl<'a, T> I2CMasterWriteReadDMA for DMAAtomicDevice<'a, T>
    where
        T: I2c + I2CMasterWriteReadDMA,
    {
        unsafe fn write_read_dma(
            &mut self,
            addr: u8,
            bytes: &[u8],
            buf: &mut [u8],
            callback: Option<stm32f4xx_hal::i2c::dma::I2cCompleteCallback>,
        ) -> nb::Result<(), stm32f4xx_hal::i2c::Error> {
            self.busy
                .compare_exchange_weak(
                    BusyState::Free,
                    BusyState::BusyDMA,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                )
                .map_err(|_| nb::Error::WouldBlock)?;
            unsafe {
                let bus = &mut *self.bus.get();
                bus.write_read_dma(addr, bytes, buf, callback)
            }
        }
    }
}

#[macro_export]
macro_rules! i2c3_pins {
    ($gpio:ident) => {{
        #[cfg(feature = "target-maxi")]
        {
            (
                $gpio
                    .a
                    .pa8
                    .into_alternate()
                    .internal_pull_up(true)
                    .set_open_drain(),
                $gpio
                    .b
                    .pb4
                    .into_alternate()
                    .internal_pull_up(true)
                    .set_open_drain(),
            )
        }

        #[cfg(feature = "target-ultra")]
        {
            (
                $gpio
                    .a
                    .pa8
                    .into_alternate()
                    .internal_pull_up(true)
                    .set_open_drain(),
                $gpio
                    .c
                    .pc9
                    .into_alternate()
                    .internal_pull_up(true)
                    .set_open_drain(),
            )
        }
    }};
}

#[macro_export]
macro_rules! buzzer_pin {
    ($gpio_buses:ident) => {{
        #[cfg(feature = "target-mini")]
        {
            $gpio_buses.b.pb5.into_push_pull_output()
        }
        #[cfg(feature = "target-maxi")]
        {
            $gpio_buses.e.pe1.into_push_pull_output()
        }
        #[cfg(feature = "target-ultra")]
        {
            $gpio_buses.c.pc1.into_push_pull_output()
        }
    }};
}

#[macro_export]
macro_rules! neopixel_pin {
    ($gpio_buses:ident) => {{
        #[cfg(feature = "target-mini")]
        {
            $gpio_buses.c.pc3
        }
        #[cfg(feature = "target-maxi")]
        {
            $gpio_buses.c.pc12
        }
        #[cfg(feature = "target-ultra")]
        {
            $gpio_buses.c.pc12
        }
    }};
}

#[macro_export]
macro_rules! neopixel_spi {
    ($dp:ident) => {{
        #[cfg(feature = "target-mini")]
        {
            $dp.SPI2
        }
        #[cfg(feature = "target-maxi")]
        {
            $dp.SPI3
        }
        #[cfg(feature = "target-ultra")]
        {
            $dp.SPI3
        }
    }};
}

#[macro_export]
macro_rules! i2c1_pins {
    ($gpio_buses:ident) => {{
        #[cfg(feature = "target-mini")]
        {
            ($gpio_buses.b.pb8, $gpio_buses.b.pb7)
        }
        #[cfg(feature = "target-maxi")]
        {
            ($gpio_buses.b.pb8, $gpio_buses.b.pb9)
        }
        #[cfg(feature = "target-ultra")]
        {
            ($gpio_buses.b.pb6, $gpio_buses.b.pb7)
        }
    }};
}

#[macro_export]
macro_rules! imu_spi_pins {
    ($gpio:ident) => {{
        #[cfg(not(feature = "target-ultra"))]
        {
            ($gpio.b.pb13, $gpio.c.pc2, $gpio.c.pc3)
        }
        #[cfg(feature = "target-ultra")]
        {
            ($gpio.a.pa5, $gpio.b.pb4.into_alternate(), $gpio.b.pb5)
        }
    }};
}

#[cfg(feature = "target-ultra")]
pub type QspiBank = stm32f4xx_hal::qspi::Bank2;
#[cfg(not(feature = "target-ultra"))]
pub type QspiBank = stm32f4xx_hal::qspi::Bank1;

#[macro_export]
macro_rules! qspi_pins {
    ($gpio:ident) => {{
        #[cfg(feature = "target-mini")]
        {
            // (ncs, io0, io1, io2, io3, clk)
            (
                $gpio.b.pb6,
                $gpio.c.pc9,
                $gpio.c.pc10,
                $gpio.c.pc8,
                $gpio.a.pa1,
                $gpio.b.pb1,
            )
        }
        #[cfg(feature = "target-maxi")]
        {
            // (ncs, io0, io1, io2, io3, clk)
            (
                $gpio.b.pb6,
                $gpio.d.pd11,
                $gpio.d.pd12,
                $gpio.e.pe2,
                $gpio.a.pa1,
                $gpio.b.pb1,
            )
        }
        #[cfg(feature = "target-ultra")]
        {
            // (ncs, io0, io1, io2, io3, clk)
            (
                $gpio.c.pc11,
                $gpio.a.pa6,
                $gpio.a.pa7,
                $gpio.c.pc4,
                $gpio.c.pc5,
                $gpio.b.pb1,
            )
        }
    }};
}

#[cfg(not(feature = "target-ultra"))]
pub type ImuSpi = stm32f4xx_hal::pac::SPI2;

#[cfg(feature = "target-ultra")]
pub type ImuSpi = stm32f4xx_hal::pac::SPI1;

#[macro_export]
macro_rules! imus_spi {
    ($dp:ident) => {{
        #[cfg(not(feature = "target-ultra"))]
        {
            $dp.SPI2
        }
        #[cfg(feature = "target-ultra")]
        {
            $dp.SPI1
        }
    }};
}

#[macro_export]
macro_rules! lrhp_nss {
    ($gpio:ident) => {{
        #[cfg(not(feature = "target-ultra"))]
        {
            $gpio.e.pe12.into_push_pull_output()
        }
        #[cfg(feature = "target-ultra")]
        {
            $gpio.b.pb12.into_push_pull_output()
        }
    }};
}

#[macro_export]
macro_rules! hrlp_nss {
    ($gpio:ident) => {{
        #[cfg(not(feature = "target-ultra"))]
        {
            $gpio.e.pe10.into_push_pull_output()
        }
        #[cfg(feature = "target-ultra")]
        {
            $gpio.b.pb2.into_push_pull_output()
        }
    }};
}

pub mod gps {
    use stm32f4xx_hal::gpio::{Alternate, gpioa};

    #[cfg(feature = "target-maxi")]
    pub type PpsPin = stm32f4xx_hal::gpio::gpioe::PE11<stm32f4xx_hal::gpio::Input>;

    #[cfg(feature = "target-mini")]
    pub type PpsPin = stm32f4xx_hal::gpio::gpioa::PA2<stm32f4xx_hal::gpio::Input>;

    #[cfg(feature = "target-ultra")]
    pub type PpsPin = stm32f4xx_hal::gpio::gpioc::PC8<stm32f4xx_hal::gpio::Input>;

    #[macro_export]
    macro_rules! pps_pin {
        ($gpio_buses:ident) => {{
            #[cfg(all(feature = "target-maxi", feature = "ultra-dev"))]
            {
                $gpio_buses.e.pe11.into_input()
            }
            #[cfg(all(feature = "target-maxi", not(feature = "ultra-dev")))]
            {
                $gpio_buses.e.pe11.into_input()
            }
            #[cfg(all(feature = "target-mini"))]
            {
                $gpio_buses.a.pa2.into_input()
            }
            #[cfg(feature = "target-ultra")]
            {
                $gpio_buses.c.pc8.into_input()
            }
        }};
    }

    #[cfg(feature = "target-mini")]
    pub type GPSPins = (gpioa::PA9<Alternate<7>>, gpioa::PA10<Alternate<7>>);

    #[cfg(feature = "target-ultra")]
    pub type GPSPins = (gpioa::PA9<Alternate<7>>, gpioa::PA10<Alternate<7>>);

    #[cfg(all(feature = "target-maxi", not(feature = "ultra-dev")))]
    pub type GPSPins = (gpioa::PA15<Alternate<7>>, gpioa::PA10<Alternate<7>>);

    #[cfg(all(feature = "target-maxi", feature = "ultra-dev"))]
    pub type GPSPins = (gpioa::PA2<Alternate<7>>, gpioa::PA3<Alternate<7>>);

    #[cfg(any(
        feature = "target-mini",
        all(feature = "target-maxi", not(feature = "ultra-dev")),
        feature = "target-ultra"
    ))]
    pub type GPSUsart = stm32f4xx_hal::pac::USART1;

    #[cfg(all(feature = "target-maxi", feature = "ultra-dev"))]
    pub type GPSUsart = stm32f4xx_hal::pac::USART2;

    #[cfg(any(
        feature = "target-mini",
        all(feature = "target-maxi", not(feature = "ultra-dev")),
        feature = "target-ultra"
    ))]
    pub type GPSDma = stm32f4xx_hal::pac::DMA2;

    #[cfg(all(feature = "target-maxi", feature = "ultra-dev"))]
    pub type GPSDma = stm32f4xx_hal::pac::DMA1;

    #[cfg(all(feature = "target-maxi", feature = "ultra-dev"))]
    pub type GPSRxStream = stm32f4xx_hal::dma::Stream5<GPSDma>;

    #[cfg(any(
        feature = "target-mini",
        all(feature = "target-maxi", not(feature = "ultra-dev")),
        feature = "target-ultra"
    ))]
    pub type GPSRxStream = stm32f4xx_hal::dma::Stream2<GPSDma>;

    #[macro_export]
    macro_rules! gps_rx_stream {
        ($streams:ident) => {{
            #[cfg(any(
                feature = "target-mini",
                all(feature = "target-maxi", not(feature = "ultra-dev")),
                feature = "target-ultra"
            ))]
            {
                $streams.2
            }

            #[cfg(all(feature = "target-maxi", feature = "ultra-dev"))]
            {
                $streams.5
            }
        }};
    }

    #[macro_export]
    macro_rules! gps_usart_peripheral {
        ($dp:ident) => {{
            #[cfg(any(
                feature = "target-mini",
                all(feature = "target-maxi", not(feature = "ultra-dev")),
                feature = "target-ultra"
            ))]
            {
                $dp.USART1
            }

            #[cfg(all(feature = "target-maxi", feature = "ultra-dev"))]
            {
                $dp.USART2
            }
        }};
    }

    #[macro_export]
    macro_rules! gps_serial_baudrate {
        () => {{
            #[cfg(any(
                feature = "target-mini",
                all(feature = "target-maxi", not(feature = "ultra-dev")),
            ))]
            {
                9600.bps()
            }

            #[cfg(any(
                all(feature = "target-maxi", feature = "ultra-dev"),
                feature = "target-ultra"
            ))]
            {
                115200.bps()
            }
        }};
    }

    #[macro_export]
    macro_rules! gps_dma_streams {
        ($dp:ident) => {{
            #[cfg(any(
                feature = "target-mini",
                all(feature = "target-maxi", not(feature = "ultra-dev")),
                feature = "target-ultra"
            ))]
            {
                stm32f4xx_hal::dma::StreamsTuple::new($dp.DMA2)
            }

            #[cfg(all(feature = "target-maxi", feature = "ultra-dev"))]
            {
                stm32f4xx_hal::dma::StreamsTuple::new($dp.DMA1)
            }
        }};
    }

    #[macro_export]
    macro_rules! gps_pins {
        ($gpio_buses:ident) => {{
            #[cfg(feature = "target-mini")]
            {
                (
                    $gpio_buses.a.pa9.into_alternate(),
                    $gpio_buses.a.pa10.into_alternate(),
                )
            }
            #[cfg(all(feature = "target-maxi", not(feature = "ultra-dev")))]
            {
                (
                    $gpio_buses.a.pa15.into_alternate(),
                    $gpio_buses.a.pa10.into_alternate(),
                )
            }
            #[cfg(all(feature = "target-maxi", feature = "ultra-dev"))]
            {
                (
                    $gpio_buses.a.pa2.into_alternate(),
                    $gpio_buses.a.pa3.into_alternate(),
                )
            }
            #[cfg(feature = "target-ultra")]
            {
                (
                    $gpio_buses.a.pa9.into_alternate(),
                    $gpio_buses.a.pa10.into_alternate(),
                )
            }
        }};
    }
}
pub mod radio {
    #[cfg(not(feature = "target-ultra"))]
    pub type RadioInteruptPin = stm32f4xx_hal::gpio::gpioc::PC4<stm32f4xx_hal::gpio::Input>;
    #[cfg(feature = "target-ultra")]
    pub type RadioInteruptPin = stm32f4xx_hal::gpio::gpioa::PA15<stm32f4xx_hal::gpio::Input>;

    #[cfg(not(feature = "target-ultra"))]
    pub type RadioSpi = stm32f4xx_hal::pac::SPI1;
    #[cfg(feature = "target-ultra")]
    pub type RadioSpi = stm32f4xx_hal::pac::SPI2;

    #[cfg(not(feature = "target-ultra"))]
    pub type RadioResetPin =
        stm32f4xx_hal::gpio::gpiob::PB0<stm32f4xx_hal::gpio::Output<stm32f4xx_hal::gpio::PushPull>>;
    #[cfg(feature = "target-ultra")]
    pub type RadioResetPin =
        stm32f4xx_hal::gpio::gpiob::PB8<stm32f4xx_hal::gpio::Output<stm32f4xx_hal::gpio::PushPull>>;

    #[cfg(not(feature = "target-ultra"))]
    pub type RadioBusyPin = stm32f4xx_hal::gpio::gpioc::PC5<stm32f4xx_hal::gpio::Input>;
    #[cfg(feature = "target-ultra")]
    pub type RadioBusyPin = stm32f4xx_hal::gpio::gpiod::PD2<stm32f4xx_hal::gpio::Input>;

    #[cfg(not(feature = "target-ultra"))]
    pub type RadioChipSelect =
        stm32f4xx_hal::gpio::gpioa::PA4<stm32f4xx_hal::gpio::Output<stm32f4xx_hal::gpio::PushPull>>;
    #[cfg(feature = "target-ultra")]
    pub type RadioChipSelect =
        stm32f4xx_hal::gpio::gpioc::PC0<stm32f4xx_hal::gpio::Output<stm32f4xx_hal::gpio::PushPull>>;

    #[macro_export]
    macro_rules! radio_spi {
        ($dp:ident) => {{
            #[cfg(not(feature = "target-ultra"))]
            {
                $dp.SPI1
            }
            #[cfg(feature = "target-ultra")]
            {
                $dp.SPI2
            }
        }};
    }

    #[macro_export]
    macro_rules! radio_pins {
        ($gpio:ident) => {{
            #[cfg(feature = "target-mini")]
            {
                // (nss, sck, miso, mosi, reset, busy, dio1)
                (
                    $gpio.a.pa4,
                    $gpio.a.pa5,
                    $gpio.a.pa6,
                    $gpio.a.pa7,
                    $gpio.b.pb0,
                    $gpio.c.pc5,
                    $gpio.c.pc4,
                )
            }

            #[cfg(feature = "target-maxi")]
            {
                (
                    $gpio.a.pa4,
                    $gpio.a.pa5,
                    $gpio.a.pa6,
                    $gpio.a.pa7,
                    $gpio.b.pb0,
                    $gpio.c.pc5,
                    $gpio.c.pc4,
                )
            }

            #[cfg(feature = "target-ultra")]
            {
                (
                    $gpio.c.pc0,
                    $gpio.b.pb13,
                    $gpio.c.pc2,
                    $gpio.c.pc3,
                    $gpio.b.pb8,
                    $gpio.d.pd2,
                    $gpio.a.pa15,
                )
            }
        }};
    }
}

pub mod pyro {
    #[cfg(feature = "target-mini")]
    use dummy_pin::DummyPin;
    use stm32f4xx_hal::gpio::{Analog, Output, Pin};

    #[cfg(feature = "target-mini")]
    pub type PyroEnable = DummyPin;
    #[cfg(feature = "target-mini")]
    pub type PyroFire2 = DummyPin;
    #[cfg(feature = "target-mini")]
    pub type PyroFire1 = DummyPin;
    #[cfg(feature = "target-mini")]
    pub type PyroCont1 = DummyPin;
    #[cfg(feature = "target-mini")]
    pub type PyroCont2 = DummyPin;

    #[cfg(feature = "target-maxi")]
    pub type PyroEnable = Pin<'D', 7, Output>;
    #[cfg(feature = "target-maxi")]
    pub type PyroFire2 = Pin<'D', 6, Output>;
    #[cfg(feature = "target-maxi")]
    pub type PyroFire1 = Pin<'D', 5, Output>;
    #[cfg(feature = "target-maxi")]
    pub type PyroCont1 = Pin<'D', 3, Analog>;
    #[cfg(feature = "target-maxi")]
    pub type PyroCont2 = Pin<'D', 4, Analog>;

    #[cfg(feature = "target-ultra")]
    pub type PyroEnable = Pin<'A', 1, Output>;
    #[cfg(feature = "target-ultra")]
    pub type PyroFire2 = Pin<'B', 15, Output>;
    #[cfg(feature = "target-ultra")]
    pub type PyroFire1 = Pin<'C', 6, Output>;
    #[cfg(feature = "target-ultra")]
    pub type PyroCont1 = Pin<'A', 4, Analog>;
    #[cfg(feature = "target-ultra")]
    pub type PyroCont2 = Pin<'B', 0, Analog>;

    /// (enable, fire2, fire1, cont2, cont1)
    #[macro_export]
    macro_rules! pyro_pins {
        ($gpio:ident) => {{
            #[cfg(feature = "target-mini")]
            {
                (
                    DummyPin::new_low(),
                    DummyPin::new_low(),
                    DummyPin::new_low(),
                    DummyPin::new_low(),
                    DummyPin::new_low(),
                )
            }

            #[cfg(feature = "target-maxi")]
            {
                (
                    $gpio.d.pd7.into_push_pull_output(),
                    $gpio.d.pd6.into_push_pull_output(),
                    $gpio.d.pd5.into_push_pull_output(),
                    $gpio.d.pd4.into_analog(),
                    $gpio.d.pd3.into_analog(),
                )
            }

            #[cfg(feature = "target-ultra")]
            {
                (
                    $gpio.a.pa1.into_push_pull_output(),
                    $gpio.b.pb15.into_push_pull_output(),
                    $gpio.c.pc6.into_push_pull_output(),
                    $gpio.b.pb0.into_analog(),
                    $gpio.a.pa4.into_analog(),
                )
            }
        }};
    }
}
