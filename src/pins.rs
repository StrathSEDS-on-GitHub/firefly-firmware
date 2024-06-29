#![allow(unused_imports)]
use core::cell::RefCell;

use bmp388::{Blocking, BMP388};
use cortex_m::interrupt::Mutex;
use stm32f4xx_hal::{
    dma::{Stream0, Stream1},
    gpio::{gpioa, gpiob, gpioc, gpiod, gpioe, Alternate, Output, PushPull},
    i2c::{
        dma::{I2CMasterDma, RxDMA, TxDMA},
        I2c,
    },
    pac::{DMA1, I2C1, SPI2, SPI3},
    timer::Delay,
};

use crate::altimeter::BMP388Wrapper;
use crate::bmp581::BMP581;

pub struct GpioBuses {
    pub a: gpioa::Parts,
    pub b: gpiob::Parts,
    pub c: gpioc::Parts,
    pub d: gpiod::Parts,
    pub e: gpioe::Parts,
}

#[cfg(feature = "target-mini")]
pub type BuzzerPin = gpiob::PB5<Output<PushPull>>;
#[cfg(feature = "target-maxi")]
pub type BuzzerPin = gpioe::PE1<Output<PushPull>>;

#[cfg(feature = "target-mini")]
pub type NeopixelPin = gpioc::PC3<Output<PushPull>>;
#[cfg(feature = "target-mini")]
pub type NeopixelSPI = SPI2;
#[cfg(feature = "target-maxi")]
pub type NeopixelPin = gpioc::PC12<Output<PushPull>>;
#[cfg(feature = "target-maxi")]
pub type NeopixelSPI = SPI3;

#[cfg(feature = "target-mini")]
pub type GPSPins = (gpioa::PA9<Alternate<7>>, gpioa::PA10<Alternate<7>>);

#[cfg(feature = "target-maxi")]
pub type GPSPins = (gpioa::PA15<Alternate<7>>, gpioa::PA10<Alternate<7>>);

pub type I2c1Handle =
    I2CMasterDma<I2C1, TxDMA<I2C1, Stream1<DMA1>, 0>, RxDMA<I2C1, Stream0<DMA1>, 1>>;

pub mod i2c {
    use core::{
        cell::{RefCell, UnsafeCell},
        fmt::Write,
        sync::atomic::{AtomicBool, Ordering},
    };
    use cortex_m_semihosting::hprintln;
    use embedded_hal::{
        delay::DelayNs,
        i2c::{ErrorType, I2c},
    };
    use embedded_hal_bus::i2c::AtomicError;
    use stm32f4xx_hal::i2c::dma::{I2CMasterHandleIT, I2CMasterWriteReadDMA};

    /// Replacement for embedded-hal-bus AtomicDevice that also implements I2CMasterHandleIT
    pub struct AtomicDevice<'a, T> {
        bus: &'a UnsafeCell<T>,
        busy: &'a AtomicBusyState,
    }
    unsafe impl<'a, T> Send for AtomicDevice<'a, T> {}

    #[derive(PartialEq, Eq)]
    #[atomic_enum::atomic_enum]
    pub enum BusyState {
        Free,
        Busy,
        BusyDMA, // Only handle_dma_interrupt is allowed
    }

    impl<'a, T> AtomicDevice<'a, T> {
        #[inline]
        pub fn new(bus: &'a UnsafeCell<T>, busy: &'a AtomicBusyState) -> Self {
            busy.store(BusyState::Free, Ordering::Relaxed);
            Self { bus, busy }
        }
    }

    impl<'a, T> ErrorType for AtomicDevice<'a, T>
    where
        T: I2c,
    {
        type Error = AtomicError<T::Error>;
    }

    impl<'a, T: I2c> AtomicDevice<'a, T> {
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

    impl<'a, T> I2c for AtomicDevice<'a, T>
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

    impl<'a, T: I2c + I2CMasterHandleIT> I2CMasterHandleIT for AtomicDevice<'a, T> {
        fn handle_dma_interrupt(&mut self) {
            match self.busy.load(Ordering::Relaxed) {
                BusyState::BusyDMA => {
                    let bus = unsafe { &mut *self.bus.get() };
                    bus.handle_dma_interrupt();
                }
                _ => panic!("Invalid state for DMA interrupt"),
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

    impl<'a, T> I2CMasterWriteReadDMA for AtomicDevice<'a, T>
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
                    Ordering::Acquire,
                    Ordering::Relaxed,
                )
                .map_err(|_| nb::Error::WouldBlock)?;
            let bus = &mut *self.bus.get();
            bus.write_read_dma(addr, bytes, buf, callback)
        }
    }
}

pub type I2c1Proxy = i2c::AtomicDevice<'static, I2c1Handle>;

#[cfg(feature = "target-mini")]
pub type Altimeter = BMP388Wrapper;

#[cfg(feature = "target-maxi")]
pub type Altimeter = BMP581<I2c1Proxy>;

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
        #[cfg(feature = "target-maxi")]
        {
            (
                $gpio_buses.a.pa15.into_alternate(),
                $gpio_buses.a.pa10.into_alternate(),
            )
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
    }};
}

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
    }};
}
