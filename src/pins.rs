#![allow(unused_imports)]
use bmp388::{BMP388, Blocking};
use stm32f4xx_hal::{
    gpio::{gpioa, gpiob, gpioc, gpiod, gpioe, Alternate, Output, PushPull}, i2c::I2c, pac::{I2C1, SPI2, SPI3}
};

use crate::bmp581::{BMP581, I2c1Handle};
use crate::altimeter::BMP388Wrapper;

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

#[cfg(feature = "target-mini")]
pub type Altimeter = BMP388Wrapper;

#[cfg(feature = "target-maxi")]
pub type Altimeter = BMP581;

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
macro_rules! qspi_pins {
    ($gpio:ident) => {{
        #[cfg(feature = "target-mini")]
        {
            // (ncs, io0, io1, io2, io3, clk)
            ($gpio.b.pb6, $gpio.c.pc9, $gpio.c.pc10, $gpio.c.pc8, $gpio.a.pa1, $gpio.b.pb1)
        }
        #[cfg(feature = "target-maxi")]
        {
            // (ncs, io0, io1, io2, io3, clk)
            ($gpio.b.pb6, $gpio.d.pd11, $gpio.d.pd12, $gpio.e.pe2, $gpio.a.pa1, $gpio.b.pb1)
        }
    }};
}


#[macro_export]
macro_rules! radio_pins {
    ($gpio:ident) => {{
        #[cfg(feature = "target-mini")]
        {
            // (nss, sck, miso, mosi, reset, busy, dio1)
            ($gpio.a.pa4, $gpio.a.pa5, $gpio.a.pa6, $gpio.a.pa7, $gpio.b.pb0, $gpio.c.pc5, $gpio.c.pc4)
        }

        #[cfg(feature = "target-maxi")]
        {
            ($gpio.a.pa4, $gpio.a.pa5, $gpio.a.pa6, $gpio.a.pa7, $gpio.b.pb0, $gpio.c.pc5, $gpio.c.pc4)
        }
    }};
}
