use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use smart_leds::SmartLedsWrite;
use ws2812_spi::Ws2812;
use stm32f4xx_hal::spi::Spi;
use crate::pins::NeopixelSPI;

static BUFFER: Mutex<RefCell<[[u8; 3]; 4]>> = Mutex::new(RefCell::new([[0,0,0]; 4]));
pub static NEOPIXEL: Mutex<RefCell<Option<Ws2812<Spi<NeopixelSPI>>>>> = Mutex::new(RefCell::new(None));

    /// Creates a new neopixel controller passing a neopixel SPI Bus.
    pub fn new_neopixel(neopixel_spi: ws2812_spi::Ws2812<Spi<NeopixelSPI>>) {
        cortex_m::interrupt::free(|cs| {
            NEOPIXEL.borrow(cs).borrow_mut().replace(neopixel_spi);
        });
    }

    /// Updates the color of a given neopixel, with a given RGB value.
    pub fn update_pixel(pixel: usize, color: [u8; 3]) {

        cortex_m::interrupt::free(|cs| {
            let mut neo_ref = NEOPIXEL.borrow(cs).borrow_mut();
            let mut buffer_ref = BUFFER.borrow(cs).borrow_mut();

            buffer_ref[pixel] = [color[0], color[1], color[2]];

            let _ = neo_ref
                .as_mut()
                .unwrap()
                .write(buffer_ref.into_iter());
        });
    }
