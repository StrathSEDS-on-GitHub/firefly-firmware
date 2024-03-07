// Right now, we use some godawful bitbanging directly to the neopixel. This is bad as we can't fucking use the neopixel without optimisations otherwise the bitbanging is too slow. It also unnecessarily hogs the CPU.

// In addition, because all the neopixels are wired together in series and we don't have any abstractions for this, 
// any write to neopixels affects all the previous neopixels in the chain.

// Firefly 1.1 will have the Neopixel wired to an SPI bus so we can use this well known 
// hack to drive the Neopixel using the much faster SPI bus: https://learn.adafruit.com/circuitpython-neopixels-using-spi/overview

// In addition, we should add an abstraction that allows writes to individual neopixels 
// without affecting the others. I would recommend perhaps doing this in a way that allows the 
// Neopixel controller task to be provided with lambdas for each pixel so we can have 4 different 
// patterns going on at the same time. We could use each Neopixel for different sensors to monitor them all at a glance.

// use a buffer like [neo1, neo2, neo3, neo4] where each value is an rgb value, initialised to [0,0,0] for each pixel
// when a neopixel is to be updated, the buffer is updated and the neopixel controller task is notified of the change

use core::cell::RefCell;

use crate::hal::gpio::{Output, Pin};
use crate::hal::timer::CounterHz;
use cortex_m::interrupt::Mutex;
use stm32f4xx_hal::pac::TIM2;
use ws2812_timer_delay::Ws2812;
use smart_leds::{SmartLedsWrite, RGB};


    // n: Ws2812<CounterHz<TIM2>, Pin<'A', 9, Output>>;
static BUFFER: Mutex<RefCell<[[i32; 3]; 4]>> = Mutex::new(RefCell::new([[0,0,0]; 4]));
static NEOPIXEL: Mutex<RefCell<Option<Ws2812<CounterHz<TIM2>, Pin<'A', 9, Output>>>>> =
    Mutex::new(RefCell::new(None));

/*
Creates a new neopixel controller with the given neopixel and timer.
 */
    /*
    Creates a new neopixel controller with the given neopixel. If the neopixel is Some, 
    then the buffer is written to the neopixel.
     */
    pub fn new_neopixel(neopixel: Ws2812<CounterHz<TIM2>, Pin<'A', 9, Output>>, timer: CounterHz<TIM2>, pa9: Pin<'A', 9, Output>) {
        let mut neopixel = Ws2812::new(timer, pa9);
        neopixel.write([[0, 5, 0]; 4].into_iter()).unwrap();
    }

    /*
    Updates the pixel at the given index with the given color. If the neopixel is Some, 
    the buffer is written to the neopixel.
     */
    
    pub fn update_pixel(pixel: usize, color: [i32; 3]) {

        let color = RGB {
            r: color[0] as u8,
            g: color[1] as u8,
            b: color[2] as u8,
        };

        cortex_m::interrupt::free(|cs| {
            let mut neo_ref = NEOPIXEL.borrow(cs).borrow_mut();
            neo_ref
                .as_mut()
                .unwrap()
                .write([color].into_iter())
                .unwrap();
        });
    }
