use bmp388::{BMP388, Blocking, config::FifoConfig};
use heapless::Vec;
use core::{cell::RefCell, ptr::addr_of_mut, sync::atomic::AtomicBool};
use cortex_m::{interrupt::Mutex, peripheral::NVIC};
use stm32f4xx_hal::{
    pac::I2C1,
    i2c::{
        I2c,
        dma::{
            I2CMasterWriteReadDMA,
            I2cCompleteCallback,
            I2CMasterHandleIT
        }
    },
    interrupt,
};
use stm32f4xx_hal::i2c;
use stm32f4xx_hal::timer::delay::SysDelay;
use crate::futures::YieldFuture;
use crate::pins::Altimeter;
use crate::bmp581::I2c1Handle;
use crate::bmp581::BMP581;

const BMP388_FRAME_COUNT: usize = 16;
const BMP388_BUF_SIZE:    usize = 512;
const BMP581_FRAME_COUNT: usize = 16;
const BMP581_BUF_SIZE:    usize = BMP581_FRAME_COUNT * 6;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct PressureTemp {
    pub pressure: u32,
    pub temperature: u32,
}

pub struct BMP388Wrapper {
    pub bmp: BMP388<I2c1Handle, Blocking>
}

impl BMP388Wrapper {
    pub fn new(i2c: I2c1Handle, delay: &mut SysDelay) -> Self {
        let mut bmp = BMP388::new_blocking(i2c, Self::ADDRESS, delay).unwrap();
        bmp.set_fifo_config(FifoConfig {
            enabled: true,
            stop_on_full: false,
            store_pressure: true,
            store_temperature: true,
            return_sensor_time: false,
            subsampling: 0,
            filter_data: false
        });
        Self { bmp: bmp }
    }
}

pub trait AltimeterDMA<
    const FRAMES: usize, const BUF_SIZE: usize
>: I2CMasterWriteReadDMA {
    const FIFO_READ_REG: u8;
    const ADDRESS: u8;
    fn dma_interrupt(&mut self);
    fn process_buffer(data: [u8; BUF_SIZE]) -> [PressureTemp; FRAMES];
}

impl AltimeterDMA<BMP581_FRAME_COUNT, BMP581_BUF_SIZE> for BMP581 {
    const FIFO_READ_REG: u8 = 0x29;
    const ADDRESS: u8 = 0x46;

    fn dma_interrupt(&mut self) {
        self.com.handle_dma_interrupt();
    }

    fn process_buffer(
        data: [u8; BMP581_BUF_SIZE]
    ) -> [PressureTemp; BMP581_FRAME_COUNT] {
        data
        .chunks(6)
        .map(|x| x.split_at(3))
        .map(|(pres, temp)| PressureTemp {
            pressure: (pres[0] as u32) | (pres[1] as u32) << 8 | (pres[2] as u32) << 16,
            temperature: (temp[0] as u32) | (temp[1] as u32) << 8 | (temp[2] as u32) << 16,
        })
        .collect::<Vec<_, 16>>()
        .into_array()
        .unwrap()
    }
}

impl I2CMasterWriteReadDMA for BMP581 {
    unsafe fn write_read_dma(
        &mut self,
        addr: u8,
        bytes: &[u8],
        buf: &mut [u8],
        callback: Option<I2cCompleteCallback>
    ) -> Result<(), nb::Error<i2c::Error>> {
        self.com.write_read_dma(addr, bytes, buf, callback)
    }
}

impl AltimeterDMA<BMP388_FRAME_COUNT, BMP388_BUF_SIZE> for BMP388Wrapper {
    const FIFO_READ_REG: u8 = 0x14;
    const ADDRESS: u8 = 0x77;

    fn dma_interrupt(&mut self) {
        self.bmp.com.handle_dma_interrupt();
    }

    fn process_buffer(
        data: [u8; BMP388_BUF_SIZE]
    ) -> [PressureTemp; BMP388_FRAME_COUNT] {
         
        panic!()
    }
}

impl I2CMasterWriteReadDMA for BMP388Wrapper {
    unsafe fn write_read_dma(
        &mut self,
        addr: u8,
        bytes: &[u8],
        buf: &mut [u8],
        callback: Option<I2cCompleteCallback>
    ) -> Result<(), nb::Error<i2c::Error>> {
        self.bmp.com.write_read_dma(addr, bytes, buf, callback)
    }
}

static BMP: Mutex<RefCell<Option<Altimeter>>> = Mutex::new(RefCell::new(None));

#[interrupt]
fn DMA1_STREAM1() {
    cortex_m::interrupt::free(|cs| {
        let mut i2c = BMP.borrow(cs).borrow_mut();
        i2c.as_mut().unwrap().dma_interrupt();
    });
}

#[interrupt]
fn DMA1_STREAM0() {
    cortex_m::interrupt::free(|cs| {
        let mut i2c = BMP.borrow(cs).borrow_mut();
        i2c.as_mut().unwrap().dma_interrupt();
    });
}

#[cfg(feature = "target-mini")]
const ALTIMETER_FRAME_COUNT: usize = BMP388_FRAME_COUNT;

#[cfg(feature = "target-maxi")]
const ALTIMETER_FRAME_COUNT: usize = BMP581_FRAME_COUNT;

#[cfg(feature = "target-mini")]
const ALTIMETER_BUF_SIZE: usize = BMP388_BUF_SIZE;

#[cfg(feature = "target-maxi")]
const ALTIMETER_BUF_SIZE: usize = BMP581_BUF_SIZE;

pub async fn read_altimeter_fifo(
    bmp: Altimeter
) -> ([PressureTemp; ALTIMETER_FRAME_COUNT], Altimeter) {
    static mut DATA: [u8; ALTIMETER_BUF_SIZE] = [0u8; ALTIMETER_BUF_SIZE];
    static DONE: AtomicBool = AtomicBool::new(false);
    unsafe {
        cortex_m::interrupt::free(|cs| {
            BMP.borrow(cs).replace(Some(bmp));
            nb::block!(BMP
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .write_read_dma(
                    Altimeter::ADDRESS,
                    &[Altimeter::FIFO_READ_REG],
                    &mut *addr_of_mut!(DATA),
                    Some(|_| {
                        DONE.store(true, core::sync::atomic::Ordering::Release);
                    }),
                ))
            .unwrap();
        });
    }

    while !DONE.load(core::sync::atomic::Ordering::Acquire) {
        YieldFuture::new().await;
    }

    DONE.store(false, core::sync::atomic::Ordering::Release);

    let bmp = cortex_m::interrupt::free(|cs| BMP.borrow(cs).replace(None).unwrap());

    let frames = Altimeter::process_buffer(unsafe{DATA});

    (frames, bmp)
}

