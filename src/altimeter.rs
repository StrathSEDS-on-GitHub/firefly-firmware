use bmp388::{config::{FifoConfig, OversamplingConfig}, Blocking, PowerControl, BMP388};
use heapless::Vec;
use core::{cell::RefCell, ptr::addr_of_mut, sync::atomic::AtomicBool};
use cortex_m::interrupt::Mutex;
use stm32f4xx_hal::{
    i2c::dma::{
            I2CMasterHandleIT, I2CMasterWriteReadDMA, I2cCompleteCallback
        }, interrupt
};
use stm32f4xx_hal::i2c;
use stm32f4xx_hal::timer::delay::SysDelay;
use crate::futures::YieldFuture;
use crate::pins::Altimeter;
use crate::bmp581::I2c1Handle;
use crate::bmp581::BMP581;

const BMP388_FRAME_COUNT: usize = 73;
const BMP388_BUF_SIZE:    usize = 512;
const BMP581_FRAME_COUNT: usize = 16;
const BMP581_BUF_SIZE:    usize = BMP581_FRAME_COUNT * 6;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct PressureTemp {
    pub pressure: u32,
    pub temperature: u32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FifoFrames([PressureTemp; ALTIMETER_FRAME_COUNT]);

impl Default for FifoFrames {
    fn default() -> Self {
        FifoFrames([PressureTemp::default(); BMP388_FRAME_COUNT])
    }
}

impl From<[PressureTemp; ALTIMETER_FRAME_COUNT]> for FifoFrames {
    fn from(frames: [PressureTemp; ALTIMETER_FRAME_COUNT]) -> Self {
        FifoFrames(frames)
    }
}

impl Into<[PressureTemp; ALTIMETER_FRAME_COUNT]> for FifoFrames {
    fn into(self) -> [PressureTemp; ALTIMETER_FRAME_COUNT] {
        self.0
    }
}

impl<'a> Into<&'a [PressureTemp; ALTIMETER_FRAME_COUNT]> for &'a FifoFrames {
    fn into(self) -> &'a [PressureTemp; ALTIMETER_FRAME_COUNT] {
        &self.0
    }
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
        }).unwrap();
        Self { bmp }
    }
}

pub trait AltimeterFifoDMA<
    const FRAMES: usize, const BUF_SIZE: usize
>: I2CMasterWriteReadDMA {
    const FIFO_READ_REG: u8;
    const ADDRESS: u8;
    fn dma_interrupt(&mut self);
    fn process_fifo_buffer(data: [u8; BUF_SIZE]) -> [PressureTemp; FRAMES];
}

fn frame_to_reading(pres: &[u8], temp: &[u8]) -> PressureTemp {
    PressureTemp {
        pressure:    (pres[0] as u32) | (pres[1] as u32) << 8 | (pres[2] as u32) << 16,
        temperature: (temp[0] as u32) | (temp[1] as u32) << 8 | (temp[2] as u32) << 16,
    }
}

impl AltimeterFifoDMA<BMP581_FRAME_COUNT, BMP581_BUF_SIZE> for BMP581 {
    const FIFO_READ_REG: u8 = 0x29;
    const ADDRESS: u8 = 0x46;

    fn dma_interrupt(&mut self) {
        self.com.handle_dma_interrupt();
    }

    fn process_fifo_buffer(
        data: [u8; BMP581_BUF_SIZE]
    ) -> [PressureTemp; BMP581_FRAME_COUNT] {
        data
        .chunks(6)
        .map(|x| x.split_at(3))
        .map(|(pres, temp)| frame_to_reading(pres, temp))
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

impl AltimeterFifoDMA<BMP388_FRAME_COUNT, BMP388_BUF_SIZE> for BMP388Wrapper {
    const FIFO_READ_REG: u8 = 0x14;
    const ADDRESS: u8 = 0x77;

    fn dma_interrupt(&mut self) {
        self.bmp.com.handle_dma_interrupt();
    }

    fn process_fifo_buffer(
        data: [u8; BMP388_BUF_SIZE]
    ) -> [PressureTemp; BMP388_FRAME_COUNT] {
        let mut i: usize = 0;
        let mut output = Vec::<PressureTemp, BMP388_FRAME_COUNT>::new();

        while i < BMP388_FRAME_COUNT {
            let header = data[i];
            let fh_mode = (header >> 6) & 0b11;
            let fh_param = (header >> 2) & 0b1111;
            let p = (fh_param & 1) != 0;
            let t = ((fh_param >> 2) & 1) != 0;
            let s = ((fh_param >> 3) & 1) != 0;

            // TODO: log invalid frames
            match fh_mode {
                0b01 => {
                    // Config change or error
                    i += 2;
                },
                0b10 => match (s, t, p) { // Data frame
                    (true, false, false) => {
                        // sensortime 
                        i += 4;
                    },
                    (false, true, false) => {
                        // temp only
                        i += 4;
                    },
                    (false, false, true) => {
                        // pressure only
                        i += 4;
                    },
                    (false, true, true) => {
                        // pressure & temp
                        let t = &data[i+1 .. i+4];
                        let p = &data[i+4 .. i+7];
                        output.push(frame_to_reading(p, t));
                        i += 7;
                    },
                    (false, false, false) => {
                        // Empty
                        i += 2;
                    },
                    _ => {
                        // Invalid
                        break;
                    }
                },
                _ => {
                    // Invalid
                    break;
                }
            }
        }
        output.resize(BMP388_FRAME_COUNT, PressureTemp { pressure: 0, temperature: 0 });
        output.into_array().unwrap()
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
pub const ALTIMETER_FRAME_COUNT: usize = BMP388_FRAME_COUNT;

#[cfg(feature = "target-maxi")]
pub const ALTIMETER_FRAME_COUNT: usize = BMP581_FRAME_COUNT;

#[cfg(feature = "target-mini")]
pub const ALTIMETER_BUF_SIZE: usize = BMP388_BUF_SIZE;

#[cfg(feature = "target-maxi")]
pub const ALTIMETER_BUF_SIZE: usize = BMP581_BUF_SIZE;

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

    let frames = Altimeter::process_fifo_buffer(unsafe{DATA});

    (frames, bmp)
}

