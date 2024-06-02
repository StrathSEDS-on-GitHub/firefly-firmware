use bmp388::{
    config::{
        FifoConfig, OversamplingConfig, SubsamplingFactor
    }, 
    Blocking, 
    PowerControl, 
    PowerMode,
    Oversampling,
    BMP388
};
use heapless::Vec;
use core::{cell::RefCell, ptr::addr_of_mut};
use cortex_m::interrupt::Mutex;
use stm32f4xx_hal::{
    i2c::dma::{
        I2CMasterHandleIT, I2CMasterWriteReadDMA, I2cCompleteCallback
    }, interrupt
};
use stm32f4xx_hal::i2c;
use stm32f4xx_hal::timer::delay::SysDelay;
use crate::{futures::bmp_wake, pins::{Altimeter, I2c1Handle}};
use crate::interrupt_wake;

#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct PressureTemp {
    pub pressure: f32,
    pub temperature: f32,
}

pub type FifoFrames = Vec<PressureTemp, ALTIMETER_FRAME_COUNT>;

/*
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FifoFrames([PressureTemp; ALTIMETER_FRAME_COUNT]);

impl Default for FifoFrames {
    fn default() -> Self {
        FifoFrames([PressureTemp::default(); ALTIMETER_FRAME_COUNT])
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
*/

pub struct BMP388Wrapper {
    bmp: BMP388<I2c1Handle, Blocking>
}

impl BMP388Wrapper {
    pub const FRAME_COUNT: usize = 73;
    pub const BUF_SIZE:    usize = 512;

    pub fn new(i2c: I2c1Handle, delay: &mut SysDelay) -> Self {
        let mut bmp = BMP388::new_blocking(i2c, Self::ADDRESS, delay).unwrap();
        bmp.set_power_control(PowerControl {
            pressure_enable: true,
            temperature_enable: true,
            mode: PowerMode::Normal
        }).unwrap();
        bmp.set_oversampling(OversamplingConfig {
            osr_pressure: Oversampling::x1,
            osr_temperature: Oversampling::x1
        }).unwrap();
        bmp.set_fifo_config(FifoConfig {
            enabled: true,
            stop_on_full: false,
            store_pressure: true,
            store_temperature: true,
            return_sensor_time: false,
            subsampling: SubsamplingFactor::subsample_1,
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
    fn process_fifo_buffer(&self, data: [u8; BUF_SIZE]) -> Vec<PressureTemp, FRAMES>;
    
    fn decode_frame(pres: &[u8], temp: &[u8]) -> (u32, u32) {
        ((pres[0] as u32) | (pres[1] as u32) << 8 | (pres[2] as u32) << 16,
         (temp[0] as u32) | (temp[1] as u32) << 8 | (temp[2] as u32) << 16)
    }
}


impl AltimeterFifoDMA<
    {BMP388Wrapper::FRAME_COUNT}, 
    {BMP388Wrapper::BUF_SIZE}
> for BMP388Wrapper {
    const FIFO_READ_REG: u8 = 0x14;
    const ADDRESS: u8 = 0x77;

    fn dma_interrupt(&mut self) {
        self.bmp.com.handle_dma_interrupt();
    }

    fn process_fifo_buffer(
        &self,
        data: [u8; BMP388Wrapper::BUF_SIZE]
    ) -> Vec<PressureTemp, {BMP388Wrapper::FRAME_COUNT}> {
        let mut i: usize = 0;
        let mut output = Vec::<PressureTemp, {BMP388Wrapper::FRAME_COUNT}>::new();

        while i < BMP388Wrapper::BUF_SIZE - 7 {
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
                        let (u_pres, u_temp) = Self::decode_frame(p, t);
                        let temp = self.bmp.compensate_temp(u_temp);
                        let pres = self.bmp.compensate_pressure(u_pres, temp);
                        let frame = PressureTemp { pressure: pres as f32, temperature: temp as f32 };
                        output.push(frame).unwrap();
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

        output
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
pub const ALTIMETER_FRAME_COUNT: usize = BMP388Wrapper::FRAME_COUNT;

#[cfg(feature = "target-maxi")]
pub const ALTIMETER_FRAME_COUNT: usize = BMP581::FRAME_COUNT;

#[cfg(feature = "target-mini")]
pub const ALTIMETER_BUF_SIZE: usize = BMP388Wrapper::BUF_SIZE;

#[cfg(feature = "target-maxi")]
pub const ALTIMETER_BUF_SIZE: usize = BMP581::BUF_SIZE;

pub async fn read_altimeter_fifo(bmp: Altimeter) -> (FifoFrames, Altimeter) {
    static mut DATA: [u8; ALTIMETER_BUF_SIZE] = [0u8; ALTIMETER_BUF_SIZE];

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
                    Some(|_| { interrupt_wake!(bmp_wake); } )
                ))
            .unwrap();
        });
    }
    crate::futures::bmp_wake::future().await;
    
    let bmp = cortex_m::interrupt::free(|cs| 
        BMP.borrow(cs).replace(None).unwrap()
    );
    let frames = bmp.process_fifo_buffer(unsafe{DATA});
    (frames, bmp)
}
