use bmp388::{config::*, *};
use heapless::Vec;
use stm32f4xx_hal::{
    i2c::{self, dma::I2cCompleteCallback},
    timer::SysDelay,
};

use crate::bmp388::i2c::dma::I2CMasterWriteReadDMA;
use crate::{
    altimeter::{AltimeterFifoDMA, PressureTemp},
    pins::i2c::I2c1Proxy,
};

pub struct BMP388Wrapper {
    bmp: BMP388<I2c1Proxy, Blocking>,
}

impl BMP388Wrapper {
    pub const FRAME_COUNT: usize = 73;
    pub const BUF_SIZE: usize = 512;

    pub(crate) unsafe fn dma_complete(&self) -> Result<(), ()> {
        unsafe { self.bmp.com.dma_complete() }
    }

    pub fn new(i2c: I2c1Proxy, delay: &mut SysDelay) -> Self {
        let mut bmp = BMP388::new_blocking(i2c, Self::ADDRESS, delay).unwrap();
        bmp.set_power_control(PowerControl {
            pressure_enable: true,
            temperature_enable: true,
            mode: PowerMode::Normal,
        })
        .unwrap();
        bmp.set_oversampling(OversamplingConfig {
            osr_pressure: Oversampling::x1,
            osr_temperature: Oversampling::x1,
        })
        .unwrap();
        bmp.set_fifo_config(FifoConfig {
            enabled: true,
            stop_on_full: false,
            store_pressure: true,
            store_temperature: true,
            return_sensor_time: false,
            subsampling: SubsamplingFactor::Subsample1,
            filter_data: false,
        })
        .unwrap();
        Self { bmp }
    }
}

impl AltimeterFifoDMA<{ BMP388Wrapper::FRAME_COUNT }, { BMP388Wrapper::BUF_SIZE }>
    for BMP388Wrapper
{
    const FIFO_READ_REG: u8 = 0x14;
    const ADDRESS: u8 = 0x77;

    fn dma_interrupt(&mut self) {
        i2c::dma::I2CMasterHandleIT::handle_dma_interrupt(&mut self.bmp.com);
    }

    fn process_fifo_buffer(
        &self,
        data: [u8; BMP388Wrapper::BUF_SIZE],
    ) -> Vec<PressureTemp, { BMP388Wrapper::FRAME_COUNT }> {
        let mut i: usize = 0;
        let mut output = Vec::<PressureTemp, { BMP388Wrapper::FRAME_COUNT }>::new();

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
                }
                0b10 => match (s, t, p) {
                    // Data frame
                    (true, false, false) => {
                        // sensortime
                        i += 4;
                    }
                    (false, true, false) => {
                        // temp only
                        i += 4;
                    }
                    (false, false, true) => {
                        // pressure only
                        i += 4;
                    }
                    (false, true, true) => {
                        // pressure & temp
                        let t = &data[i + 1..i + 4];
                        let p = &data[i + 4..i + 7];
                        let (u_pres, u_temp) = Self::decode_frame(p, t);
                        let temp = self.bmp.compensate_temp(u_temp);
                        let pres = self.bmp.compensate_pressure(u_pres, temp);
                        let frame = PressureTemp {
                            pressure: pres as f32,
                            temperature: temp as f32,
                        };
                        output.push(frame).unwrap();
                        i += 7;
                    }
                    (false, false, false) => {
                        // Empty
                        i += 2;
                    }
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

    fn decode_frame(pres: &[u8], temp: &[u8]) -> (u32, u32) {
        (
            (pres[0] as u32) | (pres[1] as u32) << 8 | (pres[2] as u32) << 16,
            (temp[0] as u32) | (temp[1] as u32) << 8 | (temp[2] as u32) << 16,
        )
    }
}

impl I2CMasterWriteReadDMA for BMP388Wrapper {
    unsafe fn write_read_dma(
        &mut self,
        addr: u8,
        bytes: &[u8],
        buf: &mut [u8],
        callback: Option<I2cCompleteCallback>,
    ) -> Result<(), nb::Error<i2c::Error>> {
        unsafe { self.bmp.com.write_read_dma(addr, bytes, buf, callback) }
    }
}
