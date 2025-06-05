
use cortex_m::peripheral::NVIC;
use embedded_hal::i2c::I2c;
use heapless::Vec;
use stm32f4xx_hal::{
    i2c::dma::{
            I2CMasterHandleIT, 
            I2CMasterWriteReadDMA, 
            I2cCompleteCallback
        },
    i2c,
    interrupt,
};

use crate::{altimeter::{AltimeterFifoDMA, PressureTemp}, pins::i2c::I2c1Proxy};

const ADDR: u8 = 0x46;

pub struct BMP581<I2C> {
    com: I2C
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Register {
    ChipId = 0x01,
    Status = 0x28,
}

impl BMP581<()> {
    pub const FRAME_COUNT: usize = 16;
    pub const BUF_SIZE:    usize = Self::FRAME_COUNT * 6;
}

impl BMP581<I2c1Proxy> {
    pub(crate) unsafe fn dma_complete(&self) -> Result<(), ()> {
        unsafe { self.com.dma_complete() }
    }
}

impl<I2C: I2c + I2CMasterHandleIT> BMP581<I2C>  {
    pub fn new(mut i2c: I2C) -> Result<Self, I2C::Error> {
        i2c.write(ADDR, &[0x37, 0b0000_0001])?;
        let chip = BMP581 { com: i2c };
        Ok(chip)
    }

    pub fn id(&mut self) -> Result<u8, I2C::Error> {
        self.read_byte(Register::ChipId)
    }

    pub fn read_byte(&mut self, reg: Register) -> Result<u8, I2C::Error> {
        let mut data: [u8; 1] = [0];
        match self.com.write_read(ADDR, &[reg as u8], &mut data) {
            Ok(_) => Ok(data[0]),
            Err(e) => Err(e),
        }
    }

    pub fn enable_pressure_temperature(&mut self) -> Result<(), I2C::Error> {
        // First enter continuous mode
        // Then set over sampling rate.
        self.com.write(ADDR, &[0x37, 0x03])?;
        self.com.write(ADDR, &[0x36, 0b01_101_101])
    }

    pub fn pressure(&mut self) -> Result<u32, I2C::Error> {
        let mut data: [u8; 3] = [0; 3];
        match self.com.write_read(ADDR, &[0x20], &mut data) {
            Ok(_) => Ok((data[0] as u32) | (data[1] as u32) << 8 | (data[2] as u32) << 16),
            Err(e) => Err(e),
        }
    }

    pub fn temperature(&mut self) -> Result<u32, I2C::Error> {
        let mut data: [u8; 3] = [0; 3];
        match self.com.write_read(ADDR, &[0x1d], &mut data) {
            Ok(_) => Ok((data[0] as u32) | (data[1] as u32) << 8 | (data[2] as u32) << 16),
            Err(e) => Err(e),
        }
    }

    pub fn setup_fifo(&mut self) -> Result<(), I2C::Error> {
        unsafe { NVIC::unmask(interrupt::EXTI0) };
        self.com.write(ADDR, &[0x18, 0b_11])?; // Fifo sel
        self.com.write(ADDR, &[0x16, 0])?; // Streaming mode
        self.com.write(ADDR, &[0x14, 1 << 3])?; // int enable
        self.com.write(ADDR, &[0x15, 1 << 1]) // int source fifo full
    }

    fn handle_dma_interrupt(&mut self) {
        self.com.handle_dma_interrupt();
    }

    pub fn read_fifo(&mut self) -> Result<[PressureTemp; 16], I2C::Error> {
        let mut data = [0u8; 16 * 6];
        let mut frames = [PressureTemp {
            pressure: 0.0,
            temperature: 0.0,
        }; 16];

        (self.com.write_read(ADDR, &[0x29], &mut data)).unwrap();

        for (i, frame) in data
            .chunks(6)
            .map(|x| x.split_at(3))
            .map(|(temp, pres)| PressureTemp {
                pressure: ((pres[0] as u32) | (pres[1] as u32) << 8 | (pres[2] as u32) << 16) as f32,
                temperature: ((temp[0] as u32) | (temp[1] as u32) << 8 | (temp[2] as u32) << 16) as f32,
            })
            .enumerate()
        {
            frames[i] = frame;
        }

        Ok(frames)
    }
}

impl<I2C: I2c + I2CMasterHandleIT + I2CMasterWriteReadDMA> AltimeterFifoDMA<{BMP581::FRAME_COUNT}, {BMP581::BUF_SIZE}> for BMP581<I2C> {
    const FIFO_READ_REG: u8 = 0x29;
    const ADDRESS: u8 = 0x46;

    fn dma_interrupt(&mut self) {
        self.com.handle_dma_interrupt();
    }

    fn process_fifo_buffer(
        &self,
        data: [u8; BMP581::BUF_SIZE]
    ) -> Vec<PressureTemp, {BMP581::FRAME_COUNT}> {
        data
        .chunks(6)
        .map(|x| x.split_at(3))
        .map(|(temp, pres)| {
            let (u_pres, u_temp) = Self::decode_frame(pres, temp);
            let pres = u_pres as f32 / libm::powf(2.0, 6.0);
            let temp = u_temp as f32 / libm::powf(2.0, 16.0);
            PressureTemp {
                temperature: temp,
                pressure: pres
            }
        })
        .collect::<Vec<_, 16>>()
    }
}

impl<I2C: I2c + I2CMasterWriteReadDMA> I2CMasterWriteReadDMA for BMP581<I2C> {
    unsafe fn write_read_dma(
        &mut self,
        addr: u8,
        bytes: &[u8],
        buf: &mut [u8],
        callback: Option<I2cCompleteCallback>
    ) -> Result<(), nb::Error<i2c::Error>> {
        unsafe { self.com.write_read_dma(addr, bytes, buf, callback) }
    }
}

