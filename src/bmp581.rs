use core::{cell::RefCell, ptr::addr_of_mut, sync::atomic::AtomicBool};

use cortex_m::{interrupt::Mutex, peripheral::NVIC};
use heapless::Vec;
use stm32f4xx_hal::{
    dma::{Stream0, Stream1},
    i2c::{
        dma::{I2CMasterDma, I2CMasterHandleIT, I2CMasterWriteReadDMA, RxDMA, TxDMA},
        Error,
    },
    interrupt,
    pac::{DMA1, I2C1},
};

use crate::futures::YieldFuture;
use crate::altimeter::PressureTemp;

pub type I2c1Handle =
    I2CMasterDma<I2C1, TxDMA<I2C1, Stream1<DMA1>, 0>, RxDMA<I2C1, Stream0<DMA1>, 1>>;

const ADDR: u8 = 0x46;

pub struct BMP581 {
    pub com: I2c1Handle,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Register {
    ChipId = 0x01,
    Status = 0x28,
}

impl BMP581 {
    pub const FRAME_COUNT: usize = 16;
    pub const BUF_SIZE:    usize = Self::FRAME_COUNT * 6;

    pub fn new(mut i2c: I2c1Handle) -> Result<Self, Error> {
        nb::block!(i2c.write(ADDR, &[0x37, 0b0000_0001]))?;
        let chip = BMP581 { com: i2c };
        Ok(chip)
    }

    pub fn id(&mut self) -> Result<u8, Error> {
        self.read_byte(Register::ChipId)
    }

    pub fn read_byte(&mut self, reg: Register) -> Result<u8, Error> {
        let mut data: [u8; 1] = [0];
        match nb::block!(self.com.write_read(ADDR, &[reg as u8], &mut data)) {
            Ok(_) => Ok(data[0]),
            Err(e) => Err(e),
        }
    }

    pub fn enable_pressure_temperature(&mut self) -> Result<(), Error> {
        // First enter continuous mode
        // Then set over sampling rate.
        nb::block!(self.com.write(ADDR, &[0x37, 0x03]))?;
        nb::block!(self.com.write(ADDR, &[0x36, 0b01_101_101]))
    }

    pub fn pressure(&mut self) -> Result<u32, Error> {
        let mut data: [u8; 3] = [0; 3];
        match nb::block!(self.com.write_read(ADDR, &[0x20], &mut data)) {
            Ok(_) => Ok((data[0] as u32) | (data[1] as u32) << 8 | (data[2] as u32) << 16),
            Err(e) => Err(e),
        }
    }

    pub fn temperature(&mut self) -> Result<u32, Error> {
        let mut data: [u8; 3] = [0; 3];
        match nb::block!(self.com.write_read(ADDR, &[0x1d], &mut data)) {
            Ok(_) => Ok((data[0] as u32) | (data[1] as u32) << 8 | (data[2] as u32) << 16),
            Err(e) => Err(e),
        }
    }

    pub fn setup_fifo(&mut self) -> Result<(), Error> {
        unsafe { NVIC::unmask(interrupt::EXTI0) };
        nb::block!(self.com.write(ADDR, &[0x18, 0b_11]))?; // Fifo sel
        nb::block!(self.com.write(ADDR, &[0x16, 0]))?; // Streaming mode
        nb::block!(self.com.write(ADDR, &[0x14, 1 << 3]))?; // int enable
        nb::block!(self.com.write(ADDR, &[0x15, 1 << 1])) // int source fifo full
    }

    fn handle_dma_interrupt(&mut self) {
        self.com.handle_dma_interrupt();
    }

    pub fn read_fifo(&mut self) -> Result<[PressureTemp; 16], Error> {
        let mut data = [0u8; 16 * 6];
        let mut frames = [PressureTemp {
            pressure: 0,
            temperature: 0,
        }; 16];

        nb::block!(self.com.write_read(ADDR, &[0x29], &mut data)).unwrap();

        for (i, frame) in data
            .chunks(6)
            .map(|x| x.split_at(3))
            .map(|(temp, pres)| PressureTemp {
                pressure: (pres[0] as u32) | (pres[1] as u32) << 8 | (pres[2] as u32) << 16,
                temperature: (temp[0] as u32) | (temp[1] as u32) << 8 | (temp[2] as u32) << 16,
            })
            .enumerate()
        {
            frames[i] = frame;
        }

        Ok(frames)
    }
}

