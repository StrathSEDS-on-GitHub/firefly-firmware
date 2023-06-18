use embedded_hal::blocking::i2c;

const ADDR: u8 = 0x4A;

pub struct BNO085<'a, I2C: i2c::WriteRead> {
    com: &'a mut I2C,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Register {
    ChipId = 0x01,
}

#[derive(Debug)]
pub enum Error<E> {
    I2c(E),
    InvalidId,
}

impl<'a, I2C: i2c::WriteRead> BNO085<'a, I2C> {
    pub fn new<E: core::fmt::Debug>(i2c: &'a mut I2C) -> Result<Self, Error<E>>
    where
        I2C: i2c::WriteRead<Error = E>,
    {
        let chip = BNO085 { com: i2c };
        Ok(chip)
    }

    pub fn id(&mut self) -> Result<u8, I2C::Error> {
        self.read_byte(Register::ChipId)
    }

    fn read_byte(&mut self, reg: Register) -> Result<u8, I2C::Error> {
        let mut data: [u8; 1] = [0];
        match self.com.write_read(ADDR, &[reg as u8], &mut data) {
            Ok(_) => Ok(data[0]),
            Err(e) => Err(e),
        }
    }

    pub fn pressure(&mut self) -> Result<u32, I2C::Error> {
        let mut data: [u8; 3] = [0; 3];
        match self.com.write_read(ADDR, &[0x20], &mut data) {
            Ok(_) => Ok((data[0] as u32) << 16 | (data[1] as u32) << 8 | (data[2] as u32)),
            Err(e) => Err(e),
        }
    }
}
