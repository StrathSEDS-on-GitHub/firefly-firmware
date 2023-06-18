use embedded_hal::blocking::i2c;

const ADDR: u8 = 0x46;

pub struct BMP581<'a, E, I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E>> {
    com: &'a mut I2C,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Register {
    ChipId = 0x01,
    Status = 0x28,
}

impl<'a, E, I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E>> BMP581<'a, E, I2C> {
    pub fn new(i2c: &'a mut I2C) -> Result<Self, E> {
        i2c.write(ADDR, &[0x37, 0b0000_0001]);
        let chip = BMP581 { com: i2c };
        Ok(chip)
    }

    pub fn id(&mut self) -> Result<u8, E> {
        self.read_byte(Register::ChipId)
    }

    pub fn read_byte(&mut self, reg: Register) -> Result<u8, E> {
        let mut data: [u8; 1] = [0];
        match self.com.write_read(ADDR, &[reg as u8], &mut data) {
            Ok(_) => Ok(data[0]),
            Err(e) => Err(e),
        }
    }

    pub fn enable_pressure(&mut self) -> Result<(), E> {
        self.com.write(ADDR, &[0x36, 0x7F])
    }

    pub fn pressure(&mut self) -> Result<u32, E> {
        let mut data: [u8; 3] = [0; 3];
        match self.com.write_read(ADDR, &[0x20], &mut data) {
            Ok(_) => Ok((data[0] as u32) | (data[1] as u32) << 8 | (data[2] as u32) << 16),
            Err(e) => Err(e),
        }
    }

    pub fn temperature(&mut self) -> Result<u32, E> {
        let mut data: [u8; 3] = [0; 3];
        match self.com.write_read(ADDR, &[0x1d], &mut data) {
            Ok(_) => Ok((data[0] as u32) | (data[1] as u32) << 8 | (data[2] as u32) << 16),
            Err(e) => Err(e),
        }
    }
}
