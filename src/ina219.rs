use embedded_hal::blocking::i2c;

const ADDR: u8 = 0x42;

pub struct INA219<'a, E, I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E>> {
    com: &'a mut I2C,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Register {
    Config = 0x00,
    ShuntVoltage = 0x01,
    BusVoltage = 0x02,
    Power = 0x03,
    Current = 0x04,
    Calibration = 0x05,
}

impl<'a, E, I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E>> INA219<'a, E, I2C> {
    pub fn new(i2c: &'a mut I2C) -> Result<Self, E> {
        let chip = INA219 { com: i2c };
        Ok(chip)
    }

    pub fn calibrate(&mut self, value: u16) -> Result<(), E> {
        self.com.write(
            ADDR,
            &[Register::Calibration as u8, (value >> 8) as u8, value as u8],
        )?;
        Ok(())
    }

    pub fn shunt_voltage(&mut self) -> Result<i16, E> {
        let value = self.read(Register::ShuntVoltage)?;
        Ok(value as i16)
    }

    pub fn voltage(&mut self) -> Result<u16, E> {
        let value = self.read(Register::BusVoltage)?;
        Ok((value >> 3) * 4)
    }

    pub fn power(&mut self) -> Result<i16, E> {
        let value = self.read(Register::Power)?;
        Ok(value as i16)
    }

    pub fn current(&mut self) -> Result<i16, E> {
        let value = self.read(Register::Current)?;
        Ok(value as i16)
    }

    pub fn read_config_reg(&mut self) -> Result<u16, E> {
        self.read(Register::Config)
    }

    fn read(&mut self, register: Register) -> Result<u16, E> {
        let mut buf: [u8; 2] = [0x00; 2];
        self.com.write_read(ADDR, &[register as u8], &mut buf)?;
        Ok(u16::from_be_bytes(buf))
    }
}
