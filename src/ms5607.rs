use embedded_hal::i2c::I2c;
use embedded_hal_async::delay::DelayNs;

use crate::altimeter::PressureTemp;

type CalibParams = (u16, u16, u16, u16, u16, u16);
pub struct Uncalibrated;
pub struct Calibrated(CalibParams);

mod sealed {
    pub trait CalibrationState {}
    impl CalibrationState for super::Uncalibrated {}
    impl CalibrationState for super::Calibrated {}
}

pub trait CalibrationState: sealed::CalibrationState {}
impl<T: sealed::CalibrationState> CalibrationState for T {}

#[derive(Copy, Clone, Debug)]
pub enum Oversampling {
    OSR4096,
    OSR2048,
    OSR1024,
    OSR512,
    OSR256,
}

impl Oversampling {
    pub fn timeout_us(&self) -> u32 {
        match self {
            Oversampling::OSR4096 => 9040,
            Oversampling::OSR2048 => 4540,
            Oversampling::OSR1024 => 2280,
            Oversampling::OSR512 => 1170,
            Oversampling::OSR256 => 600,
        }
    }

    pub fn bits(&self) -> u8 {
        match self {
            Oversampling::OSR4096 => 0b100,
            Oversampling::OSR2048 => 0b011,
            Oversampling::OSR1024 => 0b010,
            Oversampling::OSR512 => 0b001,
            Oversampling::OSR256 => 0b000,
        }
    }
}

pub struct MS5607<I2C, DELAY, CAL: CalibrationState> {
    com: I2C,
    delay: DELAY,
    addr: u8,
    calibration: CAL,
}

impl<I2C,DELAY,CAL> DelayNs for MS5607<I2C, DELAY, CAL>
where
    DELAY: DelayNs,
    I2C: I2c,
    CAL: CalibrationState,
{
    async fn delay_ns(&mut self, ns: u32) {
        self.delay.delay_ns(ns).await;
    }
    async fn delay_us(&mut self, us: u32) {
        self.delay.delay_us(us).await;
    }
    async fn delay_ms(&mut self, ms: u32) {
        self.delay.delay_ms(ms).await;
    }
}

impl<I2C: I2c, DELAY: DelayNs> MS5607<I2C, DELAY, Uncalibrated> {
    pub async fn new(i2c: I2C, addr: u8, delay: DELAY) -> Result<Self, I2C::Error> {
        let mut it = Self {
            com: i2c,
            addr,
            delay,
            calibration: Uncalibrated,
        };
        it.reset().await?;
        Ok(it)
    }
}
impl<I2C: I2c, DELAY: DelayNs, CAL: CalibrationState> MS5607<I2C, DELAY, CAL> {
    pub async fn reset(&mut self) -> Result<(), I2C::Error> {
        self.com.write(self.addr, &[0x1E])?;
        self.delay.delay_ms(20).await;
        Ok(())
    }

    pub async fn prom_read(&mut self, reg: u8) -> Result<u16, I2C::Error> {
        let mut data: [u8; 2] = [0; 2];
        let cmd = 0b1010_0000 | (reg << 1);
        self.com.write(self.addr, &[cmd])?;
        self.com.read(self.addr, &mut data)?;
        Ok(u16::from_be_bytes(data))
    }

    pub async fn calibrate(mut self) -> Result<MS5607<I2C, DELAY, Calibrated>, I2C::Error> {
        let c1 = self.prom_read(1).await?;
        let c2 = self.prom_read(2).await?;
        let c3 = self.prom_read(3).await?;
        let c4 = self.prom_read(4).await?;
        let c5 = self.prom_read(5).await?;
        let c6 = self.prom_read(6).await?;

        Ok(MS5607 {
            com: self.com,
            delay: self.delay,
            addr: self.addr,
            calibration: Calibrated((c1, c2, c3, c4, c5, c6)),
        })
    }
}

impl<I2C: I2c, DELAY: DelayNs> MS5607<I2C, DELAY, Calibrated> {
    pub fn calibration_params(&self) -> &CalibParams {
        let Calibrated(ref params) = self.calibration;
        params
    }

    pub async fn read_adc(&mut self) -> Result<u32, I2C::Error> {
        let mut data: [u8; 3] = [0; 3];
        self.com.write(self.addr, &[0x00])?;
        self.delay.delay_ms(1).await;
        self.com.read(self.addr, &mut data)?;
        let reading = u32::from_be_bytes([0, data[0], data[1], data[2]]);

        Ok(reading)
    }

    #[allow(unused_variables, non_snake_case)]
    pub fn compensate(&self, d1: u32, d2: u32) -> PressureTemp {
        let &(c1, c2, c3, c4, c5, c6) = self.calibration_params();

        let dT = d2 as i64 - ((c5 as i64) << 8);
        let mut temp = 2000 + dT * (c6 as i64) / 2i64.pow(23);

        let mut off = ((c2 as i64) << 17) + ((c4 as i64) * dT) / 2i64.pow(6);
        let mut sens = ((c1 as i64) << 16) + ((c3 as i64) * dT) / 2i64.pow(7);

        if temp < 2000 {
            // Second order compensation
            let t2 = dT * dT / 2i64.pow(31);
            let mut off2 = 61 * (temp - 2000) * (temp - 2000) / 16;
            let mut sens2 = 2 * (temp - 2000) * (temp - 2000);

            if temp < -1500 {
                off2 += 15 * (temp + 1500) * (temp + 1500);
                sens2 += 8 * (temp + 1500) * (temp + 1500);
            }

            temp -= t2;
            off -= off2;
            sens -= sens2;
        }

        let pres = ((d1 as i64) * sens / 2i64.pow(21) - off) / 2i64.pow(15);

        PressureTemp {
            pressure: pres as f32,
            temperature: temp as f32 / 100.0,
        }
    }

    pub async fn read(&mut self, osr: Oversampling) -> Result<PressureTemp, I2C::Error> {
        self.com
            .write(self.addr, &[0b0100_0000 | (osr.bits() << 1)])?;
        self.delay.delay_us(osr.timeout_us()).await;
        let d1 = self.read_adc().await?;

        self.com
            .write(self.addr, &[0b0101_0000 | (osr.bits() << 1)])?;
        self.delay.delay_us(osr.timeout_us()).await;
        let d2 = self.read_adc().await?;

        Ok(self.compensate(d1, d2))
    }
}
