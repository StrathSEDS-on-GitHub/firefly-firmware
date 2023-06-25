use core::cell::RefCell;
use core::fmt::{self, Write};

use cortex_m::interrupt::Mutex;
use cortex_m_semihosting::{hprint, hprintln};
use embedded_sdmmc::{Block, BlockDevice, BlockIdx, Controller, File, Mode, Volume, VolumeIdx};
use f4_w25q::w25q::W25Q;
use hal::qspi::Bank1;
use hal::rtc::Rtc;
use littlefs2::fs::{Filesystem, OpenOptions};
use littlefs2::path;
use littlefs2::path::Path;
use stm32f4xx_hal::sdio::{SdCard, Sdio};

use stm32f4xx_hal as hal;

use crate::logger::get_serial;
use crate::RTC;

static mut LOGGER: Option<Logger> = Some(Logger {
    sd_logger: None,
    flash: None,
});

pub fn setup_logger<'a>(
    sdio: Sdio<SdCard>,
    flash: Filesystem<'static, W25Q<Bank1>>,
) -> Result<(), embedded_sdmmc::Error<hal::sdio::Error>> {
    unsafe {
        LOGGER.replace(Logger {
            sd_logger: Some(SdLogger::new(sdio)?),
            flash: Some(flash),
        });
    }
    Ok(())
}

pub fn get_logger() -> &'static mut Logger<'static> {
    unsafe { LOGGER.as_mut().unwrap() }
}

struct DummyTimeSource;
impl embedded_sdmmc::TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp::from_fat(0, 0)
    }
}
pub struct SdWrapper {
    sdio: RefCell<Sdio<SdCard>>,
}

pub struct SdLogger {
    cont: Controller<SdWrapper, DummyTimeSource>,
    vol: Volume,
    file: File,
}

pub struct FixedWriter<'a>(pub &'a mut [u8], pub usize);

impl<'a> Write for FixedWriter<'a> {
    fn write_str(&mut self, s: &str) -> Result<(), core::fmt::Error> {
        for c in s.chars() {
            if self.1 >= self.0.len() {
                return Ok(());
            }
            self.0[self.1] = c as u8;
            self.1 += 1;
        }
        Ok(())
    }
}

impl BlockDevice for SdWrapper {
    type Error = hal::sdio::Error;

    fn read(
        &self,
        blocks: &mut [Block],
        address: BlockIdx,
        reason: &str,
    ) -> Result<(), Self::Error> {
        for i in 0..blocks.len() {
            let addr = address.0 + i as u32;
            //hprintln!("Reading block {}", addr);
            self.sdio
                .borrow_mut()
                .read_block(addr, &mut blocks[i].contents)?;
        }
        Ok(())
    }

    fn write(&self, buf: &[Block], address: BlockIdx) -> Result<(), Self::Error> {
        for i in 0..buf.len() {
            let addr = address.0 + i as u32;
            //hprintln!("Reading block {}", addr);
            self.sdio.borrow_mut().write_block(addr, &buf[i])?;
        }
        Ok(())
    }

    fn num_blocks(&self) -> Result<embedded_sdmmc::BlockCount, Self::Error> {
        Ok(embedded_sdmmc::BlockCount(
            self.sdio
                .borrow()
                .card()
                .map(|c| c.block_count())
                .unwrap_or(0),
        ))
    }
}

impl SdLogger {
    pub fn new(sdio: Sdio<SdCard>) -> Result<SdLogger, embedded_sdmmc::Error<hal::sdio::Error>> {
        let wrapper = SdWrapper {
            sdio: RefCell::new(sdio),
        };
        let mut cont = Controller::new(wrapper, DummyTimeSource);
        let mut vol = cont.get_volume(VolumeIdx(0))?;
        let mut dir = cont.open_root_dir(&mut vol)?;
        let file =
            cont.open_file_in_dir(&mut vol, &mut dir, "log.txt", Mode::ReadWriteCreateOrAppend)?;

        Ok(SdLogger { cont, vol, file })
    }

    pub fn log(&mut self, fmt: fmt::Arguments) {
        let mut buf = [0u8; 1024];
        let mut buf = FixedWriter(&mut buf, 0);
        write!(buf, "{}", fmt).unwrap();
        self.log_str(unsafe { core::str::from_utf8_unchecked(&buf.0[..buf.1]) })
    }

    pub fn log_str(&mut self, msg: &str) {
        loop {
            if self
                .cont
                .write(&mut self.vol, &mut self.file, msg.as_bytes())
                .and_then(|_| self.cont.write(&mut self.vol, &mut self.file, b"\n"))
                .is_ok()
            {
                break;
            }
        }
    }
}

pub struct Logger<'a> {
    sd_logger: Option<SdLogger>,
    flash: Option<Filesystem<'a, f4_w25q::w25q::W25Q<Bank1>>>,
}

impl Logger<'_> {
    pub fn log(&mut self, fmt: fmt::Arguments) {
        let mut buf = [0u8; 1024];
        let mut buf = FixedWriter(&mut buf, 0);
        write!(buf, "{}", fmt).unwrap();
        self.log_str(unsafe { core::str::from_utf8_unchecked(&buf.0[..buf.1]) })
    }

    pub fn log_str(&mut self, msg: &str) {
        let mut buf = [0u8; 1024];
        let mut buf = FixedWriter(&mut buf, 0);
        let (h, m, s, millis) = cortex_m::interrupt::free(|cs| {
            // SAFETY: Mutex makes access of static mutable variable safe
            let mut borrow = RTC.borrow(cs).borrow_mut();
            let hms = borrow.as_mut().unwrap().get_datetime().as_hms_milli();
            hms
        });
        writeln!(buf, "[{:02}:{:02}:{:02}.{:03}] {}", h, m, s, millis, msg).unwrap();
        match &mut self.sd_logger {
            Some(sd_logger) => {
                sd_logger.log_str(unsafe { core::str::from_utf8_unchecked(&buf.0[..buf.1]) })
            }
            None => {
                get_serial()
                    .write_fmt(format_args!("logs|{}\n", msg))
                    .unwrap();
            }
        }

        self.flash
            .as_mut()
            .unwrap()
            .open_file_with_options_and_then(
                |o| o.append(true).create(true),
                path!("log.txt"),
                |f| f.write(&buf.0[..buf.1]),
            )
            .ok();
    }
}
