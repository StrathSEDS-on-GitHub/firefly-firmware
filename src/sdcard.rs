use core::cell::RefCell;
use core::fmt::{self, Write};

use cortex_m::interrupt::Mutex;
use cortex_m_semihosting::{hprint, hprintln};
use embedded_sdmmc::{Block, BlockDevice, BlockIdx, Controller, File, Mode, Volume, VolumeIdx};
use hal::rtc::Rtc;
use stm32f4xx_hal::sdio::{SdCard, Sdio};

use stm32f4xx_hal as hal;

static mut LOGGER: Option<Logger> = Some(Logger { sd_logger: None });

pub fn get_logger() -> &'static mut Logger {
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
            self.sdio
                .borrow_mut()
                .read_block(addr, &mut blocks[i].contents)?;
        }
        Ok(())
    }

    fn write(&self, buf: &[Block], address: BlockIdx) -> Result<(), Self::Error> {
        for i in 0..buf.len() {
            let addr = address.0 + i as u32;
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
    pub fn new(sdio: Sdio<SdCard>) -> Result<(), embedded_sdmmc::Error<hal::sdio::Error>> {
        let wrapper = SdWrapper {
            sdio: RefCell::new(sdio),
        };
        let mut cont = Controller::new(wrapper, DummyTimeSource);
        let mut vol = cont.get_volume(VolumeIdx(0))?;
        let mut dir = cont.open_root_dir(&mut vol)?;
        let file =
            cont.open_file_in_dir(&mut vol, &mut dir, "log.txt", Mode::ReadWriteCreateOrAppend)?;

        let sd_logger = Some(SdLogger { cont, vol, file });
        unsafe {
            LOGGER.replace(Logger { sd_logger });
        };
        Ok(())
    }

    pub fn log(&mut self, fmt: fmt::Arguments) {
        let mut buf = [0u8; 1024];
        let mut buf = FixedWriter(&mut buf, 0);
        write!(buf, "{}", fmt).unwrap();
        self.log_str(unsafe { core::str::from_utf8_unchecked(&buf.0[..buf.1]) })
    }

    pub fn log_str(&mut self, msg: &str) {
        let mut buf = [0u8; 2048];
        let mut buf = FixedWriter(&mut buf, 0);

        let ((h, m, s), millis) = cortex_m::interrupt::free(|cs| ((1, 2, 3), 4));
        
        // Can't do much about a failed write, so just ignore it
        let _ = self.cont
            .write(&mut self.vol, &mut self.file, &buf.0[..buf.1])
            .and_then(|_| {
                self.cont
                    .write(&mut self.vol, &mut self.file, msg.as_bytes())
                    .and_then(|_| self.cont.write(&mut self.vol, &mut self.file, b"\n"))
            });
    }
}

pub struct Logger {
    sd_logger: Option<SdLogger>,
}

impl Logger {
    pub fn log(&mut self, fmt: fmt::Arguments) {
        match &mut self.sd_logger {
            Some(sd_logger) => sd_logger.log(fmt),
            None => {}
        }
    }

    pub fn log_str(&mut self, msg: &str) {
        match &mut self.sd_logger {
            Some(sd_logger) => sd_logger.log_str(msg),
            None => {}
        }
    }
}
