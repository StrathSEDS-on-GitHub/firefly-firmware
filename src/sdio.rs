use core::cell::RefCell;
use core::fmt::{self, Write};

use embedded_sdmmc::{Block, BlockDevice, BlockIdx, Controller, File, Volume};
use f4_w25q::embedded_storage::W25QSequentialStorage;
use hal::qspi::Bank1;
use sequential_storage::cache::NoCache;
use sequential_storage::queue;
use stm32f4xx_hal::sdio::{SdCard, Sdio};

use stm32f4xx_hal as hal;

use crate::{CAPACITY, LOGS_FLASH_RANGE, RTC};

static mut LOGGER: Option<Logger> = Some(Logger {
    sd_logger: None,
    flash: None,
});

pub fn setup_logger<'a>(
    flash: W25QSequentialStorage<Bank1, CAPACITY>,
) -> Result<(), embedded_sdmmc::Error<hal::sdio::Error>> {
    unsafe {
        LOGGER.replace(Logger {
            sd_logger: None,
            flash: Some(flash),
        });
    }
    Ok(())
}

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
                return Err(fmt::Error);
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
        _reason: &str,
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
                .map(|c| c.block_count() as u32)
                .unwrap_or(0),
        ))
    }
}

pub struct Logger {
    sd_logger: Option<SdLogger>,
    flash: Option<W25QSequentialStorage<Bank1, CAPACITY>>,
}

impl Logger {
    pub async fn log(&mut self, fmt: fmt::Arguments<'_>) {
        let mut buf = [0u8; 1024];
        let mut buf = FixedWriter(&mut buf, 0);
        write!(buf, "{}", fmt).unwrap();
        self.log_str(unsafe { core::str::from_utf8_unchecked(&buf.0[..buf.1]) }).await;
    }

    pub async fn log_str(&mut self, msg: &str) {
        let mut buffer = [0u8; 2048];
        let buffer = cortex_m::interrupt::free(|cs| {
            let mut rtc = RTC.borrow(cs).borrow_mut();
            let (h,m,s,millis) = rtc.as_mut().unwrap().get_datetime().as_hms_milli();
            let mut writer = FixedWriter(&mut buffer, 0);
            let _ = write!(writer, "{:02}:{:02}:{:02}.{:03} {}\n", h, m, s, millis, msg);
            &writer.0[..writer.1]
        });

        if let Some(ref mut flash) = self.flash {
            let _ = queue::push(flash, LOGS_FLASH_RANGE, &mut NoCache::new(), &buffer, false).await;
            // Ignore the result, we can't do anything about it.
        }
    }
}
