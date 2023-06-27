use core::cell::RefCell;
use core::fmt::{self, Write};

use cortex_m::interrupt::Mutex;
use embedded_sdmmc::{Block, BlockDevice, BlockIdx, Controller, File, Volume}; 
use f4_w25q::w25q::W25Q;
use hal::qspi::Bank1;
use littlefs2::fs::Filesystem;
use littlefs2::path;
use littlefs2::path::Path;
use stm32f4xx_hal::sdio::{SdCard, Sdio};

use stm32f4xx_hal as hal;

use crate::RTC;

static mut LOGGER: Option<Logger> = Some(Logger {
    sd_logger: None,
    flash: None,
});

pub fn setup_logger<'a>(
    flash: Filesystem<'static, W25Q<Bank1>>,
) -> Result<(), embedded_sdmmc::Error<hal::sdio::Error>> {
    unsafe {
        LOGGER.replace(Logger {
            sd_logger: None,
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
                .map(|c| c.block_count())
                .unwrap_or(0),
        ))
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
        static BUF: Mutex<RefCell<[u8; 2048]>> = Mutex::new(RefCell::new([0u8; 2048]));
        static OFFSET: Mutex<RefCell<usize>> = Mutex::new(RefCell::new(0));
        cortex_m::interrupt::free(|cs| {
            let mut buf = BUF.borrow(cs).borrow_mut();
            let mut buf_writer = FixedWriter(
                buf.as_mut(),
                *OFFSET.borrow(cs).borrow(),
            );

            let mut borrow = RTC.borrow(cs).borrow_mut();
            let (h, m, s, millis) = borrow.as_mut().unwrap().get_datetime().as_hms_milli();
            match writeln!(
                buf_writer,
                "[{:02}:{:02}:{:02}.{:03}] {}",
                h, m, s, millis, msg
            ) {
                Ok(_) => {
                    *OFFSET.borrow(cs).borrow_mut() = buf_writer.1;
                }
                Err(_) => {
                    // Flush buffer
                    self.flash
                        .as_mut()
                        .unwrap()
                        .open_file_with_options_and_then(
                            |o| o.append(true).create(true),
                            path!("log.txt"),
                            |f| f.write(&buf_writer.0[..buf_writer.1]),
                        )
                        .ok();

                    *OFFSET.borrow(cs).borrow_mut() = 0;
                }
            }
        });
    }
}
