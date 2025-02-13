use core::cell::RefCell;
use core::fmt::{self, Write};

use cortex_m::interrupt::{CriticalSection, Mutex};
use cortex_m::register::primask::{self, Primask};
use f4_w25q::embedded_storage::W25QSequentialStorage;
use hal::qspi::Bank1;
use sequential_storage::cache::NoCache;
use sequential_storage::{map, queue};

use stm32f4xx_hal as hal;
use storage_types::ConfigKey;

use crate::{CAPACITY, CONFIG_FLASH_RANGE, LOGS_FLASH_RANGE, RTC};

static mut LOGGER: Option<Logger> = Some(Logger { flash: Mutex::new(RefCell::new(None)) });

pub fn setup_logger<'a>(
    flash: W25QSequentialStorage<Bank1, CAPACITY>,
) -> Result<(), embedded_sdmmc::Error<hal::sdio::Error>> {
    unsafe {
        LOGGER.replace(Logger { flash: Mutex::new(RefCell::new(Some(flash))) });
    }
    Ok(())
}

pub fn get_logger() -> &'static mut Logger {
    unsafe { LOGGER.as_mut().unwrap() }
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

pub struct Logger {
    flash: Mutex<RefCell<Option<W25QSequentialStorage<Bank1, CAPACITY>>>>,
}

impl Logger {
    pub async fn log(&mut self, fmt: fmt::Arguments<'_>) {
        let mut buf = [0u8; 1024];
        let mut buf = FixedWriter(&mut buf, 0);
        write!(buf, "{}", fmt).unwrap();
        self.log_str(unsafe { core::str::from_utf8_unchecked(&buf.0[..buf.1]) })
            .await;
    }

    pub async fn log_str(&mut self, msg: &str) {
        let mut buffer = [0u8; 2048];
        let buffer = cortex_m::interrupt::free(|cs| {
            let mut rtc = RTC.borrow(cs).borrow_mut();
            let (h, m, s, millis) = rtc.as_mut().unwrap().get_datetime().as_hms_milli();
            let mut writer = FixedWriter(&mut buffer, 0);
            let _ = write!(writer, "{:02}:{:02}:{:02}.{:03} {}\n", h, m, s, millis, msg);
            &writer.0[..writer.1]
        });

        {
            let mask = primask::read();
            cortex_m::interrupt::disable();
            if let Some(flash) = self.flash.borrow(unsafe { &CriticalSection::new() }).borrow_mut().as_mut() {
                let _ =
                    queue::push(flash, LOGS_FLASH_RANGE, &mut NoCache::new(), &buffer, false).await;
                // Ignore the result, we can't do anything about it.
            }
            if mask.is_active() { unsafe { cortex_m::interrupt::enable(); }}
        };
    }


    pub async fn get_logs<'a>(&'a mut self, f: impl Fn(&'_ str)) {
        {
            let mask = primask::read();
            cortex_m::interrupt::disable();
            if let Some(flash) = self.flash.borrow(unsafe { &CriticalSection::new() }).borrow_mut().as_mut() {
                let mut no_cache = NoCache::new();
                let mut iterator = queue::iter(flash, LOGS_FLASH_RANGE, &mut no_cache).await.unwrap();
                
                let mut buffer = [0u8; 2048];
                let next = iterator.next(&mut buffer).await;
                while let Ok(Some(ref buf)) = next {
                    let s = unsafe { core::str::from_utf8_unchecked(buf) };
                    f(s);
                }
            }
            if mask.is_active() { unsafe { cortex_m::interrupt::enable(); }}
        };
    }

    pub async fn edit_config(&mut self, key: &ConfigKey, value: u64) {
        {
            let mask = primask::read();
            cortex_m::interrupt::disable();
            if let Some(flash) = self.flash.borrow(unsafe { &CriticalSection::new() }).borrow_mut().as_mut() {
                let _ = map::store_item(
                    flash,
                    CONFIG_FLASH_RANGE,
                    &mut NoCache::new(),
                    &mut [0u8; 2048],
                    key,
                    &value,
                )
                .await;
            }
            if mask.is_active() { unsafe { cortex_m::interrupt::enable(); }}
        };
    }
}
