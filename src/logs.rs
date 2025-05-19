use core::cell::RefCell;
use core::fmt::{self, Write};

use cortex_m::interrupt::{CriticalSection, Mutex};
use cortex_m::register::primask;
use embassy_futures::select::select;
use f4_w25q::embedded_storage::W25QSequentialStorage;
use hal::qspi::Bank1;
use sequential_storage::cache::NoCache;
use sequential_storage::{map, queue};

use stm32f4xx_hal as hal;
use storage_types::{CONFIG_KEYS, ConfigKey};

use crate::futures::YieldFuture;
use crate::{CAPACITY, CONFIG_FLASH_RANGE, LOGS_FLASH_RANGE, RTC, neopixel};

static LOGGER: Logger = Logger {
    flash: Mutex::new(RefCell::new(None)),
};

/// SAFETY: This function must be called only once, and prior to any call
///         to `get_logger()`.
pub fn setup_logger<'a>(
    flash: W25QSequentialStorage<Bank1, CAPACITY>,
) -> Result<(), embedded_sdmmc::Error<hal::sdio::Error>> {
    cortex_m::interrupt::free(|cs| {
        let mut flash_ref = LOGGER.flash.borrow(cs).borrow_mut();
        flash_ref.replace(flash);
    });
    Ok(())
}

pub fn get_logger() -> &'static Logger {
    &LOGGER
}

pub struct FixedWriter<'a>(&'a mut [u8], usize);

impl<'a> FixedWriter<'a> {
    pub fn new(buffer: &'a mut [u8]) -> Self {
        FixedWriter(buffer, 0)
    }

    pub fn data(&self) -> &[u8] {
        &self.0[..self.1]
    }

    pub fn copy_from_slice(&mut self, other: &[u8]) {
        let len = core::cmp::min(self.0.len() - self.1, other.len());
        self.0[self.1..self.1 + len].copy_from_slice(&other[..len]);
        self.1 = len;
    }
}

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
    /// Best-effort log. May silently discard the log if another task is
    /// using the flash.
    pub async fn log(&self, fmt: fmt::Arguments<'_>) {
        let mut buf = [0u8; 2048];
        let mut writer = FixedWriter(&mut buf, 0);

        cortex_m::interrupt::free(|cs| {
            let mut rtc = RTC.borrow(cs).borrow_mut();
            let (h, m, s, millis) = rtc.as_mut().unwrap().get_datetime().as_hms_milli();
            let _ = write!(writer, "{:02}:{:02}:{:02}.{:03} ", h, m, s, millis);
        });

        write!(writer, "{}\n", fmt).unwrap();

        self.log_impl(writer.data()).await;
    }

    /// For important logs that shouldn't be discarded, will
    /// retry obtaining the flash if it is busy.
    pub async fn retry_log(&self, fmt: fmt::Arguments<'_>) {
        let mut buf = [0u8; 2048];
        let mut writer = FixedWriter(&mut buf, 0);

        cortex_m::interrupt::free(|cs| {
            let mut rtc = RTC.borrow(cs).borrow_mut();
            let (h, m, s, millis) = rtc.as_mut().unwrap().get_datetime().as_hms_milli();
            let _ = write!(writer, "{:02}:{:02}:{:02}.{:03} [HIGH] ", h, m, s, millis);
        });

        write!(writer, "{}\n", fmt).unwrap();

        self.retry_log_impl(writer.data()).await;
    }

    async fn retry_log_impl(&self, buffer: &[u8]) {
        loop {
            let mask = primask::read();
            cortex_m::interrupt::disable();
            let cs = unsafe { CriticalSection::new() };
            let rc = self.flash.borrow(&cs);
            let Ok(mut flash_ref) = rc.try_borrow_mut() else {
                // Another task is using the flash, we can yield and try again.
                drop(cs);
                if mask.is_active() {
                    unsafe {
                        cortex_m::interrupt::enable();
                    }
                }
                YieldFuture::new().await;
                continue;
            };

            let Some(flash) = flash_ref.as_mut() else {
                // Can't do anything if we don't even have the flash
                drop(flash_ref);
                drop(cs);
                if mask.is_active() {
                    unsafe {
                        cortex_m::interrupt::enable();
                    }
                }
                return;
            };

            let _ = queue::push(flash, LOGS_FLASH_RANGE, &mut NoCache::new(), &buffer, false).await;
            // Still can't do anything if there's no space though
            drop(flash_ref);
            drop(cs);
            if mask.is_active() {
                unsafe {
                    cortex_m::interrupt::enable();
                }
            }
            return;
        }
    }

    async fn log_impl(&self, buffer: &[u8]) {
        let mask = primask::read();
        cortex_m::interrupt::disable();
        let cs = unsafe { CriticalSection::new() };
        if let Some(mut flash) = self.flash.borrow(&cs).try_borrow_mut().ok() {
            if let Some(flash) = flash.as_mut() {
                let _ =
                    queue::push(flash, LOGS_FLASH_RANGE, &mut NoCache::new(), &buffer, false).await;
            }
            // Ignore the result, we can't do anything about it.
            // If we can't borrow the flash, just discard the log.
        }
        if mask.is_active() {
            unsafe {
                cortex_m::interrupt::enable();
            }
        }
        drop(cs);
    }

    /// Best-effort log. May silently discard the log if another task is
    /// using the flash.
    pub async fn log_str(&self, msg: &str) {
        let mut buffer = [0u8; 2048];
        let mut writer = FixedWriter::new(&mut buffer);

        cortex_m::interrupt::free(|cs| {
            let mut rtc = RTC.borrow(cs).borrow_mut();
            let (h, m, s, millis) = rtc.as_mut().unwrap().get_datetime().as_hms_milli();
            let _ = write!(writer, "{:02}:{:02}:{:02}.{:03} {}\n", h, m, s, millis, msg);
        });

        self.log_impl(writer.data()).await;
    }

    /// Try to take exclusive control of the flash. Other writers will fail
    /// until the flash is returned.
    /// Used for long-running operations like erasing the flash or reading
    /// all logs.
    async fn take_flash(&self) -> Option<W25QSequentialStorage<Bank1, CAPACITY>> {
        loop {
            let mask = primask::read();
            cortex_m::interrupt::disable();
            let cs = unsafe { CriticalSection::new() };
            let rc = self.flash.borrow(&cs);
            let Ok(mut flash) = rc.try_borrow_mut() else {
                // Another task is using the flash, we can yield and try again.
                drop(cs);
                if mask.is_active() {
                    unsafe {
                        cortex_m::interrupt::enable();
                    }
                }
                YieldFuture::new().await;
                continue;
            };
            let taken = flash.take();
            drop(flash);
            drop(cs);
            if mask.is_active() {
                unsafe {
                    cortex_m::interrupt::enable();
                }
            }

            return taken;
        }
    }

    async fn return_flash(&self, flash: W25QSequentialStorage<Bank1, CAPACITY>) {
        loop {
            let mask = primask::read();
            cortex_m::interrupt::disable();
            let cs = unsafe { CriticalSection::new() };
            let rc = self.flash.borrow(&cs);

            let Ok(mut flash_ref) = rc.try_borrow_mut() else {
                // Another task is using the flash, we can yield and try again.
                drop(cs);
                if mask.is_active() {
                    unsafe {
                        cortex_m::interrupt::enable();
                    }
                }
                YieldFuture::new().await;
                continue;
            };

            flash_ref.replace(flash);
            drop(flash_ref);
            drop(cs);
            if mask.is_active() {
                unsafe {
                    cortex_m::interrupt::enable();
                }
            }

            break;
        }
    }

    pub async fn get_logs<'a>(
        &'a self,
        mut f: impl AsyncFnMut(&'_ str),
    ) -> Result<(), &'static str> {
        let mut no_cache = NoCache::new();
        let mut flash = self.take_flash().await;
        let Some(mut flash) = flash.take() else {
            // Flash not available
            return Err("Flash not available");
        };

        let mut iterator = queue::iter(&mut flash, LOGS_FLASH_RANGE, &mut no_cache)
            .await
            .unwrap();

        let mut buffer = [0u8; 2048];
        while let Ok(Some(ref buf)) = iterator.next(&mut buffer).await {
            let s = core::str::from_utf8(buf).unwrap_or("[Corrupted line]\n");
            f(s).await;
        }

        self.return_flash(flash).await;
        Ok(())
    }

    pub async fn erase_logs(&self) -> Result<(), &'static str> {
        let flash = self.take_flash().await;
        let Some(mut flash) = flash else {
            // Flash not available
            return Err("Flash not available");
        };

        let mut data_buffer = [0u8; 256];
        let mut config = heapless::Vec::<(&'static str, u64), { CONFIG_KEYS.len() }>::new();
        for (k, value_type) in CONFIG_KEYS {
            let key: ConfigKey = k.try_into().unwrap();
            match value_type {
                storage_types::ValueType::U64 => {
                    if let Ok(Some(v)) = map::fetch_item(
                        &mut flash,
                        CONFIG_FLASH_RANGE,
                        &mut NoCache::new(),
                        &mut data_buffer,
                        &key,
                    )
                    .await
                    {
                        config.push((k, v)).ok();
                    }
                }
            }
        }

        // Drop the SequentialStorage wrapper to get the underlying flash
        let mut flash = flash.release();
        let pending = flash.chip_erase().unwrap();

        let mut color = 255u8;
        let mut i = 0;
        let pulse = async {
            loop {
                neopixel::update_pixel(0, [color, color, 0]);
                i += 1;
                // crappy pulse without using a timer
                if i % 100 == 0 {
                    color = color ^ 255;
                }
                YieldFuture::new().await;
            }
        };

        select(pulse, pending).await;
        neopixel::update_pixel(0, [0, 128, 0]);

        let mut flash = W25QSequentialStorage::<_, { CAPACITY }>::new(flash);

        for (k, v) in config {
            let key: ConfigKey = k.try_into().unwrap();
            let _ = map::store_item(
                &mut flash,
                CONFIG_FLASH_RANGE,
                &mut NoCache::new(),
                &mut data_buffer,
                &key,
                &v,
            )
            .await;
        }

        self.return_flash(flash).await;
        Ok(())
    }

    pub async fn edit_config(&self, key: &ConfigKey, value: u64) {
        let mask = primask::read();
        cortex_m::interrupt::disable();
        loop {
            let cs = unsafe { CriticalSection::new() };
            let rc = self.flash.borrow(&cs);
            let Ok(mut flash) = rc.try_borrow_mut() else {
                // Another task is using the flash, we can yield and try again.
                if mask.is_active() {
                    unsafe {
                        cortex_m::interrupt::enable();
                    }
                }
                YieldFuture::new().await;
                cortex_m::interrupt::disable();
                continue;
            };
            let Some(flash) = flash.as_mut() else {
                break;
            };
            let _ = map::store_item(
                flash,
                CONFIG_FLASH_RANGE,
                &mut NoCache::new(),
                &mut [0u8; 2048],
                key,
                &value,
            )
            .await;
            break;
        }

        if mask.is_active() {
            unsafe {
                cortex_m::interrupt::enable();
            }
        }
    }
}
