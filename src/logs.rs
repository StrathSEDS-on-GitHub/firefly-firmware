use core::cell::SyncUnsafeCell;
use core::fmt::{self, Write};

use f4_w25q::embedded_storage::W25QSequentialStorage;
use sequential_storage::cache::{NoCache, PagePointerCache};
use sequential_storage::{map, queue};

use storage_types::logs::{LocalCtxt, Message, MessageType};
use storage_types::{CONFIG_KEYS, ConfigKey};
use thingbuf::mpsc::{StaticChannel, StaticReceiver, StaticSender};

use crate::futures::YieldFuture;
use crate::logs::StorageTask::*;
use crate::mission::current_rtc_time;
use crate::pins::QspiBank;
use crate::{CAPACITY, CONFIG_FLASH_RANGE, LOGS_FLASH_RANGE, PAGE_COUNT, neopixel};

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

pub(crate) static LOG_CHANNEL: StaticChannel<StorageTask, 32> = StaticChannel::new();
pub(crate) static LOG_SENDER: SyncUnsafeCell<Option<StaticSender<StorageTask>>> =
    SyncUnsafeCell::new(None);

#[derive(Clone, Default)]
pub enum StorageTask {
    Msg(Message<LocalCtxt>),
    GetLogs(&'static (dyn Fn(&Message<LocalCtxt>) + Sync)),
    EraseLogs,
    ReadConfig(ConfigKey, &'static (dyn Fn(ConfigKey, u64) + Sync)),
    // really dislike how an option is in the function signature
    ReadAllConfig(&'static (dyn Fn([(Option<ConfigKey>, u64); CONFIG_KEYS.len()]) + Sync)),
    EditConfig(ConfigKey, u64),
    GetSpaceLeft(&'static (dyn Fn(u32) + Sync)),
    #[default]
    DoNothing, // Needed for StaticChannel DefaultRecycle
}
pub fn log_sender() -> StaticSender<StorageTask> {
    // SAFETY: LOG_SENDER is only written to once, in main(), before any tasks are spawned.
    let borrow = unsafe { &*LOG_SENDER.get() };
    if borrow.is_none() {
        panic!("log_sender() called before split");
    }
    borrow.as_ref().unwrap().clone()
}

pub fn log(msg: Message<LocalCtxt>) {
    let _ = log_sender().try_send(StorageTask::Msg(msg));
}

#[allow(unused)]
pub async fn blocking_log(msg: Message<LocalCtxt>) {
    log_sender().send(StorageTask::Msg(msg)).await.unwrap();
}

pub fn log_str(msg: &str) {
    let time: u32 = current_rtc_time();
    log(MessageType::new_log(time, msg)
        .unwrap()
        .into_message(LocalCtxt { timestamp: time }));
}

pub async fn get_logs(f: &'static (dyn Fn(&Message<LocalCtxt>) + Sync)) {
    log_sender().send(StorageTask::GetLogs(f)).await.unwrap();
}

pub async fn erase_logs() {
    log_sender().send(StorageTask::EraseLogs).await.unwrap();
}

pub async fn edit_config(key: ConfigKey, value: u64) {
    log_sender()
        .send(StorageTask::EditConfig(key, value))
        .await
        .unwrap();
}

pub async fn read_config(key: ConfigKey, f: &'static (dyn Fn(ConfigKey, u64) + Sync)) {
    log_sender()
        .send(StorageTask::ReadConfig(key, f))
        .await
        .unwrap();
}

pub async fn read_all_config(
    f: &'static (dyn Fn([(Option<ConfigKey>, u64); CONFIG_KEYS.len()]) + Sync),
) {
    log_sender()
        .send(StorageTask::ReadAllConfig(f))
        .await
        .unwrap();
}

pub async fn get_space_left(f: &'static (dyn Fn(u32) + Sync)) {
    log_sender()
        .send(StorageTask::GetSpaceLeft(f))
        .await
        .unwrap();
}

pub async fn log_handler(
    mut flash: W25QSequentialStorage<QspiBank, { CAPACITY }>,
    receiver: StaticReceiver<StorageTask>,
) -> ! {
    let mut cache: PagePointerCache<{ PAGE_COUNT }> = PagePointerCache::new();

    loop {
        if let Some(task) = receiver.recv().await {
            match task {
                Msg(msg) => {
                    let mut buf = [0u8; 2048];
                    if let Ok(data) = postcard::to_slice(&msg, &mut buf) {
                        let _ = queue::push(&mut flash, LOGS_FLASH_RANGE, &mut cache, &data, false)
                            .await;
                    }
                }
                GetLogs(f) => {
                    let mut iter = queue::iter(&mut flash, LOGS_FLASH_RANGE, &mut cache)
                        .await
                        .unwrap();

                    let mut buf = [0u8; 2048];

                    while let Ok(Some(ref buffer)) = iter.next(&mut buf).await {
                        let msg: Message<LocalCtxt> = match postcard::from_bytes(buffer) {
                            Ok(msg) => msg,
                            Err(_) => continue,
                        };

                        f(&msg);
                    }
                }
                EraseLogs => {
                    let mut buf = [0u8; 256];
                    let mut config =
                        heapless::Vec::<(&'static str, u64), { CONFIG_KEYS.len() }>::new();

                    for (k, value_type) in CONFIG_KEYS {
                        let key: ConfigKey = k.try_into().unwrap();

                        match value_type {
                            storage_types::ValueType::U64 => {
                                if let Ok(Some(v)) = map::fetch_item(
                                    &mut flash,
                                    CONFIG_FLASH_RANGE,
                                    &mut NoCache::new(),
                                    &mut buf,
                                    &key,
                                )
                                .await
                                {
                                    let _ = config.push((k, v));
                                }
                            }
                        }
                    }

                    let mut flashh = flash.release();
                    let pending = flashh.chip_erase().unwrap();

                    let mut color = 255u8;
                    let mut i = 0;
                    let ooh_pretty_lights = async {
                        loop {
                            neopixel::update_pixel(0, [color, color, 0]);
                            i += 1;

                            if i % 100 == 0 {
                                color ^= 255;
                            }

                            YieldFuture::new().await;
                        }
                    };

                    embassy_futures::select::select(ooh_pretty_lights, pending).await;
                    neopixel::update_pixel(0, [0, 128, 0]);

                    flash = W25QSequentialStorage::<_, { CAPACITY }>::new(flashh);
                    cache = PagePointerCache::new();

                    for (k, v) in config {
                        let key: ConfigKey = k.try_into().unwrap();

                        let _ = map::store_item(
                            &mut flash,
                            CONFIG_FLASH_RANGE,
                            &mut NoCache::new(),
                            &mut buf,
                            &key,
                            &v,
                        )
                        .await;
                    }
                }
                ReadConfig(key, f) => {
                    let mut buf = [0u8; 64];
                    let thing = map::fetch_item(
                        &mut flash,
                        CONFIG_FLASH_RANGE,
                        &mut NoCache::new(),
                        &mut buf,
                        &key,
                    )
                    .await;

                    if let Ok(Some(value)) = thing {
                        f(key, value);
                    }
                }
                ReadAllConfig(f) => {
                    let mut buf = [0u8; 64];

                    let mut cache = NoCache::new();

                    let thing = map::fetch_all_items::<ConfigKey, _, _>(
                        &mut flash,
                        CONFIG_FLASH_RANGE,
                        &mut cache,
                        &mut buf,
                    )
                    .await;

                    if let Ok(mut iter) = thing {
                        let mut buff = [0u8; 64];
                        let mut res: [(Option<ConfigKey>, u64); CONFIG_KEYS.len()] =
                            core::array::from_fn(|_| (None, 0));

                        for i in 0..CONFIG_KEYS.len() {
                            if let Ok(Some((k, v))) = iter.next(&mut buff).await {
                                let k: ConfigKey = k;
                                res[i] = (Some(k), v);
                            }
                        }

                        f(res);
                    }
                }
                EditConfig(key, value) => {
                    let _ = map::store_item(
                        &mut flash,
                        CONFIG_FLASH_RANGE,
                        &mut NoCache::new(),
                        &mut [0u8; 64],
                        &key,
                        &value,
                    )
                    .await;
                }
                GetSpaceLeft(f) => {
                    let space = queue::space_left(&mut flash, LOGS_FLASH_RANGE, &mut cache).await;

                    if let Ok(space) = space {
                        f(space);
                    }
                }
                DoNothing => {
                    // well, what did you expect?
                }
            }
        }
    }
}
