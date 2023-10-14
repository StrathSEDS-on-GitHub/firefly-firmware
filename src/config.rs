use heapless::Vec;
use littlefs2::fs::Filesystem;
use serde::{Serialize, Deserialize};
use serde_json_core;

static mut CONFIG: Option<Config> = None;

#[derive(Debug, Deserialize)]
pub struct Config {
    pub main_deployment_height: f32,
}

impl Config {
    pub fn build(file: Vec<u8, 1024>) {
        let (config, _): (Config, _) = serde_json_core::from_slice(&file).unwrap();

        // Safety: Only ran once on startup, so reads will not alias with the write
        unsafe {
            CONFIG = Some(config);
        };
    }

    pub fn get() -> &'static Config {
        unsafe { CONFIG.as_ref().unwrap() }
    }
}