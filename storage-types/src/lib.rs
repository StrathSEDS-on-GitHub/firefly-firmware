#![no_std]

use core::str::FromStr;

use heapless::String;
use sequential_storage::map::Key;
use serde::{Deserialize, Serialize};

pub mod logs;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConfigKey(String<32>);

#[derive(Debug, Eq, PartialEq, Clone, Copy, Serialize, Deserialize)]
#[repr(u8)]
pub enum PyroPin {
    One,
    Two,
    Both
}

#[derive(Debug, Eq, PartialEq, Clone, Copy, Serialize, Deserialize)]
#[repr(u8)]
pub enum Role {
    Cansat,
    Avionics,
    CansatBackup,
    GroundMain,
    GroundBackup,
}

#[derive(Debug, Eq, PartialEq, Clone, Copy, Serialize, Deserialize)]
#[repr(u8)]
pub enum MissionStage {
    Disarmed,
    Armed,
    Ascent,
    DescentDrogue,
    DescentMain,
    Landed,
}


pub enum ValueType {
    U64,
}

pub const CONFIG_KEYS: [(&'static str, ValueType); 6] = [
    ("id", ValueType::U64),
    ("rf_freq", ValueType::U64),
    ("cr", ValueType::U64),
    ("sf", ValueType::U64),
    ("bw", ValueType::U64),
    ("power", ValueType::U64),
];

impl TryFrom<&str> for ConfigKey {
    type Error = ();

    fn try_from(value: &str) -> Result<Self, Self::Error> {
        Ok(ConfigKey(String::from_str(value)?))
    }
}

impl Key for ConfigKey {
    fn serialize_into(
        &self,
        buffer: &mut [u8],
    ) -> Result<usize, sequential_storage::map::SerializationError> {
        if buffer.len() < 1 + self.0.len() {
            return Err(sequential_storage::map::SerializationError::BufferTooSmall);
        }
        buffer[0] = self.0.len() as u8;
        buffer[1..(self.0.len() + 1)].copy_from_slice(&self.0.as_bytes());
        Ok(1 + self.0.len())
    }

    fn deserialize_from(
        buffer: &[u8],
    ) -> Result<(Self, usize), sequential_storage::map::SerializationError> {
        let mut key = String::new();
        let len = buffer
            .get(0)
            .cloned()
            .ok_or(sequential_storage::map::SerializationError::BufferTooSmall)?
            as usize;

        if buffer.len() < 1 + len {
            return Err(sequential_storage::map::SerializationError::BufferTooSmall);
        }

        key.push_str(
            core::str::from_utf8(&buffer[1..=len])
                .map_err(|_| sequential_storage::map::SerializationError::InvalidData)?,
        )
        .unwrap();

        Ok((ConfigKey(key), 1 + len))
    }
}