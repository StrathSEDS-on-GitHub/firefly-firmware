#![no_std]

use core::convert::Infallible;

use heapless::String;
use sequential_storage::map::StorageItem;

#[derive(Debug)]
pub struct U64Item(pub String<32>,pub  u64);

impl StorageItem for U64Item {
    type Key = String<32>;
    type Error = Infallible;
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        let len = self.0.len();
        buffer[0] = len as u8;
        buffer[1..len + 1].copy_from_slice(&self.0.as_bytes());
        buffer[len + 1..len + 9].copy_from_slice(&self.1.to_le_bytes());
        Ok(len + 9)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<Self, Self::Error> {
        let len = buffer[0] as usize;
        assert!(len <= 32);
        let key = core::str::from_utf8(&buffer[1..len + 1]).unwrap().try_into().unwrap();
        let value = u64::from_le_bytes(buffer[len + 1..len + 9].try_into().unwrap());
        Ok(U64Item(key, value))
    }

    fn key(&self) -> Self::Key {
        // FIXME: this shouldn't have to be cloned
        self.0.clone()
    }
}