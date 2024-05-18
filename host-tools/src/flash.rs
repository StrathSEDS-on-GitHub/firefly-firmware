use std::io::{Read, Seek, Write};

use embedded_storage_async::nor_flash::{ErrorType, NorFlash, NorFlashError, ReadNorFlash};

pub const CONFIG_FLASH_RANGE: core::ops::Range<u32> = 0..8192;
pub const LOGS_FLASH_RANGE: core::ops::Range<u32> = 8192..(16777216);

pub struct FileWrapper {
    file: std::fs::File,
}

impl FileWrapper {
    pub fn new(file: std::fs::File) -> Self {
        Self { file }
    }
}

#[derive(Debug)]
pub struct FileErrorType;
impl NorFlashError for FileErrorType {
    fn kind(&self) -> embedded_storage_async::nor_flash::NorFlashErrorKind {
        embedded_storage_async::nor_flash::NorFlashErrorKind::Other
    }
}

impl ErrorType for FileWrapper {
    type Error = FileErrorType;
}

impl ReadNorFlash for FileWrapper {
    async fn read(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), FileErrorType> {
        self.file
            .seek(std::io::SeekFrom::Start(address as u64))
            .unwrap();
        self.file.read_exact(buffer).unwrap();
        Ok(())
    }

    const READ_SIZE: usize = 1;

    fn capacity(&self) -> usize {
        self.file.metadata().unwrap().len() as usize
    }
}

impl NorFlash for FileWrapper {
    async fn write(&mut self, address: u32, data: &[u8]) -> Result<(), FileErrorType> {
        self.file
            .seek(std::io::SeekFrom::Start(address as u64))
            .unwrap();
        self.file.write_all(data).unwrap();
        Ok(())
    }

    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = 4096;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if to == LOGS_FLASH_RANGE.end && from == CONFIG_FLASH_RANGE.start {
            // Secret chip erase
            self.file.flush().unwrap();
            self.file.seek(std::io::SeekFrom::Start(0x6969)).unwrap();
            self.file.write(&[0x42, 0x42, 0x42, 0x42, 0x42]).unwrap();
            self.file.flush().unwrap();
            self.file
                .seek(std::io::SeekFrom::Start(from as u64))
                .unwrap();
            self.file.flush().unwrap();
            let buf = vec![0xFFu8; (to - from) as usize];

            self.file.write_all(&buf).unwrap();
            return Ok(());
        }
        panic!("Erase not implemented");
    }
}
