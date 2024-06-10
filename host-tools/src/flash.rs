use std::{
    io::{Read, Seek, Write},
    marker::PhantomData,
};

use embedded_storage_async::nor_flash::{ErrorType, NorFlash, NorFlashError, ReadNorFlash};

pub const CONFIG_FLASH_RANGE: core::ops::Range<u32> = 0..8192;
pub const LOGS_FLASH_RANGE: core::ops::Range<u32> = 8192..(16777216);

pub struct Cache;
pub struct NoCache;

pub struct FileWrapper<MODE> {
    file: std::fs::File,
    read_cache: Vec<u8>,
    dirty: bool,
    _mode: PhantomData<MODE>,
}

impl FileWrapper<()> {
    pub fn new(mut file: std::fs::File) -> anyhow::Result<FileWrapper<Cache>> {
        let mut cache = vec![];
        file.read_to_end(&mut cache)?;
        file.seek(std::io::SeekFrom::Start(0)).unwrap();
        Ok(FileWrapper {
            file,
            read_cache: cache,
            dirty: false,
            _mode: PhantomData,
        })
    }
    pub fn new_no_cache(file: std::fs::File) -> anyhow::Result<FileWrapper<NoCache>> {
        Ok(FileWrapper {
            file,
            read_cache: vec![],
            dirty: false,
            _mode: PhantomData,
        })
    }
}

#[derive(Debug)]
pub struct FileErrorType;
impl NorFlashError for FileErrorType {
    fn kind(&self) -> embedded_storage_async::nor_flash::NorFlashErrorKind {
        embedded_storage_async::nor_flash::NorFlashErrorKind::Other
    }
}

impl<MODE> ErrorType for FileWrapper<MODE> {
    type Error = FileErrorType;
}

impl ReadNorFlash for FileWrapper<NoCache> {
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
impl ReadNorFlash for FileWrapper<Cache> {
    async fn read(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), FileErrorType> {
        if self.dirty {
            self.read_cache.clear();
            self.file.seek(std::io::SeekFrom::Start(0)).unwrap();
            self.file.read_to_end(&mut self.read_cache).unwrap();
            self.dirty = false;
        }

        buffer.copy_from_slice(
            &self.read_cache[address as usize..(address + buffer.len() as u32) as usize],
        );
        Ok(())
    }

    const READ_SIZE: usize = 1;

    fn capacity(&self) -> usize {
        self.file.metadata().unwrap().len() as usize
    }
}

impl<MODE> NorFlash for FileWrapper<MODE>
where
    FileWrapper<MODE>: ReadNorFlash,
{
    async fn write(&mut self, address: u32, data: &[u8]) -> Result<(), Self::Error> {
        self.file
            .seek(std::io::SeekFrom::Start(address as u64))
            .unwrap();
        self.file.write_all(data).unwrap();
        self.dirty = true;
        Ok(())
    }

    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = 4096;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if to == LOGS_FLASH_RANGE.end && from == CONFIG_FLASH_RANGE.start {
            // Secret chip erase
            self.file.flush().unwrap();
            self.file.seek(std::io::SeekFrom::Start(0x6969)).unwrap();
            self.file.write_all(&[0x42, 0x42, 0x42, 0x42, 0x42]).unwrap();
            self.file.flush().unwrap();
            self.dirty = true;
            return Ok(());
        }
        panic!("Partial erase not implemented");
    }
}
