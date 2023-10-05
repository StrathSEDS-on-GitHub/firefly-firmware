// Sets up a usb mass storage device.

use core::{sync::atomic::AtomicU32, cell::{UnsafeCell, RefCell}};

use cortex_m::peripheral::NVIC;
use cortex_m_semihosting::hprintln;
use f4_w25q::w25q::{SectorAddress, W25Q};
use smart_leds::SmartLedsWrite;
use stm32f4xx_hal::{
    gpio::{alt::otg_fs, Input, Pin},
    otg_fs::{UsbBus, UsbBusType, USB},
    pac::{OTG_FS_DEVICE, OTG_FS_GLOBAL, OTG_FS_PWRCLK},
    qspi::Bank1,
    rcc::Clocks,
};
use stm32f4xx_hal::{pac};
use usb_device::prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usbd_scsi::{BlockDevice, Scsi};

use crate::{logger::USB_BUS, EP_MEMORY, NEOPIXEL};

pub static mut USB_STORAGE: Option<Scsi<UsbBusType, Storage>> = None;
pub static mut USB_DEVICE: Option<UsbDevice<UsbBusType>> = None;

static OFFSET: AtomicU32 = AtomicU32::new(0);

// Cache up to one sector in RAM
static mut SECTOR_CACHE: [u8; 4096] = [0u8; 4096];
// The address of the cached sector
static mut SECTOR_CACHE_ADDR: Option<u32> = None;
// Which blocks of the cached sector are valid
static mut SECTOR_CACHED_BLOCKS: [bool; 8] = [false; 8];

pub struct Storage {
    host: RefCell<W25Q<Bank1>>,
}
impl Storage {
    fn flush_sector_cache(&mut self) {
        // Flush the cached sector to flash
        // Since we must erase a whole sector at a time, we need to read the uncached blocks from flash
        // and merge them with the cached blocks

        if let Some(cached_addr) = unsafe { SECTOR_CACHE_ADDR } {
            let sector = unsafe { &mut SECTOR_CACHE };
            for block in 0..8 {
                if !unsafe { SECTOR_CACHED_BLOCKS[block] } {
                    // This block is not
                    self.read_block(cached_addr/512 + block as u32, &mut sector[block * 512..(block + 1) * 512])
                        .unwrap();
                }
            }

            self.write_sector(cached_addr, sector).unwrap();
        }
    }

    // flash a whole sector
    fn write_sector(
        &mut self,
        addr: u32,
        sector: &[u8],
    ) -> Result<(), usbd_scsi::BlockDeviceError> {
        cortex_m::interrupt::free(|cs| {
            NEOPIXEL
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .write([[128, 0, 0u8]].into_iter())
                .unwrap()
        });

        self.host
            .borrow_mut()
            .erase_sector(SectorAddress::from_address(addr))
            .map_err(|_| usbd_scsi::BlockDeviceError::HardwareError)?;

        for (i, chunk) in sector.chunks(256).enumerate() {
            self.host
                .borrow_mut()
                .program_page((addr + (i as u32) * 256).into(), chunk)
                .map_err(|_| usbd_scsi::BlockDeviceError::HardwareError)?;
            self.host.borrow_mut().wait_on_busy().unwrap();
        }

        cortex_m::interrupt::free(|cs| {
            NEOPIXEL
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .write([[0, 0, 0u8]].into_iter())
                .unwrap()
        });

        Ok(())
    }
}

impl BlockDevice for Storage {
    const BLOCK_BYTES: usize = 512;

    fn read_block(&self, lba: u32, block: &mut [u8]) -> Result<(), usbd_scsi::BlockDeviceError> {
        if let Some(cached_addr) = unsafe { SECTOR_CACHE_ADDR } {
            // If our sector is cached
            if cached_addr == (lba / 8 * 4096) {
                // .. and the block is cached
                if unsafe { SECTOR_CACHED_BLOCKS[(lba % 8) as usize] } {
                    // .. then read from the cache
                    let offset = lba as usize * Self::BLOCK_BYTES % 4096;
                    block.copy_from_slice(unsafe {
                        &SECTOR_CACHE[offset..(offset + Self::BLOCK_BYTES)]
                    });
                    return Ok(());
                }
            }
        }

        //hprintln!("read {:x}", lba);
        cortex_m::interrupt::free(|cs| {
            NEOPIXEL
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .write([[0, 0, 128u8]].into_iter())
                .unwrap()
        });

        let res = self
            .host
            .borrow_mut()
            .read((lba * Self::BLOCK_BYTES as u32).into(), block)
            .map_err(|x| {
                usbd_scsi::BlockDeviceError::HardwareError
            });
        cortex_m::interrupt::free(|cs| {
            NEOPIXEL
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .write([[0, 0, 0u8]].into_iter())
                .unwrap()
        });

        res
    }

    fn write_block(&mut self, lba: u32, block: &[u8]) -> Result<(), usbd_scsi::BlockDeviceError> {
        if let Some(cached_addr) = unsafe { SECTOR_CACHE_ADDR } {
            if cached_addr != lba / 8 * 4096 {
                // This is a different sector, flush the cache
                self.flush_sector_cache();

                for i in 0..8 {
                    unsafe { SECTOR_CACHED_BLOCKS[i] = false };
                }
            }
        }
        let offset = lba as usize * Self::BLOCK_BYTES % 4096;
        unsafe {
            SECTOR_CACHE_ADDR = Some(lba / 8 * 4096);
            SECTOR_CACHE[offset..(offset + Self::BLOCK_BYTES)].copy_from_slice(block);
            SECTOR_CACHED_BLOCKS[(lba % 8) as usize] = true;
        }

        self.flush_sector_cache();

        Ok(())
    }

    fn max_lba(&self) -> u32 {
        // 128 Mbit / 8 = 16 MByte
        // 16 MByte / 512 = 32768
        16 * 1024 * 1024 / Self::BLOCK_BYTES as u32 - 1
    }
}

pub fn setup_usb_msc<'a>(
    otg_fs_global: OTG_FS_GLOBAL,
    otg_fs_device: OTG_FS_DEVICE,
    otg_fs_powerclock: OTG_FS_PWRCLK,
    gpioa_pa11: Pin<'A', 11, Input>,
    gpioa_pa12: Pin<'A', 12, Input>,
    clocks: &'a Clocks,
    w25q: W25Q<Bank1>,
) {
    let usb = USB {
        usb_global: otg_fs_global,
        usb_device: otg_fs_device,
        usb_pwrclk: otg_fs_powerclock,
        pin_dm: otg_fs::Dm::PA11(gpioa_pa11.into_alternate()),
        pin_dp: otg_fs::Dp::PA12(gpioa_pa12.into_alternate()),
        hclk: clocks.hclk(),
    };

    let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });
    // SAFETY: This function is the only access of USB_BUS and it is only called once at init.
    // As a result we can use static mut.
    unsafe {
        crate::logger::USB_BUS = Some(usb_bus);
    }
    let storage = Storage { host: RefCell::new(w25q) };

    let scsi = unsafe {
        Scsi::new(
            USB_BUS.as_ref().unwrap(),
            64,
            storage,
            "Fake co",
            "MSC",
            "TEST",
        )
    };

    let usb_dev = UsbDeviceBuilder::new(
        unsafe { &crate::logger::USB_BUS.as_ref().unwrap() },
        UsbVidPid(0x16c0, 0x27dd),
    )
    .manufacturer("Fake company")
    .product("Serial port")
    .serial_number("TEST")
    .device_class(usbd_mass_storage::USB_CLASS_MSC)
    .self_powered(true)
    .build();

    unsafe {
        USB_DEVICE = Some(usb_dev);
        USB_STORAGE = Some(scsi);
        NVIC::unmask(pac::Interrupt::OTG_FS);
    }
}

#[cfg(feature = "msc")]
use stm32f4xx_hal::interrupt;
#[cfg(feature = "msc")]
#[interrupt]
fn OTG_FS() {
    unsafe {
        USB_DEVICE
            .as_mut()
            .unwrap()
            .poll(&mut [USB_STORAGE.as_mut().unwrap()]);
    }
}
