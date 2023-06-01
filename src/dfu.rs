use stm32f4xx_hal::{flash::FlashExt, pac::FLASH};

#[link_section = ".ota_update"]
static SECTOR_SIZES: [u32; 11] = [
    16 * 1024,
    16 * 1024,
    16 * 1024,
    16 * 1024,
    64 * 1024,
    128 * 1024,
    128 * 1024,
    128 * 1024,
    128 * 1024,
    128 * 1024,
    128 * 1024,
];

#[link_section = ".ota_update_begin"]
pub fn begin_update(firmware: &[u8], mut internal_flash: FLASH) {
    let mut firmware = firmware;

    unlock_flash(&mut internal_flash);

    let mut sector_idx = 0;
    let mut bytes_written = 0;
    while bytes_written < firmware.len() {
        let sector_size = SECTOR_SIZES[sector_idx as usize];
        erase(&mut internal_flash, sector_idx);
        let bytes_to_write = core::cmp::min(firmware.len() - bytes_written, sector_size as usize);
        let (firmware_to_write, remaining_firmware) = firmware.split_at(bytes_to_write);
        for chunk in firmware_to_write.chunks(128 / 8) {
            let address = bytes_written;
            if chunk.len() < 128 / 8 {
                let mut padded_chunk = [0; 128 / 8];
                padded_chunk[..chunk.len()].copy_from_slice(chunk);
                program(&mut internal_flash, address, padded_chunk.iter());
            } else {
                program(&mut internal_flash, address, chunk.iter());
            }
            bytes_written += chunk.len();
        }
        firmware = remaining_firmware;
        sector_idx += 1;
    }

    lock(&internal_flash);

    // Reset the device
    cortex_m::peripheral::SCB::sys_reset();
}

#[link_section = ".ota_update"]
fn unlock_flash(flash: &mut FLASH) {
    const UNLOCK_KEY1: u32 = 0x45670123;
    const UNLOCK_KEY2: u32 = 0xCDEF89AB;
    flash.keyr.write(|w| w.key().bits(UNLOCK_KEY1));
    flash.keyr.write(|w| w.key().bits(UNLOCK_KEY2));
}

#[link_section = ".ota_update"]
fn erase(flash: &mut FLASH, sector: u8) {
    let snb = if sector < 12 { sector } else { sector + 4 };
    const PSIZE_X8: u8 = 0b00;

    #[rustfmt::skip]
    flash.cr.modify(|_, w| unsafe {
        w
            // start
            .strt().set_bit()
            .psize().bits(PSIZE_X8)
            // sector number
            .snb().bits(snb)
            // sectore erase
            .ser().set_bit()
            // no programming
            .pg().clear_bit()
    });
    wait_ready(flash);
}

#[link_section = ".ota_update"]
fn wait_ready(flash: &mut FLASH) {
    while flash.sr.read().bsy().bit() {}
}

#[link_section = ".ota_update"]
fn program<'a, I>(flash: &mut FLASH, mut offset: usize, mut bytes: I)
where
    I: Iterator<Item = &'a u8>,
{
    const PSIZE_X8: u8 = 0b00;
    let ptr = 0x0800_0000 as *mut u8;
    let mut bytes_written = 1;
    while bytes_written > 0 {
        bytes_written = 0;
        let amount = 16 - (offset % 16);

        #[rustfmt::skip]
        flash.cr.modify(|_, w| 
            w
                .psize().bits(PSIZE_X8)
                // no sector erase
                .ser().clear_bit()
                // programming
                .pg().set_bit()
        );
        for _ in 0..amount {
            match bytes.next() {
                Some(byte) => {
                    unsafe {
                        core::ptr::write_volatile(ptr.add(offset), *byte);
                    }
                    offset += 1;
                    bytes_written += 1;
                }
                None => break,
            }
        }
        wait_ready(flash);
    }
    flash.cr.modify(|_, w| w.pg().clear_bit());
}

#[link_section = ".ota_update"]
fn lock(flash: &FLASH) {
    flash.cr.modify(|_, w| w.lock().set_bit());
}