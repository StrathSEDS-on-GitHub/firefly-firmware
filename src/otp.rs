use core::ffi::CStr;
use stm32f4xx_hal::pac::FLASH; 

const OTP_BLOCK0: *mut u8 = 0x1FFF7800 as *mut u8;
const OTP_BLOCK1: *mut [u8] = core::ptr::from_raw_parts_mut(0x1FFF7820 as *mut u8, 32);
const OTP_LOCKB0: *mut u8 = 0x1FFF7A00 as *mut u8;
const OTP_LOCKB1: *mut u8 = 0x1FFF7A01 as *mut u8;

pub fn write_board_identifier_to_otp(flash: FLASH, id: u8, version: &[u8], lock: bool) {
    // Unlocks flash register
    flash.keyr().write(|w| unsafe { w.bits(0x45670123) });
    flash.keyr().write(|w| unsafe { w.bits(0xCDEF89AB) });

    // Checks BSY bit to ensure no write is taking place
    while flash.sr().read().bsy().bit() {}

    // Sets PG bit and sets to x8 parallelism
    flash 
        .cr()
        .modify(|_, w| w.pg().set_bit().psize().psize8());

    unsafe {
        OTP_BLOCK0.write_volatile(id);
        (&mut *OTP_BLOCK1)[0..9].copy_from_slice(version);

        if lock {
            OTP_LOCKB0.write_volatile(0x00);
            OTP_LOCKB1.write_volatile(0x00);
        }
    }

    while flash.sr().read().bsy().bit() {}
}

pub fn read_id_from_otp() -> u8 {
    let mut val = 0;

    unsafe {
        val = OTP_BLOCK0.read_volatile();
    }

    return val;
}

pub fn read_version_from_otp() -> &'static str {
    let val: &str;

    unsafe {
        val = CStr::from_bytes_with_nul(&(&*OTP_BLOCK1)[..9])
            .unwrap()
            .to_str()
            .unwrap();
    }

    return val;
}
