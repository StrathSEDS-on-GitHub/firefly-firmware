MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  FLASH : ORIGIN   = 0x08000000, LENGTH = 1M
  RAM : ORIGIN     = 0x20000000, LENGTH = 256K
  EXTERNAL_FLASH : ORIGIN = 0x90010000, LENGTH = 16M
}

SECTIONS {
  .ota_update : ALIGN(4) {
    sota_update = .;
    *(.ota_update_begin*)
    *(.ota_update*)
    eota_update = .;
  } > EXTERNAL_FLASH AT> FLASH
} INSERT AFTER .text;
