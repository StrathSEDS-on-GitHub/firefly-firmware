MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  FLASH : ORIGIN   = 0x08000000, LENGTH = 1M
  RAM : ORIGIN     = 0x20000000, LENGTH = 256K
}

SECTIONS {
  .ota_update : ALIGN(4) {
    sota_update = .;
    *(.ota_update)
    eota_update = .;
  } > FLASH
} INSERT AFTER .text;
