llvm-objcopy -O binary $1 firmware.bin
dfu-util -a 0 --dfuse-address 0x08000000 -D firmware.bin
