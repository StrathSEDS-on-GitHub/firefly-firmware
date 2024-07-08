#!/bin/sh

#llvm-objcopy -O binary $1 firmware.bin
#dfu-util -a 0 --dfuse-address 0x08000000 -D firmware.bin

arm-none-eabi-gdb   \
                -ex 'target extended-remote localhost:3334' \
                -ex 'load $1' \
                -ex 'set confirm off' \
                -ex 'file $1' \
                -ex 'monitor arm semihosting enable'
