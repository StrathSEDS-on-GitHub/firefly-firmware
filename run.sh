#!/bin/sh

# check if USE_DFU is set
if [ -z "$USE_DFU" ]; then
    if [ -z $(pidof openocd) ]; then
        echo "openOCD is not running, please run 'openocd -f interface/stlink.cfg -f target/stm32f4x.cfg' in another terminal."
    else
        arm-none-eabi-gdb   \
                        -ex 'target extended-remote localhost:3333' \
                        -ex "load $1" \
                        -ex 'set confirm off' \
                        -ex "file $1" \
                        -ex 'monitor arm semihosting enable'
    fi
else
    # If USE_DFU is set, use the dfu-util command
    echo "Using DFU mode to flash the firmware."
    llvm-objcopy -O binary $1 firmware.bin
    dfu-util -a 0 --dfuse-address 0x08000000 -D firmware.bin
fi

