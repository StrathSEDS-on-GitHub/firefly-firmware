
# Windows Installation Instructions

## Requirments

* Install git if not already installed

* Authenticate git: [docs.github.com](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/checking-for-existing-ssh-keys)

* Install [OpenOCD](https://github.com/xpack-dev-tools/openocd-xpack/), [socat for windows](https://github.com/tech128/socat-1.7.3.0-windows), and [dfu-util](https://sourceforge.net/projects/dfu-util/files/dfu-util-0.9-win64.zip/download)

* Add all these to your path

* Follow [SETUP.md](./SETUP.md)

* Clone the repo: `git clone https://github.com/finlaysr/firefly-firmware-fork --recursive`

    * If you forget the recursive: `git submodule update --init –recursive`

## File changes

* Change this line in ./cargo/config.toml: `runner = "powershell.exe -ExecutionPolicy Bypass -File run.ps1"`

* Add this file ./run.ps1 -> 
    
    `$path = $args[0] - replace '\\', '/'`

    `arm-none-eabi-gdb -ex "target extended-remote localhost:3333" -ex "load $path" -ex "set confirm off" -ex "file $path" -ex "monitor arm semihosting enable"`
    
## Useful commands

* `cargo build --release --features target-ultra`

* `openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "init"`

* `socat - /dev/ttyS10`

* `git submodule update --init –recursive`