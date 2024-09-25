#SETUP Instructions for firefly-firmware
=======================================

## Step 1: Install Rust

1. First, get [rustup](https://rustup.rs/). `rustup` is an installer and toolchain manager for Rust.

2. Next up, add the nightly toolchain.

```
rustup toolchain install nightly
```

3. Then set it as the default

```
rustup default nightly
```

3. Finally, install the thumbv7em-eabihf target -- this is the instruction set used by the microprocessors we have.

```
rustup target add thumbv7em-none-eabihf
```

## Step 2: Install openocd

Unfortunately this varies a lot based on operating system.

On Arch distros: 
```
pacman -S openocd
```

On Debian distros:
```
apt install openocd
```

MacOS:

With homebrew
```
brew install open-ocd
```

Windows: https://github.com/openocd-org/openocd/releases/tag/v0.12.0
Note: add this to your PATH

## Step 3: Install gdb-multiarch

On Ubuntu derivatives:
```
apt install gdb-multiarch
```

On arch derivatives:
```
pacman -S arm-none-eabi-gdb
```

MacOS/Windows:
Select the arm-none-eabi package for your operating system: https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
Extract the file and add the bin/arm-none-eabi-gdb executable to your PATH

## Step 4: Patch the code with your changes!

To test your setup, try adding this code in main.rs after the `neopixel::new\_neopixel()` line
```
        let mut timer = dp.TIM10.counter_hz(&clocks);
        timer.start(1.Hz()).unwrap();
        let mut color = 0;
        loop {
            neopixel::update_pixel(0, [0, color, 0]);
            color  = !color;
            timer.wait().unwrap();
        }
```

## Step 5: Run your code

In a background terminal, run 
```
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "init"
```

Then, in the project run `cargo run --features target-mini`.

The neopixel should blink!

