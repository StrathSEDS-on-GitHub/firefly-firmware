# firefly-firmware

The firmware for the [firefly flight computer](https://github.com/StrathSEDS-on-GitHub/Mach23-Firefly). 

Developed by StrathSEDS for the Mach-23 launch competition and flown for the first time at IRW 2023 at 
Fairlie Moor. 

[![Strathoshpere II's flight](https://img.youtube.com/vi/E2Db4XrMgVg/sddefault.jpg)](https://www.youtube.com/watch?v=E2Db4XrMgVg0)

Features
    - Cooperative multi-tasking using async/await using no RTOS.
    - Time division multiplexed radio controller.
    - 10Hz GPS + 40Hz altimeter recorded telemetry onto both QSPI flash and SD card.
