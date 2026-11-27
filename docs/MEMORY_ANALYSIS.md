# Memory and timing analysis

PlatformIO `ststm32` / `bluepill_f103c8`, STM32Cube HAL, GNU C11, `-fstack-usage`, nominal Flash 65,536 B and SRAM 20,480 B. Build from 27 November 2026:

| Profile | Flash (`pio`) | Static RAM (`pio`) | Meaning |
| --- | ---: | ---: | --- |
| `robot_v1` | 25,444 B | 856 B | Static-map firmware, not uploaded |
| `robot_v2` | 23,568 B | 856 B | Motor-locked scaffold; not a full V2 |

`robot_v1.map` is emitted during linking. Its `.data` is 116 B, `.bss` 740 B, and linker heap/stack reserve 512/1024 B. The application's largest individual compiler stack frame currently reported by `.su` is `bsp_init` at 120 B; `main` is 112 B, `map_body_clear` 96 B and `motion_update` 80 B. **Those are individual frames, not a verified worst-case call chain or measured high-water mark.** HAL/I2C call depth and IRQ nesting still need stack high-water instrumentation on hardware.

TIM1 drives the wheel loop at 5 ms; the main loop schedules gyro at 5 ms and navigation at 20 ms. Execution times have not been measured on the 72 MHz MCU. A main-loop interval >100 ms faults, but this is a missed-deadline detector, not a WCET proof. Runtime I2C1 polling has a 3 ms timeout; the motor IRQ never waits on I2C.
