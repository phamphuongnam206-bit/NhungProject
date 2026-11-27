# Reengineering report

This is a new C11 project. No baseline source module was copied into `src/` or `include/`. The measured configuration and fixed obstacle coordinates were transcribed with provenance. Design ideas were studied from the user's C/C++ projects and the MIT-licensed Micromouse reference; see [REFERENCES](REFERENCES.md).

| Concern | Existing PCB port | New project | Provenance / validation |
| --- | --- | --- | --- |
| Layout | C++ board, motor, odometry, Navigator and run mode | C11 BSP, drivers, PI, odometry, map, motion, safety, FSM | New implementation; static build/test only |
| Board | HAL clock/GPIO/TIM/I2C2+I2C1 | HAL BSP using the same verified pins, only I2C1 for V1 | Configuration from baseline; not flashed |
| Wheel control | Feedforward + PI, corrected sign clamp | Separate pure `speed_pi_step`, signed output, capped duty, target slew | Reimplemented from control law; host sign test PASS |
| Odometry | C++ fused odometry | C struct and midpoint integration, 98/2 gyro/encoder blend | Reimplemented formulas; host motion test PASS |
| Route | Runtime A*, smoothing, pure pursuit | Fixed route with explicit full-body sweep and stop-turn-go | New implementation; simulated route PASS |
| Scheduling | 200 Hz motor, ~30 Hz main, ToF in loop | 200 Hz TIM1 motor, 5 ms gyro / 20 ms main, no ToF V1 | New timing; MCU WCET unknown |
| State/safety | Recoverable supervisor plus run loop | Explicit START/STOP/countdown/GOAL/latched FAULT, command lease and IWDG | New FSM tests PASS; hardware pending |
| Diagnostics | Flash run records, SWD structs | Live SWD status and frozen fault snapshot resolved from ELF | New; hardware read pending |
| Dynamic sensors | Three ToF sensors on PCB port | V1 excludes all; V2 motor-locked stub only | Four-sensor integration not done |

This is not a line-by-line C translation or a renaming exercise. It also is **not yet behaviourally equivalent** to the moving Micromouse port: the new follower, loop period and reduced speed have only host results. V2 has no occupancy grid or replanning yet. There is no claim of lower similarity percentage, better real-world performance, or real-car success.
