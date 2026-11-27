# STM32 Autonomous Robot

Independent C11 firmware project for the user's STM32F103C8T6 differential-drive robot. The existing `micromouse_pcb_exact_clean` firmware and the firmware currently flashed on the car are untouched.

`robot_v1` is a static-map, encoder/MPU waypoint controller. It does not initialize I2C2 or read a VL53L0X. It requires a deliberate PB1 START press after calibration, then waits three seconds. PB1 during motion stops into IDLE. A latched FAULT needs a power cycle. The route and geometry are checked at boot. **This new firmware has not been uploaded or driven on the real car.**

`robot_v2` currently builds as a **motor-locked scaffold** and reports `FAULT_UNSUPPORTED_PROFILE`. It is not an autonomous obstacle-avoidance implementation. V2 development is deferred until V1 receives physical acceptance and the four sensor mounts are measured.

## Build and test

```powershell
cd D:\prj_Nhung\STM32_Autonomous_Robot
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e robot_v1
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e robot_v2
powershell -ExecutionPolicy Bypass -File .\tools\test.ps1
```

No build command uploads firmware. The firmware on the robot remains the previously flashed V1 until the user deliberately runs the guarded upload script. See [USER_GUIDE](docs/USER_GUIDE.md).

## Project map

`bsp.c` owns STM32 clocks/pins/timers/I2C1; `encoder_driver.c` and `mpu_driver.c` read sensors; `motor_control.c` runs the 200 Hz wheel loop through `speed_pi.c`; `odometry.c` fuses wheel and gyro motion; `static_map.c` checks body clearance; `motion_controller.c` follows the fixed route; `safety_manager.c` checks failures; `robot_fsm.c` owns START/STOP/countdown/fault transitions. `main.c` schedules I2C and 50 Hz navigation. See [ARCHITECTURE](docs/ARCHITECTURE.md), [ALGORITHMS](docs/ALGORITHMS.md), and [TEST_REPORT](docs/TEST_REPORT.md).

Reference parameters and geometry come from the local PCB port and the group's map. Source history and licenses are documented in [BASELINE_PARAMETERS](docs/BASELINE_PARAMETERS.md), [REFERENCES](docs/REFERENCES.md), and [THIRD_PARTY_NOTICES](THIRD_PARTY_NOTICES.md).
