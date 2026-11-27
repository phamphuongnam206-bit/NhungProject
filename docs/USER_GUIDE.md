# User guide

## Build and inspect

Open `D:\prj_Nhung\STM32_Autonomous_Robot` as the PlatformIO project. On this Windows host, PowerShell script execution is restricted; invoke scripts with `powershell -ExecutionPolicy Bypass -File ...` for that process only.

```powershell
cd D:\prj_Nhung\STM32_Autonomous_Robot
powershell -ExecutionPolicy Bypass -File .\tools\build.ps1 -Profile robot_v1
powershell -ExecutionPolicy Bypass -File .\tools\test.ps1
```

The optional guarded upload command is **not part of build/test**. It must be deliberately run by the owner only after unplugging the motor/driver battery:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\upload.ps1 -MotorPowerDisconnected
```

The script builds V1, flashes over ST-Link at 100 kHz, verifies the BIN, resets, and leaves STBY LOW in READY. It will overwrite the currently working firmware, so preserve a copy/flash backup before a physical trial. Do not run it during a moving test. The project has not executed this command.

## First hardware trial, after the owner authorizes upload

1. Disconnect motor power. Connect ST-Link and upload V1. Confirm the LED reaches slow READY blink and read status if needed.
2. Remove ST-Link, place the complete body at the surveyed start pose `(300, −80) mm`, nose toward `+Y`, and clear the fixed-map course. The model has no way to infer a misplaced START.
3. Apply robot power. Keep it still through the two-second settling and two-second gyro calibration. After slow READY blink, press/release PB1. The LED blinks fast during the 3-second countdown.
4. Watch an initial wheels-raised/short floor trial before a full map attempt. PB1 during motion stops to IDLE; unplug motor battery for an immediate emergency stop.
5. At GOAL the LED is solid and motors stop. Carry it back to START, release/press PB1 for another run. A numbered fault blink remains latched until power cycling.

`tools/start_robot.ps1` and `tools/stop_robot.ps1` print these physical actions; they do not remotely drive or halt a motor-powered CPU. `tools/read_status.ps1` and `tools/read_fault.ps1` use the matching ELF symbol table, avoiding hardcoded SRAM addresses. To read a fault safely, disconnect motor power first, keep logic and ST-Link powered, then run the read script. The raw bytes can be decoded using `RobotDiagnostic` in `include/robot_types.h`.

`robot_v2` must not be uploaded for a run: it intentionally faults as unsupported. V1 cannot detect a newly placed obstacle. If the field differs from its fixed map, do not start it.
