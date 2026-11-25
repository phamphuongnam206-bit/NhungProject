# Quick encoder and MPU6050 check with SWD Watch

Build the separate 64 KiB image with `pio run -e swd_quick_encoder_mpu`. Its only application source is `src/swd_quick_encoder_mpu.cpp`; the default `hybrid_2laps` environment is unchanged. This image has no Bluetooth, UART, VL53 polling, route planner or motor drive calls. PB0/STBY and all six bridge inputs/PWM pins are set LOW at the start of `setup()` and in every `loop()` call. It does not preserve the currently flashed application if the operator later chooses to program it. Back up the board first; this repository does not upload the image.

The firmware uses the assembled vehicle's TIM3 (left PA6/PA7) and TIM2 (right PA0/PA1) encoder wiring and signed directions from `hybrid_2laps.cpp`. MPU6050 uses PB7/PB6, address `0x68`, filter `0x03`, divider `0x09`, gyro range `0x08` (500 dps, 65.5 LSB/dps), and a fresh 50-sample stillness bias. The older 1000 dps bias cannot be copied. Keep the robot still during boot bias collection.

The V1 planner qualification/staging builds already use compact debug and do not call Bluetooth service. The legacy `hybrid_2laps` target still contains a read-only Bluetooth status command; this diagnostic does not build or use it. Autonomous navigation does not depend on HC-05.

## One short manual session

1. Disconnect the motor battery. Keep the drive wheels in contact with the floor, with a clear 10–30 cm straight strip and a marked 90-degree turn. After a reviewed backup and an explicitly manual upload, power the logic board and let it sit still for at least one second. Check `swd_quick.magic == 0x53574431`, `phase == 1`, `error == 0`, `mpu_who_am_i == 0x68`, and `mpu_gyro_config == 0x08` in SWD Watch. An idle debugger halt is permitted; do not move the robot while the CPU is halted.
2. Push the entire robot straight forward by hand for a measured 10–30 cm, then stop for at least 0.8 s. The first encoder pulse starts capture. `phase == 3` and `straight_done == 1` mean the signed straight counts have been latched. Do not stop for 0.8 s partway through the push; that would finish the segment early.
3. Rotate the entire robot in place by hand through a marked 90 degrees, then stop for at least 0.8 s. A gyro rate above 8 dps starts capture; a rate below 3 dps with no encoder pulses for 0.8 s finishes it. `phase == 5` and `turn_done == 1` mean the turn result has been latched. Do not move the robot again until the values are saved.
4. Enter the actual measured straight distance, in cm, into `swd_quick.measured_distance_cm` with the debugger (valid range 10–30). Resume once if the debugger was halted, then read `left_counts_per_cm` and `right_counts_per_cm`. This field is the only intended SWD write; it never controls a motor. Copy the whole `swd_quick` structure or take a screenshot before resetting or powering off. The latched DONE result tolerates a debugger halt, but a halt longer than 100 ms during a push or turn deliberately sets a sample-gap error.

For a cable too short to stay connected during the 10–30 cm push, supply the logic board independently at its normal voltage and keep it powered. After manual programming, disconnect the SWD signal cable while the board remains powered, perform both movements, then reconnect and **attach without reset** to inspect the RAM result. If the board loses power or the debug tool resets it, the RAM result is lost. Check the debugger's attach behavior before relying on this method. Never power the motor bridge for this diagnostic.

| Watch field | Interpretation |
|---|---|
| `phase`, `error` | 1 ready, 2 straight recording, 3 turn ready, 4 turn recording, 5 done, 6 error. Error 1 MPU init, 2 MPU read, 3 sampling gap. Reset the board after an error. |
| `left_delta`, `right_delta` | Latest signed firmware-direction encoder sample; positive should correspond to manual forward travel if signs are correct. |
| `straight_left`, `straight_right` | Signed encoder ticks latched at the end of the straight push. A zero or opposite sign needs wiring/sign investigation. |
| `turn_left`, `turn_right` | Signed ticks during the hand turn; expected opposite signs for an in-place turn. |
| `gyro_bias_raw`, `gyro_rate_dps` | Boot bias and current corrected gyro Z rate at the 500 dps configuration. |
| `turn_yaw_raw_deg`, `turn_yaw_hybrid_deg`, `turn_90_error_deg` | Signed integrated turn, hybrid sign convention, and absolute error from a nominal 90-degree turn. |
| `left_counts_per_cm`, `right_counts_per_cm` | Absolute straight ticks divided by the distance entered into RAM. Zero until a valid 10–30 cm distance is entered after capture. |

For a fresh attempt, write `1` to `swd_quick.command` while the code is running and hold the robot still. This clears the captured data and the entered distance. A power cycle also resets the results. Do not write to `phase`, counters, or result fields. Capture at least three straight pushes on the same ground surface before treating counts/cm as calibration evidence. Compare the marked 90-degree angle and signed yaw on several turns; a rough hand-marked turn is a polarity and gross-scale check, not a verified gyro calibration. Record board identity, distance, surface, direction, all signed counts, yaw, bias, and any error for `docs/V1_CALIBRATION_MATRIX.md`. Motor lock remains required for the competition firmware until its separate physical gates pass.
