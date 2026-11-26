# V1 calibration evidence matrix

`USER_MEASURED` records a measurement from the operator or a reading captured during their test. `VERIFIED` is a separate review state required by the planner motion gate. `NEEDS_REVIEW`, `CONFLICTING` and `NEEDS_MEASUREMENT` describe remaining evidence gaps.

| Parameter | Current evidence | State | Needed before operation |
|---|---|---|---|
| Left counts/cm, J13 TIM3 | User supplied 56.66 counts/cm after three 30 cm ground pushes. SWD capture recorded +1710, +1653, +1736 ticks | USER_MEASURED | Review for motion gate |
| Right counts/cm, J12 TIM2 | User supplied 51.24 counts/cm from the same pushes. SWD capture recorded +1543, +1485, +1584 ticks | USER_MEASURED | Review for motion gate |
| Left/right forward encoder signs | Firmware multipliers −1 and +1 produced positive counts on both wheels in all three forward pushes | USER_MEASURED | Compare motor wiring direction in a separately authorized powered check |
| Effective wheel track | User supplied mechanical center spacing 90 mm. Used as provisional steering geometry | NEEDS_MEASUREMENT | Capture both encoder endpoints over one marked ground turn |
| Body length/width | User-confirmed assembled 13 × 10 cm; recorded as verified in central calibration header | VERIFIED | Recheck protrusions before physical clearance acceptance |
| Gyro sign | User reported right/left 90° turns. SWD hybrid yaw with the old −1 multiplier was −86.68°/+87.42°. New multiplier +1 makes right positive in hybrid coordinates | USER_MEASURED | Review the sign in the integrated image |
| Gyro bias/drift | Teammate core driver uses `0x1B=0x10` with 32.8 LSB/(°/s). Hybrid and SWD use the tested `0x08` with 65.5 LSB/(°/s), read it back and calculate fresh bias from 50 samples at 10 ms. The teammate driver's sample magnitude and span checks are applied in physical units to Z | NEEDS_REVIEW | Observe stationary drift during the next integrated check. Both profiles compute their own raw bias |
| Lowest stable speed / safe maximum speed | Hardcoded control limits are design settings | NEEDS_MEASUREMENT | Controlled floor test |
| Braking distance | No ground stopping measurement | NEEDS_MEASUREMENT | Distance at approved speed and supply range |
| Acceleration/deceleration | Compiled deceleration 35 cm/s² is unverified | NEEDS_MEASUREMENT | Timed floor run and stop |
| Stall, route deviation, no-progress thresholds | Source contains limits and host fault tests | NEEDS_REVIEW | Compare with measured noise, traction and loop timing |
| Four VL53 positions/angles and FOV | Core `vl53_four_range_20261124.json` confirms J8..J11 communication/ranging at addresses 0x30..0x33, but not mount geometry or distance accuracy; nominal mounts are in source/host fixture | NEEDS_MEASUREMENT | Measure each origin, pointing sign, response cone and range error; do not repeat address bring-up without a new fault |
| PB1 active level | Firmware uses pull-up, active LOW, release then press; PCB test missing | NEEDS_REVIEW | Logic-level and held-at-boot test |
| START pose/region | Outside y=0, +Y expected; survey bounds and yaw tolerance absent | NEEDS_MEASUREMENT | Measure placement and repeatability |
| GOAL corridor/finish | Gate x=120..180 at y=0, -Y expected; full-body crossing required in source | NEEDS_REVIEW | Confirm competition rule, corridor and stopping envelope |
| Four random-obstacle positions | Rule/diagram says four possibilities, no coordinates verified | NEEDS_MEASUREMENT | Survey all four cases separately |

## VL53 geometry to confirm

The nominal values below come from `BEAMS` in `src/hybrid_2laps.cpp`. Offsets are in cm from the firmware pose origin, right is positive, and angles are clockwise from forward. The team already recorded communication and ranging on these four addresses. Confirm the nominal lens position and yaw for each row or report the corrected values. Supply the usable half FOV for each sensor, or one measured value if all four share it.

| PCB | I2C | Right offset (cm) | Forward offset (cm) | Yaw | Half FOV |
|---|---|---:|---:|---:|---|
| J8 | `0x30` | −2.50 | +4.50 | −35.43° | Missing |
| J9 | `0x31` | −2.50 | +4.50 | 0° | Missing |
| J10 | `0x32` | +2.50 | +4.50 | 0° | Missing |
| J11 | `0x33` | +2.50 | +4.50 | +35.43° | Missing |

Earlier raised-wheel estimates and teammate logs are retained as provenance in `docs/test_results/provisional_wheel_20261124/README.md` and core `docs/test_results/`. Update motion review state in `include/map_one_run_calibration.h` after review. The planner still requires effective track, deceleration and stopping distance evidence.
