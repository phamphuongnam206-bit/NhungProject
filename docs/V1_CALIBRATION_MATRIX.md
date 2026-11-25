# V1 calibration evidence matrix

States: `VERIFIED` means the value has evidence for this assembled robot and the stated unit; `NEEDS_REVIEW` means a specific observation exists but is insufficient for use; `CONFLICTING` means sources disagree; `NEEDS_MEASUREMENT` means no suitable observation exists. A source-code constant alone is never calibration.

| Parameter | Current evidence | State | Needed before operation |
|---|---|---|---|
| Left counts/cm, J13 TIM3 | 59.57 inferred from two raised-wheel hand turns and approximate 13.8 cm circumference; compiled placeholder 45 | CONFLICTING | Ground distance and signed ticks, at least 3 runs |
| Right counts/cm, J12 TIM2 | 54.58 inferred the same way; compiled placeholder 45 | CONFLICTING | Same ground test |
| Left/right forward encoder signs | Hand-turn logs do not establish motor forward sign; core log's `sign=1` had zero counters | NEEDS_MEASUREMENT | Forward movement with both signed tick deltas |
| Effective wheel track | Reported 9 cm center spacing is not effective turning track | NEEDS_MEASUREMENT | Ground turning calibration |
| Body length/width | User-reported assembled 13 × 10 cm; recorded in central calibration header | NEEDS_REVIEW | Recheck assembled footprint including protrusions |
| Gyro sign | MPU yaw manual log exists; firmware sign needs matching mount and axis review | NEEDS_REVIEW | Match signed physical rotation to firmware yaw |
| Gyro bias/drift | Core `mpu6050_bias_1000dps.json` records raw bias ≈ -3.698 for a prior test | NEEDS_REVIEW | Confirm same sensor/range/temp and unit conversion; measure drift on PCB |
| Lowest stable speed / safe maximum speed | Hardcoded control limits are design settings | NEEDS_MEASUREMENT | Controlled floor test |
| Braking distance | No ground stopping measurement | NEEDS_MEASUREMENT | Distance at approved speed and supply range |
| Acceleration/deceleration | Compiled deceleration 35 cm/s² is unverified | NEEDS_MEASUREMENT | Timed floor run and stop |
| Stall, route deviation, no-progress thresholds | Source contains limits and host fault tests | NEEDS_REVIEW | Compare with measured noise, traction and loop timing |
| Four VL53 positions/angles and FOV | Nominal J8..J11 mounts in source/host fixture | NEEDS_MEASUREMENT | Measure each origin, pointing sign and response cone |
| PB1 active level | Firmware uses pull-up, active LOW, release then press; PCB test missing | NEEDS_REVIEW | Logic-level and held-at-boot test |
| START pose/region | Outside y=0, +Y expected; survey bounds and yaw tolerance absent | NEEDS_MEASUREMENT | Measure placement and repeatability |
| GOAL corridor/finish | Gate x=120..180 at y=0, -Y expected; full-body crossing required in source | NEEDS_REVIEW | Confirm competition rule, corridor and stopping envelope |
| Four random-obstacle positions | Rule/diagram says four possibilities, no coordinates verified | NEEDS_MEASUREMENT | Survey all four cases separately |

Evidence: `docs/test_results/provisional_wheel_20261124/README.md`, `docs/COURSE_LAYOUT_17.md`, `docs/PLANNER_128K_V1_QUALIFICATION.md`, core `docs/test_results/encoder_moc_2a.json` and `mpu6050_bias_1000dps.json`. The archived wheel numbers are provisional; do not copy them into operating constants. Enter reviewed values in the single `include/map_one_run_calibration.h` candidate only after recording raw logs, hardware identity, method, units and reviewer. The runtime gate must still match them against the compiled values.
