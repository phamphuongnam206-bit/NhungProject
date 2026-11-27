# Test report, 27 November 2026

`robot_v1` and `robot_v2` both compile with PlatformIO. `robot_v2` is intentionally locked and does not meet full autonomous V2 requirements. Five Zig/Clang C11 host programs pass via `powershell -ExecutionPolicy Bypass -File tools/test.ps1`:

| Test | Checked | Result |
| --- | --- | --- |
| `test_geometry` | 17 blocks, forbidden zone, body sweep, pivots, boundaries | PASS |
| `test_motion` | First-order wheel acceleration, encoder quantization, fused pose, START → GOAL route | PASS; model finish `(1574.7, −157.3)` mm |
| `test_speed_pi` | No PI-induced reversal, saturation, integrator reset | PASS |
| `test_safety` | MPU stale, left/right encoder loss, map intrusion, gyro/encoder yaw disagreement | PASS |
| `test_fsm` | Deliberate START, 3 s countdown, STOP, latched fault snapshot | PASS |

The motion simulation has ideal symmetric traction and no random gyro bias or START placement error. It does not establish a real-world clearance or arrival time. A first simulation with a 25 mm waypoint stop and 110 mm/s cap grazed the map near T9; after reducing stop distance to 10 mm and cap to 90 mm/s, the nominal simulation passed. The geometry data are nominal and must be checked against the actual course.

No new firmware has been uploaded. No motor has been run by this project. The existing flashed firmware and baseline folder remain unchanged. Hardware acceptance remains open: boot/MPU calibration without ToF, PB1 START/STOP, signs of both wheels and gyro during pivot, speed/lease behaviour, stack high-water, sensor fault, stall, floor clearance and full START → GOAL run.
