# State machine

| State | Entry | Update / transition | Motor invariant |
| --- | --- | --- | --- |
| BOOT | Reset diagnostics | BSP initialization → SELF_TEST | STBY LOW |
| SELF_TEST | Check route and MPU identity/configuration | Pass → CALIBRATION; error → FAULT | LOW |
| CALIBRATION | Wait 2 s, sample 400 gyro values | Valid bias → READY; failure → FAULT | LOW |
| IDLE | Operator STOP | PB1 release/press → COUNTDOWN | LOW |
| READY | Wait for deliberate START | PB1 release/press → COUNTDOWN | LOW |
| COUNTDOWN | Reset pose to surveyed START | After 3 s, if stationary → ALIGN_HEADING; movement → IDLE | LOW |
| ALIGN_HEADING | Rotate toward current segment | Within tolerance → braking/settle → NAVIGATING | Command lease active |
| NAVIGATING | Advance with heading/cross-track correction | Near waypoint → BRAKING; fault → FAULT | Command lease active |
| BRAKING | STBY LOW; settle | Next segment → ALIGN_HEADING; last waypoint → GOAL | LOW |
| REPLANNING | Reserved for future V2 | Never entered by V1 | LOW |
| GOAL | Stop; show solid LED | After robot is manually returned to START, PB1 press → COUNTDOWN | LOW |
| FAULT | Stop, latch code and snapshot | Power cycle only; no automatic motor restart | LOW |

PB1 during motion is STOP to IDLE. The START button is the PCB button verified in the baseline port, not PB8/PB9. Release then press is required. No startup command automatically drives the car. The runtime record `g_robot_status` and frozen `g_robot_fault_snapshot` are visible over SWD; symbols are resolved from the matching ELF by the read scripts.

Fault conditions include MPU initialization/staleness, invalid route/pose, footprint violation, missing encoder counts under command, gyro/encoder yaw disagreement, motion/loop timeout, command lease expiry and IWDG reset. A sensor failure does not silently qualify READY.
