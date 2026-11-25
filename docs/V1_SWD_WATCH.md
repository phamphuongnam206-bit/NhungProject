# Locked V1 SWD watch sheet

Use the exact `hybrid_planner_check_128k` or `hybrid_v1_staging` ELF built for the board. The following symbols were present in the 2026-11-25 ARM ELF; resolve names from the current ELF after each rebuild, because addresses can change. Watch expressions are read-only. Do not use a debugger expression that calls firmware functions while timing is being measured.

| What to watch | ELF symbol / debugger expression | Meaning |
|---|---|---|
| Pose | `pose.p.x`, `pose.p.y`, `pose.yaw` | cm, cm, hybrid radians (0 faces +Y) |
| Mission state and planner error | `one_run_link_storage.mission.state`, `.fault_status`, `.last_planner_status` | C11 mission and planner status codes |
| Firmware mode and fault | `mode`, `fault`, `plannerStatus` | `LOCKED` must latch after terminal fault |
| Current route | `one_run_link_storage.mission.waypoint_count`, `.waypoints`, `plannerRouteTarget` | No old target may be consumed while stopped/replanning |
| Motor request | `plan`, `commandedSpeed`, `wheelL`, `wheelR` | Diagnostic targets only in motor-locked image |
| Encoders | `totalLeft`, `totalRight`, `prevRawL`, `prevRawR` | Signed accumulated ticks and latest raw counters |
| VL53 | `frame`, `frameAt`, `frameReady`, `lastGyroGoodMs` | Four distance/time/origin records and gyro age |
| Safety output | `one_run_link_storage.output.preview.navigation.safety.reasons`, `.motion_permitted` | Core safety decision; motor lock has final authority |
| START | `startLocalizationStatus`, `startLocalizationCandidates`, `startLocalizationMeanErrorCm`, `plannerPoseVerified` | An unverified pose must leave route stopped |
| Health | `plannerHealth.latched` | Route deviation, stall, no-progress, yaw mismatch |
| Timing | `startScanMaxCycles`, `plannerPollMaxCycles`, `controlLoopMaxCycles`, `SystemCoreClock` | Convert cycles to ms with `cycles * 1000 / SystemCoreClock` |
| Physical STBY | `GPIOB->ODR & 1`, plus voltmeter/scope on PB0 | Register watch alone is not proof of pin voltage or lease timing |

The compact ARM build may optimize out human-readable `status`/`lastStop`; use the retained `fault`, `plannerStatus`, mission fault codes and physical PB0 trace. `fault` is a broad category, while `mission.fault_status` and `last_planner_status` provide planner detail. Zero timing maxima mean that phase has not run or DWT did not count, not that it took zero time. SWD reads or breakpoints can disturb loop timing; collect timing with minimal watch traffic and a separately measured trace where possible. None of these values authorize motor unlock.
