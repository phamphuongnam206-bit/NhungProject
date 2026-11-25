# V1 qualification: planner on the reported 128 KiB part

This is a locked qualification image, **not** a firmware approved to drive the
robot. Do not upload or run it as a motor test. The chip reported 128 KiB and
the upper addresses were readable over ST-Link, but erase/write/read and
retention after power loss have not been tested. Keep the 64 KiB build as a
fallback. Neither build proves the physical Flash above 64 KiB is writable.

## Build and memory

`pio run -e hybrid_planner_check_128k` uses the BluePill STM32F103C8 board,
sets a 130048-byte linker Flash region (128 KiB minus the final 1 KiB) and
keeps 20480 bytes of SRAM. The last Flash page remains outside the ELF image.
No EEPROM or other storage write is enabled in this image. The 64 KiB
`hybrid_planner_check` retains its 64512-byte link budget and the default
environment remains `hybrid_2laps`. The core dependency for 128 KiB is pinned
to PR #23 commit `e1898ed829205d2785ba0a136f4b6947e25778fc`.

The ELF checker validates the actual load-image span, planner symbols and
live calls into START localization and planner runtime, plus static SRAM, with
at least 4096 bytes left after the linker's reserved stack and heap. The
qualification flags remain false at startup; the live call paths cannot
authorize driving without reviewed data. This margin is not a measured stack
high-water mark. The read-only Flash test is not a substitute for a
destructive retention test.

| Environment | Build | Flash image span | Static SRAM | SRAM including linker reserve |
|---|---|---:|---:|---:|
| `hybrid_planner_check` | PASS | 61684 / 64512 B | 14784 B | 16320 / 20480 B |
| `hybrid_planner_check_128k` | PASS | 67508 / 130048 B | 14844 B | 16384 / 20480 B |

Host regression passes for the real C11 planner runtime, timed four-sensor
snapshot, and all four ranges reaching the runtime map. The fixture places
left/right hits on the matching sides and a new front hit on the route;
the latter requests a stopped replan. START adapter, route deviation, no
progress, wheel stall, rotational wheel/gyro disagreement, GOAL gate, course
geometry agreement and both motor-lock variants also pass host tests. The
fixture's distances and pose are synthetic source/build checks only.

## Safety gates and missing measurements

- `HYBRID_MOTOR_LOCK=1` and `HYBRID_PLANNER_INPUTS_VERIFIED=0`. Compile-time
  checks reject unlocking the planner qualification image. STBY stays LOW.
- The legacy motor-capable `hybrid_2laps` build has a post-link gate checking
  that the framework's strong `SysTick_Handler` calls the strong
  `osSystickHandler` lease hook and that lease arming is linked. The locked
  planner image intentionally has no motor lease hook. This checks the link,
  not physical PB0 timing under a blocked I2C bus.
- START localization from PR #23 only accepts four fresh, valid VL53 readings
  while stationary, and requires a reviewed map, START search bounds, and
  sensor mounts. The firmware adapter refuses search grids over 512 candidate
  poses or more than 18 fixed/forbidden rectangles before the expensive scan.
  This is a provisional computation cap, not an ARM timing measurement; an
  ambiguous result cannot authorize a pose.
- The planner's completion status requires a separately reviewed GOAL zone,
  correct heading, full robot footprint inside that zone, and an 80 ms interval
  without encoder pulses after the stop command. If settling takes over 500 ms
  or the zone is wrong, the control stop latches. These timing limits are
  conservative software gates; physical stopping distance still needs a
  measured test. Test coordinates are synthetic fixtures.
- Route deviation, no progress, left/right stall and rotational wheel/gyro
  disagreement latch a control stop. A safety pause or replan clears the old
  route's progress timer before monitoring the new path. Translational wheel
  slip is **not** directly observable from encoders plus gyro alone; external
  pose evidence is required to claim that feature.
- The existing A*, Pure Pursuit, dynamic sensor map, replan policy, local arc,
  independent watchdog and STBY motor lease remain in the image. Their
  timing and behavior on this PCB have not been requalified by this build.

`NEEDS_MEASUREMENT`: START position/bounds and angle, four sensor mount/FOV
angles, surveyed obstacle positions and random-obstacle test, GOAL zone and
competition finish rule, wheel/gyro calibration, route-error limit, minimum
progress and stall/yaw mismatch durations, real brake distance, planner WCET,
stack high-water mark, and Flash upper-page write/retention. Values in host
tests are fixtures; they are not copied into the robot's operating config.

## Timing measurements prepared for SWD

The 128 KiB build enables the Cortex-M3 DWT cycle counter at setup and records
`startScanMaxCycles` around the real START scan, `plannerPollMaxCycles`
around the real planner poll, and `controlLoopMaxCycles` around every complete
Arduino `loop()` call, including early returns. The START/planner maxima stay
zero until reviewed inputs allow those functions to execute. In VS Code or
another SWD debugger, watch these variables and `SystemCoreClock`; convert to
milliseconds as `cycles * 1000 / SystemCoreClock`. Check that
`CoreDebug->DEMCR` bit 24 and `DWT->CTRL` bit 0 are set. This is observation,
not an automatic timing PASS. The full-loop value includes local-arc scoring
and I2C calls on that execution path, but cannot replace testing the worst
physical sensor and replanning conditions; interrupt timing requires separate
observation.

PASS before motor unlock requires real PCB measurements demonstrating that
planner timing and total loop timing fit the motor-lease and control-loop
deadlines with margin. Zero maxima or measurements only on a host are
**unmeasured**. FAIL if DWT does not count, timing exceeds the approved budget,
or the independent STBY lease fails to cut motor power when the loop stalls.

## First on-vehicle acceptance sequence

1. Leave the motor battery disconnected. Inspect the PCB supply and measure
   PB0/STBY near 0 V while ST-Link supplies the logic rail. Do not upload this
   image until a separate physical Flash test strategy is agreed.
2. With the motor lock still on, inspect `startLocalizationStatus`,
   `startLocalizationMeanErrorCm`, `startLocalizationCandidates`,
   `plannerPoseVerified`, `plannerStatus`, `plannerGoalReached`, and
   `plannerHealth.latched` in an SWD debug session. PASS for this locked stage:
   no motor motion, STBY LOW, no unverified route or GOAL accepted.
3. Survey the START/GOAL and mount values, then review them in a separate
   change. Bench-test Flash write/retention, stack and planner runtime before
   considering any motor unlock. A build PASS is not hardware PASS.

No upload, Flash erase, motor run, or merge was performed for this change.
