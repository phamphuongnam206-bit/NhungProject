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
to core commit `2454ca8be28c8fedff0204a62e339cdd162d5427` (PR #23
START code plus the ToF-reflector classification fix).

The ELF checker validates the actual load-image span, planner symbols and
live calls into START localization and planner runtime, plus static SRAM, with
at least 4096 bytes left after the linker's reserved stack and heap. The
qualification flags remain false at startup; the live call paths cannot
authorize driving without reviewed data. This margin is not a measured stack
high-water mark. The read-only Flash test is not a substitute for a
destructive retention test.

| Environment | Build | Flash image span | Static SRAM | SRAM including linker reserve |
|---|---|---:|---:|---:|
| `hybrid_planner_check` | PASS | 61700 / 64512 B | 14784 B | 16320 / 20480 B |
| `hybrid_planner_check_128k` | PASS | 67884 / 130048 B | 14844 B | 16384 / 20480 B |

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
- The 128 KiB planner also checks `PlannerKinematicsReady` before begin:
  reviewed encoder counts/cm and signs, effective track, body dimensions,
  gyro sign and deceleration must match the compiled values, with a positive
  measured brake distance. The old 45 counts/cm, 9 cm track and 35 cm/s²
  deceleration remain unverified, so this gate currently rejects runtime.
  The user-confirmed body 13 × 10 cm is recorded separately. Calibration data
  stays in read-only Flash and the ELF check enforces that placement.
- The legacy motor-capable `hybrid_2laps` build has a post-link gate checking
  that the framework's strong `SysTick_Handler` calls the strong
  `osSystickHandler` lease hook and that lease arming is linked. The locked
  planner image intentionally has no motor lease hook. This checks the link,
  not physical PB0 timing under a blocked I2C bus.
- START localization from PR #23 only accepts four fresh, valid VL53 readings
  after a current 100 ms stable-pose window, and requires a reviewed map,
  START search bounds, and sensor mounts. The firmware adapter refuses search
  grids over 512 candidate poses or more than 18 fixed/forbidden rectangles
  before the expensive scan.
  This is a provisional computation cap, not an ARM timing measurement; an
  ambiguous result cannot authorize a pose. The pose window limits estimated
  motion; it is not an independent physical proof that the robot is still.
  Failed/ambiguous scans retry only on a new, fresh frame at least 500 ms after
  the previous attempt, so every VL53 frame cannot trigger a full scan.
  The 12 October nominal map now excludes floor-tape borders and the
  forbidden zone from START ToF raycasting while retaining their collision
  constraints. T7 is corrected to center (35,90). This is map classification
  only; it does not verify the nominal obstacle positions on the real field.
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
- The control loop rechecks planner preview permission every tick before
  reusing its cached local-arc plan. A STOPPED mission or revoked safety
  permission therefore clears that plan before the braking branch.
- The existing A*, Pure Pursuit, dynamic sensor map, replan policy, local arc,
  independent watchdog and STBY motor lease remain in the image. Their
  timing and behavior on this PCB have not been requalified by this build.
- The nominal 17-obstacle C11 A* topology test finds a route. The
  full-footprint policy rejects the near-line start as `PLANNER_UNSAFE_TURN`,
  but passes from a farther interior fixture at (42.5,52.5) cm. The
  START_ENTRY segment needed to reach that area is not qualified, so the
  whole run remains an open V1 safety blocker. The 128 KiB runtime now has a
  START handoff guard: the surveyed start region must be verified, the full
  13 x 10 cm body must be inside the x=0..60 cm gate and past y=0, yaw must
  face +Y within the reviewed tolerance, pose/ranges must be fresh, wheels
  must be stopped, and the current footprint must pass local collision checks.
  This guard does not supply a path or drive the car from behind the line;
  START_ENTRY and GOAL_EXIT motion remain unimplemented. On planner COMPLETE,
  the 128 KiB firmware stops, waits for quiet encoder pulses, and calls the
  GOAL_EXIT corridor checker over the fixed map and live obstacle hits. It
  distinguishes missing measurements, unsettled sensors and blocked geometry.
  The Flash-resident terminal Y and brake overrun are both zero
  (`NEEDS_MEASUREMENT`), so the preflight rejects the current image. Even a
  clear preflight halts with `GOAL_EXIT_CONTROL_PENDING`; it never commands
  exit motion or marks FINISHED. The ELF checker requires those slots to stay
  in read-only Flash, preserving the 4 KiB RAM reserve.
  The PB1
  start-button software gate now requires release then press, but has no
  physical PCB test; see `COURSE_LAYOUT_17.md`.
- A host integration test now feeds the actual 17-block fixture through
  `PlannerSnapshot_Build` and the real C11 runtime. From the interior fixture
  (42.5,52.5), it plans four waypoints to the interior GOAL approach. A
  synthetic timed 20 cm front VL53 hit on the narrow first corridor is mapped
  after two confirmations. The mission stops, revokes preview permission,
  attempts two replans, finds no safe alternate and latches
  `ONE_RUN_REPLAN_EXHAUSTED`. This is a PASS for safe stopping, **not** proof
  that the unknown random-obstacle positions are traversable or that the
  physical sensors detect them reliably.

`NEEDS_MEASUREMENT`: START position/bounds and angle, four sensor mount/FOV
angles, surveyed obstacle positions and random-obstacle test, GOAL zone and
competition finish rule, wheel/gyro calibration, route-error limit, minimum
progress and stall/yaw mismatch durations, real brake distance, planner WCET,
stack high-water mark, and Flash upper-page write/retention. Values in host
tests are fixtures; they are not copied into the robot's operating config.
The user-corrected 13 x 10 cm overall dimensions are used in the collision
footprint; they have not been independently checked on the assembled vehicle.

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
