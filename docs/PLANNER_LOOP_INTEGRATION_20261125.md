# Planner loop integration

`src/hybrid_2laps.cpp` now connects the locked diagnostic loop to
`PlannerSnapshot_Build`, `PlannerRuntime_Begin`, `PlannerRuntime_Poll`, and
`PlannerRuntime_Invalidate` when `HYBRID_PLANNER_ROUTE=1`.

The snapshot uses encoder, valid gyro, and per-sensor VL53 timestamps from the
existing drivers. The hybrid yaw convention is converted by the snapshot
module. Invalid, stale, future, or skewed samples invalidate the runtime and
leave the route stopped.

`HYBRID_PLANNER_INPUTS_VERIFIED` defaults to `0`. The loop therefore does not
authorize a route from `MAP_START`, guessed calibration, or the placeholder
map. Setting this flag requires a reviewed course map and verified pose and
calibration. The motor lock remains required by the planner-route build.

When the runtime reports `ONE_RUN_COMPLETE`, the hybrid loop clears its motion
target, applies the locked stop path, and enters `FINISHED`. The old fixed-route
finish test remains disabled for planner mode until the official finish gate
and competition rule are confirmed.

The hybrid loop now starts the STM32F1 independent watchdog with a nominal
4-second timeout and reloads it at the start of every loop. A control-loop gap
over 150 ms latches `CONTROL_LOOP_TIMEOUT` and stops the vehicle. These paths
are source/build verified; reset timing still needs a controlled PCB test.

The core mapping API is pinned to local checkout commit `d36c524`. The loop
passes the pose captured with each VL53 sample to `SensorMap_UpdateTimed`, so
confirmed hits can update the runtime occupancy map without reprojecting old
measurements from the latest pose.

## Verification

- `hybrid_planner_check` ARM build passed.
- ELF span is `64,428 / 64,512` bytes.
- Static RAM is `14,508` bytes and RAM including linker reservation is
  `16,048 / 20,480` bytes.
- Motor-lock host test passed for `HYBRID_MOTOR_LOCK=0` and `1`.
- Runtime host test passed with the real C core.
- Snapshot and route-view host tests passed.

The build still has the existing deprecated `boolean` warning from VL53L0X.
No firmware upload or motor test was performed.
