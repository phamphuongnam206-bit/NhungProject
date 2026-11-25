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

## Verification

- `hybrid_planner_check` ARM build passed.
- ELF span is `63,940 / 64,512` bytes.
- Static RAM is `14,456` bytes and RAM including linker reservation is
  `15,992 / 20,480` bytes.
- Motor-lock host test passed for `HYBRID_MOTOR_LOCK=0` and `1`.
- Runtime host test passed with the real C core.
- Snapshot and route-view host tests passed.

The build still has the existing deprecated `boolean` warning from VL53L0X.
No firmware upload or motor test was performed.
