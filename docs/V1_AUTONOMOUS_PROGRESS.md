# V1 firmware progress — 2026-11-25

## Repositories and scope

- Firmware HEAD: `refs/heads/codex/planner-128k-v1-20261125` (resolve its current commit with `git rev-parse HEAD`); starting baseline `6789e5b`. Work is reviewed in PR #15.
- C11 core: `codex/course-reflectors-20261125`, `2454ca8be28c8fedff0204a62e339cdd162d5427` (clean, unchanged).
- The IDE's `CUOI_KY/NhungProject-main` is an older non-Git copy. All edits are in `D:/prj_Nhung/NhungProject_one_run`.
- Existing firmware PR: #15. Do not merge to main or change the pinned core without a separate review.

## Completed software work

- Audited the live START_ENTRY → C11 A* → FOLLOWING → stopped dynamic replan → GOAL_EXIT path and the existing host fixtures. The nominal 17-block course has a valid route from the *interior test pose* (42.5,52.5), but the blocked first corridor produces a safe no-path fault; neither is a full physical run.
- Added a synthetic boundary-to-boundary host integration case: surveyed START-entry geometry, C11 waypoint trace to interior GOAL, and full-body GOAL-exit completion. It also checks both swept fixed-obstacle corridors. This pose trace is imposed by the test and does not establish controller dynamics or real sensor response.
- Fixed P0: a terminal planner error now calls `halt("PLANNER_FAULT", Fault::CONTROL)` after invalidating the runtime. Previously the next loop could call `PlannerRuntime_Begin` again because only `plannerRuntimeReady` was cleared. `LOCKED` is latched until reset; the loop cannot consume a cached plan in this mode.
- Prevented local obstacle hits from being projected with an unverified VL53 measurement origin in the planner image. A host integration case confirms two short readings without valid origins leave the C11 map unchanged; valid timed origins still update the map and request a stopped replan. Direct front-range stopping remains active.
- GOAL_EXIT motion now rejects malformed gate bounds, body dimensions and heading tolerance before Begin, even if a caller passes `preflight_clear=true`. The final GOAL gate also rejects a tolerance wide enough to accept the opposite heading.
- The ARM ELF checker now rejects linker heap reserve below 512 B, stack reserve below 1024 B, or a changed static/RAM layout. It reports exact RAM bounds for the PCB stack test without claiming a measured high-water mark.
- Added a successful stopped-replan host case on a synthetic free-space map: a confirmed front hit blocks the original route, preview/route access are revoked while moving, and the replacement route excludes the occupied cell after stopped confirmation. The existing 17-block case still verifies safe no-path exhaustion.
- Four explicitly synthetic obstacle rectangles now stress the nominal C11 footprint planner. One case retains a safe route; three return `PLANNER_NO_PATH` and must stop. They are not the four unknown official positions; no safety margin or obstacle is removed to force a route.
- START_ENTRY and GOAL_EXIT swept-corridor checks now include the maximum lateral drift allowed by heading tolerance over the remaining straight travel, plus rotated body extent. The nominal narrow GOAL corridor rejects a synthetic 0.1 rad tolerance; smaller synthetic tolerances can pass. Physical heading stability and straightness remain `NEEDS_MEASUREMENT`.
- `docs/V1_SWD_WATCH.md` lists symbols confirmed in the locked ARM ELF for pose, mission, targets, encoder, four ranges, safety, faults and DWT timing. It does not add RAM or serial traffic.
- MPU initialization now reads back filter, sample divider and gyro full-scale registers before accepting samples. The core's archived 1000 dps raw bias is not transferable to the hybrid's 500 dps configuration; the matrix marks that mismatch `CONFLICTING`.
- The pinned, clean C11 core builds with strict host warnings and passes 12 native unit suites covering map/grid, A*, LOS/smoothing, replan, follower, wheel targets, safety, mission, sensor-map updates and START scan qualification. This validates the selected core algorithms on a host, not the PCB's timing or sensor geometry.
- Qualification stays `HYBRID_MOTOR_LOCK=1`, `HYBRID_PLANNER_INPUTS_VERIFIED=0`; default environment stays `hybrid_2laps`.
- `hybrid_v1_staging` is a separate build target inheriting the locked 128 KiB gates and ELF checks. It has no operating authorization; a later reviewed change must supply physical evidence before any unlock.

## Verification

| Gate | Result | Limit |
|---|---|---|
| Host C11 runtime, snapshot, START, GOAL, motion health | PASS | Synthetic fixtures; no physical pose or sensor validation |
| Python course/calibration/Flash-inspection tests | PASS, 13 tests | Offline inputs only |
| Core repository `pc_sim` unit tests | PASS, 7 tests | Legacy Python simulation, separate from locked firmware |
| Pinned C11 core algorithm unit suites | PASS, 12 suites | Strict host build of clean core commit `2454ca8`; no PCB timing or motor proof |
| PowerShell Flash-backup mock | PASS | Verifies two reads, paths with spaces, metadata logs and absence of reset/resume/write commands; no PCB contact |
| ARM `hybrid_planner_check`, `hybrid_planner_check_128k`, `hybrid_v1_staging` and ELF checker | PASS | Locked images, no Flash write |
| Flash image span | 76,448 / 130,048 B | ELF load span; PlatformIO reports 76,136 B sections |
| SRAM | 14,848 B static; 16,384 / 20,480 B including linker reserve | 4,096 B spare is not measured stack high-water |
| Hardware, upper Flash write/retention, motor motion | NOT TESTED | Requires physical acceptance |

## Priorities and blockers

- **P0:** Upper 64 KiB Flash write, alias and power-cycle retention are unverified. Keep 64 KiB fallback. A locked 128 KiB image must not be uploaded until Flash acceptance.
- A legacy local 128 KiB backup has non-`FF` data in the final page (`0x0801FC00`); provenance is unconfirmed. The final page is **not** an available test page by assumption. See `docs/FLASH_128K_ACCEPTANCE.md`.
- The read-only backup script now records size-register and UID logs without reset/resume, and the offline inspector reports per-page occupancy and hashes. These tools still require a fresh board readback before any write test.
- **P0:** START and GOAL surveyed coordinates, sensor mounts/FOV, ground wheel calibration, brake distance, planner timing/stack and PB0 lease on the PCB are missing. START_ENTRY/GOAL_EXIT motion constants intentionally remain zero.
- **P1:** Four possible random-obstacle positions are unknown (`NEEDS_MEASUREMENT`). Dynamic mapping is available, but traversability of all four cases is unproven.
- The four case IDs are recorded in `tests/fixtures/random_obstacle_cases.json` with null coordinates. They cannot be treated as official positions by a test or runtime.
- **P2:** DWT maxima exist for START scan, planner poll and full loop; stack high-water, separate A*/mapping/local-arc timing and interrupt nesting still need an instrumented PCB session.
- **P3:** Speed optimization waits until safe full-course completion.

## Decisions and next exact step

Use `docs/V1_CALIBRATION_MATRIX.md` as the one evidence table. Keep `include/map_one_run_calibration.h` as the central read-only entry point; values marked provisional or missing must not be promoted to `VERIFIED`. Use `docs/FLASH_128K_ACCEPTANCE.md` for the first physical gate: make and validate a full Flash backup, check the proposed test page against occupied firmware/data, then perform the controlled write/retention test manually. No software-only result can clear this blocker.
