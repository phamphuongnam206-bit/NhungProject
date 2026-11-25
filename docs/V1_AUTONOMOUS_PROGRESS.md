# V1 firmware progress — 2026-11-25

## Repositories and scope

- Firmware: `codex/planner-128k-v1-20261125`, baseline `6789e5b` (working tree was clean before this work).
- C11 core: `codex/course-reflectors-20261125`, `2454ca8be28c8fedff0204a62e339cdd162d5427` (clean, unchanged).
- The IDE's `CUOI_KY/NhungProject-main` is an older non-Git copy. All edits are in `D:/prj_Nhung/NhungProject_one_run`.
- Existing firmware PR: #15. Do not merge to main or change the pinned core without a separate review.

## Completed software work

- Audited the live START_ENTRY → C11 A* → FOLLOWING → stopped dynamic replan → GOAL_EXIT path and the existing host fixtures. The nominal 17-block course has a valid route from the *interior test pose* (42.5,52.5), but the blocked first corridor produces a safe no-path fault; neither is a full physical run.
- Added a synthetic boundary-to-boundary host integration case: surveyed START-entry geometry, C11 waypoint trace to interior GOAL, and full-body GOAL-exit completion. It also checks both swept fixed-obstacle corridors. This pose trace is imposed by the test and does not establish controller dynamics or real sensor response.
- Fixed P0: a terminal planner error now calls `halt("PLANNER_FAULT", Fault::CONTROL)` after invalidating the runtime. Previously the next loop could call `PlannerRuntime_Begin` again because only `plannerRuntimeReady` was cleared. `LOCKED` is latched until reset; the loop cannot consume a cached plan in this mode.
- Qualification stays `HYBRID_MOTOR_LOCK=1`, `HYBRID_PLANNER_INPUTS_VERIFIED=0`; default environment stays `hybrid_2laps`.
- `hybrid_v1_staging` is a separate build target inheriting the locked 128 KiB gates and ELF checks. It has no operating authorization; a later reviewed change must supply physical evidence before any unlock.

## Verification

| Gate | Result | Limit |
|---|---|---|
| Host C11 runtime, snapshot, START, GOAL, motion health | PASS | Synthetic fixtures; no physical pose or sensor validation |
| Python course/calibration tests | PASS, 10 tests | Nominal geometry only |
| ARM `hybrid_planner_check`, `hybrid_planner_check_128k`, `hybrid_v1_staging` and ELF checker | PASS | Locked images, no Flash write |
| Flash image span | 74,944 / 130,048 B | ELF load span; PlatformIO reports 74,632 B sections |
| SRAM | 14,848 B static; 16,384 / 20,480 B including linker reserve | 4,096 B spare is not measured stack high-water |
| Hardware, upper Flash write/retention, motor motion | NOT TESTED | Requires physical acceptance |

## Priorities and blockers

- **P0:** Upper 64 KiB Flash write, alias and power-cycle retention are unverified. Keep 64 KiB fallback. A locked 128 KiB image must not be uploaded until Flash acceptance.
- **P0:** START and GOAL surveyed coordinates, sensor mounts/FOV, ground wheel calibration, brake distance, planner timing/stack and PB0 lease on the PCB are missing. START_ENTRY/GOAL_EXIT motion constants intentionally remain zero.
- **P1:** Four possible random-obstacle positions are unknown (`NEEDS_MEASUREMENT`). Dynamic mapping is available, but traversability of all four cases is unproven.
- The four case IDs are recorded in `tests/fixtures/random_obstacle_cases.json` with null coordinates. They cannot be treated as official positions by a test or runtime.
- **P2:** DWT maxima exist for START scan, planner poll and full loop; stack high-water, separate A*/mapping/local-arc timing and interrupt nesting still need an instrumented PCB session.
- **P3:** Speed optimization waits until safe full-course completion.

## Decisions and next exact step

Use `docs/V1_CALIBRATION_MATRIX.md` as the one evidence table. Keep `include/map_one_run_calibration.h` as the central read-only entry point; values marked provisional or missing must not be promoted to `VERIFIED`. Use `docs/FLASH_128K_ACCEPTANCE.md` for the first physical gate: make and validate a full Flash backup, check the proposed test page against occupied firmware/data, then perform the controlled write/retention test manually. No software-only result can clear this blocker.
