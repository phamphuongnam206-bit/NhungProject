# Nominal course geometry, 12 October 2026 rules

Coordinate origin: lower left of the 180 x 120 cm arena; +X right, +Y up.
The nominal 10 x 10 cm foam block centers T1..T17 and the [60,0,60,60] cm
forbidden rectangle are in `tests/fixtures/simulation_map.json`. These are
diagram coordinates, **not** surveyed positions. T7 is centered at (35,90),
correcting the earlier (35,85) firmware entry. The planner build uses the
same `MAP_BOXES` rectangles for the 5 cm C11 grid and hybrid local collision
check. The 64 KiB fallback retains its old hybrid route and obstacle list.

The arena outline and forbidden rectangle are floor tape: collision/planning
constraints, not ToF targets. The 128 KiB START scan passes all 18 rectangles
for collision, skips rectangle 0 (forbidden zone) for raycasting, and disables
arena-edge echoes. The 17 foam blocks alone are modeled as reflectors. Real
ToF response still depends on material, angle, lighting and measured mounts.
One random obstacle may appear at one of four locations, but their coordinates
are unknown and are **not** in the static map. Fresh VL53 detections remain
the only source of dynamic-map updates and replanning for that obstacle.

## START and GOAL boundary

The C11 `MapGrid_WorldToCell` rejects negative Y; its 24 x 36 cells cover
0 <= Y < 120, 0 <= X < 180. Its border cells are blocked for planning. Thus
an external START pose or a GOAL pose with the entire body beyond Y=0 cannot
be represented as a planner waypoint. Do not enlarge the grid merely for
these short boundary movements.

V1 needs separate guarded states before motor unlock:

1. `START_ENTRY`: button press, surveyed initial pose, body behind the
   x=0..60 gate, heading +Y; only a short collision-checked and sensor-checked
   entry is allowed until the whole body passes Y=0. Then start C11 planning
   from an interior free cell. No hardcoded `MAP_START` authorizes pose.
2. `FOLLOWING`: C11 A*, Pure Pursuit, dynamic map/replan and local arc operate
   inside the field. An interior GOAL approach point is not a finish.
3. `GOAL_EXIT`: after an approved interior approach, continue through the
   x=120..180 gate heading -Y with full-body collision checks. Confirm
   `y + projected_half_length < 0` and the whole body between X=120..180,
   then require stopped encoder pulses before `FINISHED`.

`START_ENTRY` motion is **not implemented in the current locked image**.
`PlannerGoalExit_CorridorClear` checks a straight exit from a verified
interior pose, the full body across y=0, a reviewed brake runout and
 intersections with fixed or dynamic obstacles. The 128 KiB firmware calls it
 after the planner completes and the wheels settle. A separate straight-exit
 state machine then checks the live corridor and only accepts FINISHED when
 the whole body crosses the line and the wheels settle. Terminal Y, brake
 runout, speed and time limits in Flash are still zero, so the current image
 rejects GOAL_EXIT before motion. Host test values (including 2 cm brake
 runout) are synthetic. The 128 KiB image now waits for a stable PB1 release
followed by a
fresh 30 ms press before attempting planner control. PB1 is the reviewed PCB
mode-button input with pull-up; this is a software start gate, not START_ENTRY.
Its planner GOAL gate remains unverified and prevents acceptance. The host GOAL
geometry test covers whole-body crossing only; it does not prove the vehicle
can reach or brake after the line. The user corrected the overall dimensions
to 13 cm along travel and 10 cm across both wheels. Thus the firmware uses
`HALF_LENGTH=6.5` and `HALF_WIDTH=5.0`; the 1.5 cm obstacle margin is applied
separately. The user also reports a 15 cm gap between adjacent blocks, without
identifying the particular pair. These dimensions match the stated 15 x 10 cm
maximum on paper but have not been independently verified on the assembled
vehicle. The motor lock remains on.

## Software qualification

`python -m unittest discover -s tests -p test_course_layout.py -v` checks
all 17 nominal blocks, forbidden-zone agreement and ToF classification.
`python tools/check_course_route.py --core <checkout> --zig <zig>` runs real
C11 planning on those 18 rectangles. Topological A* finds a path from the
near-line interior cell (32.5,12.5) to GOAL approach (162.5,12.5), but the
full-footprint planner rejects that direct route as `PLANNER_UNSAFE_TURN` (6).
Its first raw diagonal step lacks body clearance near the lower boundary.
The same full-footprint policy **passes** from a farther interior test cell
(42.5,52.5) to (162.5,12.5) with four waypoints and a downward final leg.
The farther cell is a host fixture, not an operational START waypoint. This
proves only that the interior course segment has a valid nominal path. The
full START_ENTRY from behind the line and GOAL_EXIT beyond it remain untested;
motor lock stays on. Do not remove footprint checks to force a direct route.

`tests/planner_course_runtime_test.cpp` exercises the real snapshot adapter,
17-block map and C11 mission on one synthetic J9 hit near x≈40, y≈77.
That hit closes the narrow first corridor; the runtime stops, keeps preview
motion disabled and faults after two unsuccessful replans. This scenario does
not identify any of the four unknown random-obstacle locations. A dynamic
obstacle can make the nominal course unsolvable, so V1 must retain its safe
stop when no alternate path exists.

The apparent straight corridors have almost no surveyed tolerance. At
X=42.5 cm through T2/T3/T4 and X=162.5 cm beside T17, the nearest nominal
block face is 7.5 cm from the tested centerline. The remaining lateral
clearance is `7.5 - 10/2 - 1.5 = 1.0 cm` on that side. For a centered
15 cm gap, this same calculation applies to both sides. This uses the
nominal diagram and user-provided dimensions, not a surveyed safety margin.
START_ENTRY and GOAL_EXIT still need a measured path and starting pose before
motion is authorized.

`python -m unittest discover -s tests -p 'test_course_*.py' -v` also checks
straight, axis-aligned START and GOAL gate sweeps against all 17 nominal
blocks and the forbidden zone. The 10 cm body width fits the **nominal**
centerlines at X=42.5 and X=162.5 with 1.0 cm lateral remainder after the
existing obstacle margin. Shifting either path 1.1 cm toward its nearest
block, or inserting a small dynamic
block, makes the test reject that passage. This is a host geometry test only:
it does not establish pose, heading, VL53 freshness, braking distance, or
physical block placement, and does not activate START_ENTRY or GOAL_EXIT.

`pc_sim/mapping.py` is a legacy 13-block simulation with guessed random
locations; it is not the V1 course source. The JSON fixture above records the
new nominal 17-block geometry. Four random-obstacle positions, surveyed
foam positions, body size, START pose, GOAL exit/braking envelope, sensor
mounts and rule interpretation for the 3-minute/4-minute timers, reset and
penalties still need confirmation. No automatic run should depend on those
unknown values. The PB1 gate has not been exercised on the physical PCB.
