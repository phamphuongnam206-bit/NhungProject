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

`START_ENTRY` and `GOAL_EXIT` are **not implemented in the current locked
image**. The 128 KiB image now waits for a stable PB1 release followed by a
fresh 30 ms press before attempting planner control. PB1 is the reviewed PCB
mode-button input with pull-up; this is a software start gate, not START_ENTRY.
Its
planner GOAL gate remains unverified and prevents acceptance. The host GOAL
geometry test covers whole-body crossing only; it does not prove the vehicle
can reach or brake after the line. The current `HALF_LENGTH=6.8` and
`HALF_WIDTH=5.8` are conservative collision envelopes, not measured body
dimensions. The 15 x 10 cm rule applies to the physical body; measure it
separately. In particular, 2*5.8=11.6 cm cannot be labeled physical width
without violating the stated maximum.

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

`pc_sim/mapping.py` is a legacy 13-block simulation with guessed random
locations; it is not the V1 course source. The JSON fixture above records the
new nominal 17-block geometry. Four random-obstacle positions, surveyed
foam positions, body size, START pose, GOAL exit/braking envelope, sensor
mounts and rule interpretation for the 3-minute/4-minute timers, reset and
penalties still need confirmation. No automatic run should depend on those
unknown values. The PB1 gate has not been exercised on the physical PCB.
