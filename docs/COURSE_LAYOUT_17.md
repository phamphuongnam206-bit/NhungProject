# Course layout from supplied T1?T17 diagram

The supplied image confirms 17 fixed obstacles and one additional random
obstacle. The coordinates used in the planner branch are the estimates already
recorded in `xe_tranh_vat_can/pc_sim/simulation_map.json`: 180 x 120 cm,
forbidden rectangle [60,0,60,60], T1?T17 rectangles. The image does not
provide surveyed coordinates; those estimates remain unverified for hardware.

The planner branch builds its 5 cm MapGrid from the same MAP_BOXES rectangles
used by the local arc footprint check. The first rectangle is forbidden; the
17 others are fixed obstacles. The random obstacle is absent initially and
must be detected/confirmed from sensor readings. Floor lines are not modeled
as ToF reflectors. Legacy hybrid_2laps retains its prior boxes in its own
compile branch. HYBRID_PLANNER_INPUTS_VERIFIED remains 0, so this map cannot
start an autonomous run without later explicit verification and valid pose.

Test `python -m unittest discover -s tests -p test_course_layout.py -v`
checks all 17 IDs and rectangle equality against the Python configuration.
Both PlatformIO environments hybrid_2laps and hybrid_planner_check build PASS.
Planner ELF Flash span 61024/64512 B; static RAM 14448 B. Tests prove code
agreement with the estimated configuration, not real course clearance or a
successful one-run route. Next: obtain actual surveyed dimensions and valid
START/GOAL/finish crossing rule before enabling motion.
