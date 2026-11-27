# Algorithms

## Differential drive and localization

Each TIM2/TIM3 16-bit counter is differenced modulo 65536, then multiplied by the measured forward sign. For wheel `i`, `distance_i = signed_counts_i × mm_per_count_i`; the centre increment is `(distance_left + distance_right)/2`. Encoder yaw rate is `(distance_right − distance_left)/(90 mm × dt)`. Gyro Z is read at the verified 500 dps scale, its stationary mean is subtracted and the confirmed CCW sign is applied. The fused yaw rate is 98% gyro and 2% encoder while they agree within 1 rad/s; if they differ more, gyro alone governs heading. Position integrates the centre distance at the midpoint heading. Heading wraps to [−π,π]. A sustained encoder-turn/gyro-no-turn mismatch latches a fault after 500 ms.

## Wheel control

TIM1 samples each encoder at 200 Hz. Four count samples form a 20 ms speed estimate. Feedforward is `|target| / measured_gain + 0.16`; a PI correction uses Kp 0.0006 and Ki 0.003. The integral is bounded to ±0.3 and is not advanced while saturated. Duty is clamped to 140/255 and always has the target sign, so overspeed cannot make a wheel reverse. Target acceleration/deceleration are bounded at 400/600 mm/s². The motion state requests pivots explicitly. A SysTick lease disables PB0 after 150 ms without a renewed command; IWDG handles a CPU that cannot service SysTick.

## Static route and geometry

V1 uses seven fixed points in millimetres:

`(300,-80) → (425,-70) → (425,625) → (575,925) → (1600,575) → (1575,300) → (1575,-165)`.

This is an **offline waypoint route**, not a runtime A* result. The 17 baseline blocks and 600×600 mm forbidden region are encoded as rectangles. Boot preflight sweeps the whole 130×100 mm robot body with 15 mm margin along every segment at 5 mm steps and around every intermediate pivot at 5° steps. Collision uses four separating axes against axis-aligned rectangles; map edges and the legal start/finish extensions are checked separately. If any test fails, READY is withheld. The last point stops without an unnecessary pivot, with the body's front past y=−90 in the nominal geometry. The test does not include placement error or wheel slip.

The follower rotates to segment heading, waits for rotation to settle, advances with cross-track correction, brakes before the waypoint, then advances to the next segment. Host simulation found that stopping 25 mm short and cruising at 110 mm/s could cut too close to a block; V1 now stops within 10 mm and caps this unproven follower at 90 mm/s. That improvement is only simulated. Route deviation >35 mm, heading error >0.25 rad, or waypoint timeout faults. There is no dynamic obstacle detection in V1.

## V2 and planner status

A* on a bounded occupancy grid, LOS smoothing, dynamic ToF mapping and replanning are **not implemented in this independent project yet**. `robot_v2` compiles but latches `FAULT_UNSUPPORTED_PROFILE` and keeps motor power off. The group's original map and Micromouse planner were consulted for geometry and design; no Micromouse navigation module was imported. Four-sensor physical mount data, bus timing and a successful V1 hardware run are prerequisites to implement and enable V2.
