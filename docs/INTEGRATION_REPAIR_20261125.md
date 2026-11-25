# Integration repair, first checkpoint

Baseline firmware afe22b03a1f937a7006cdd81d4a60e21031b9256; core d36c524.
Both working trees were clean before changes. No earlier work overwritten.

Fixed: range_mm/valid copied for all sensors; validate every origin before
writing output; negative mounting yaw conversion; preserve legitimate STOPPED;
choosePlan requires runtime preview safety permission as well as OK status.
Motor lock remains 1; INPUTS_VERIFIED remains 0.

Actual sensor origin/time are captured at result read, with continuous 20ms
ranging. They are not exposure-aligned poses. Hardware mapping disabled pending
pose history/measurement timing integration. Synthetic timed mapping tested
through SensorMap_UpdateTimed for all four sensors and left/right direction.

Boundary-only global map was inconsistent with local MAP_BOXES. Operational
begin now refuses until a shared surveyed map/config provider is implemented.
This is a fail-closed interim fix, NOT completion of map unification.

Host runtime test: Begin -> Planning STOPPED -> Following -> confirmed obstacle
-> REPLAN_STOPPED -> wait for wheels -> replan -> Following. Safety stop retains
route and denies preview. Snapshot regression and runtime tests PASS.
These tests do not execute the complete Arduino loop.

STM32 qualification build PASS: full Flash span 60212/64512 bytes; static RAM
14448; RAM including linker reserve 15984. This locked-input build can discard
operational caller logic; it is NOT a full enabled-runtime size qualification.

Outstanding Task 1: shared map against Python, exposure-aligned origins,
full retained runtime fixture build, portable pinned core dependency (still
file:// local), caller-level loop tests. Tasks 2-5 not completed. No upload,
motor activation, merge, or hardware PASS claim. No watchdog review yet.
