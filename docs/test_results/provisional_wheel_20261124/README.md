# Encoder wheel-scale evidence recovered from the earlier firmware snapshot

These seven JSON files were copied byte-for-byte from the local archived
`CUOI_KY/NhungProject-main/docs/test_results` snapshot. The same snapshot's
existing `tools/calibrate_wheels.py` and its four unit tests were copied into
this repository. The archive is not a Git checkout, so no source commit can be
attributed; the raw counter values and measurement method are preserved here.

| Wheel | Two raised-wheel runs | Mean ticks/rev | Inferred counts/cm |
| --- | ---: | ---: | ---: |
| J12 right, TIM2 | 2251 and 2268 ticks per 3 turns | 753.17 | 54.58 |
| J13 left, TIM3 | 2477 and 2455 ticks per 3 turns | 822.00 | 59.57 |

The inferred counts/cm divide by the user's **approximate** 13.8 cm wheel
circumference. The wheels were turned by hand while raised; no ground distance
was measured. Both summaries explicitly say
`PROVISIONAL_TWO_MANUAL_RUNS_NO_GROUND_VALIDATION`. These numbers must not be
copied to `MAP_COUNTS_CM_L/R` or marked `VERIFIED`. The J12/J13 count signs in
the raw runs do not independently establish each motor's forward direction.
The reported 9 cm center-to-center wheel spacing is not the effective turning
track; the latter remains unknown.

There is no need to repeat the hand-turn test merely to recover these values.
The missing evidence is at least three measured straight runs **on the floor**,
with actual distance and signed left/right tick changes in a UTF-8 CSV:

```csv
run,distance_cm,ticks_left,ticks_right
```

Run `python tools/calibrate_wheels.py <measured.csv>` after those runs. The tool
reports weighted estimates and run-to-run deviation, but still labels its
output `CANDIDATE_ONLY`; a person must review the raw log, wheel slip and
distance before the calibration state changes. No firmware upload or motor
command is part of this archived evidence or host tool.
