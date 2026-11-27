# Baseline parameters and provenance

Source paths below are relative to `D:\prj_Nhung\micromouse_pcb_exact_clean`, commit `7523a715f023efa0a379a2037dfa11e7c1fee7da` plus its uncommitted PCB port changes. “Measured” means recorded by that port or the user's PCB trials; it does not mean independently remeasured for this project. The working image was observed moving, not completing the course.

| Parameter | New value | Unit | Baseline source | Status / decision |
| --- | ---: | --- | --- | --- |
| MCU clock | 72 | MHz | `src/board.cpp:23-43` | Retained |
| Left direction / PWM | PB14/PB15, PB9 TIM4 CH4 | pins | `include/config.h:21-28`, `src/board.cpp:76` | PCB confirmed; retained |
| Right direction / PWM | PB12/PB13, PB8 TIM4 CH3 | pins | `include/config.h:29-34`, `src/board.cpp:76` | PCB confirmed; retained |
| STBY / START | PB0 / PB1 | pins | `include/config.h:33-46` | PCB confirmed; retained |
| Encoders | TIM3 left PA6/7, TIM2 right PA0/1 | timer/pins | `src/board.cpp:83-86,222-225` | PCB confirmed; retained |
| PWM frequency | 20 | kHz | `src/board.cpp:160-176` | Retained |
| Encoder counts | 56.66 left, 51.24 right | count/cm | `include/config.h:92-93` | User measured; retained |
| Encoder signs | −1 left, +1 right | — | `include/config.h:97-98` | User tested; retained |
| Motor signs | −1 left, +1 right | — | `include/config.h:99-100` | User tested; retained |
| Wheel track | 90 | mm | `include/config.h:90` | Mechanical measurement; retained |
| Motor feedforward gains | 1000 left, 1085.68 right | mm/s per duty | `include/config.h:137-138` | Floor test in port; retained |
| Motor feedforward offset | 0.16 | duty | `include/config.h:138` | Retained |
| PI Kp / Ki | 0.0006 / 0.003 | duty/(mm/s), duty/(mm/s)/s | `include/config.h:141-142` | Retained; physical retune pending |
| PI correction cap | 0.3 | duty | `include/config.h:143` | Retained |
| PWM cap | 140/255 | duty | `src/motor.cpp:18` | Retained |
| MPU bus / WHO_AM_I | I2C1 PB6/PB7, 0x68 | pin/address | `src/board.cpp:89-93`, `lib/mpu6050/src/mpu6050.cpp:22` | Retained |
| MPU filter / divider / gyro range | 0x03 / 0x09 / 0x08 | register | `lib/mpu6050/src/mpu6050.cpp:53-55` | Retained |
| MPU scale / sign | 65.5 LSB/(°/s), +1 for CCW yaw | — | `include/config.h:105`, `lib/mpu6050/src/mpu6050.cpp` | Hand-turn sign confirmed; retained |
| Startup gyro calibration | 2 s settle + 400×5 ms, std <0.01, bias <0.1 | s, rad/s | `include/config.h:124-127` | Retained |
| ToF bus / XSHUT | I2C2 PB10/11; PA5/3/2 for 3 sensors | pins | `include/config.h:36-40` | Baseline uses 3 of 4; V1 excludes all |
| Fourth ToF | PA4 | pin | `PORT_NHUNG_PCB.md` | Mount/measurement not confirmed; V2 blocked |
| ToF placement | right −35.43°, front 0°, left +35.43° | degrees | `include/config.h:58-59` | Nominal, not transferred into V1 |
| Arena / blocks | 1800×1200; 17 blocks + central forbidden | mm | `include/map_data.h:1-24`; group's `tests/fixtures/simulation_map.json` | Retained geometry, field survey pending |
| Start pose | (300, −80), heading π/2 | mm, rad | `include/course_params.h:17-19` | Retained, must place physically |
| Finish gate | x 1215..1750, y < −90 | mm | `include/course_params.h:22-23` | Final V1 waypoint (1575, −165) targets full-body exit |
| Speed loop | 200 | Hz | `include/config.h:154` | Retained |
| Main control period | 20 vs baseline 33 | ms | `include/config.h:156` | Changed for simple V1; hardware timing unmeasured |
| Cruise target | max 90 vs baseline navigation scale 175 | mm/s | `include/config.h:132` | Conservative new motion setting; not a measured equivalence |
| Command lease | 150 | ms | `lib/base/include/reliability.h:10` | Retained |

The new route in `src/static_map.c` starts at the baseline pose and follows the group's previously checked corridor. It is not copied from Micromouse's A* module. Its on-board full-footprint preflight and host simulation pass; START placement error and map survey remain physical risks.
