# References and source history

- User's moving PCB port: `D:\prj_Nhung\micromouse_pcb_exact_clean`, worktree branch `nhung_pcb_exact`, base commit `7523a715f023efa0a379a2037dfa11e7c1fee7da`, with uncommitted PCB and controller corrections. Read for hardware behaviour and measured constants; **not modified**.
- Original public reference: [justmangoou/micromouse](https://github.com/justmangoou/micromouse), MIT license. Studied board, motor, odometry, planner, safety and map concepts; no source module imported into this project.
- User's project/history: `D:\prj_Nhung\NhungProject_one_run`, especially `tests/fixtures/simulation_map.json`, and `D:\prj_Nhung\xe_tranh_vat_can_work`. The two GitHub URLs supplied for `xe_tranh_vat_can` and `mpu6050_test` were not publicly accessible during this audit; local files were used where available.
- STM32Cube HAL (`framework-stm32cubef1` installed by PlatformIO) and the STM32F103 reference manual/datasheet for clock, timer, encoder and I2C programming. The HAL is linked as an external framework, not vendored.

The source-linked measured values are tabulated in [BASELINE_PARAMETERS](BASELINE_PARAMETERS.md). Any future direct reuse of third-party source must retain its applicable license and attribution in [THIRD_PARTY_NOTICES](../THIRD_PARTY_NOTICES.md).
