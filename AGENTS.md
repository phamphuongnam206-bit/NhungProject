# Project working rules

- Keep this project independent. Do not import source files, symlinks, or build outputs from neighboring projects.
- Preserve the verified PCB pinout and measured wheel/motor parameters in `include/robot_config.h` unless new hardware evidence warrants a change.
- Do not upload or energize the robot without the user's instruction. The guarded upload script requires motor power to be disconnected.
- Keep `robot_v2` motor-locked until four-sensor integration and physical V1 acceptance are complete. Do not label its current successful compilation as a working V2 run.
- Run `tools/test.ps1` and both PlatformIO builds after firmware edits. Report actual Flash/RAM use and remaining physical test gaps.
- Preserve attribution for any external code or data. Do not disguise copied code.
