# Architecture

The application is C11. STM32Cube HAL is the external C board library. Data flows downward; navigation never touches GPIO or timer registers.

```text
main (cooperative 5/20 ms schedule) -> robot_fsm -> motion_controller -> static_map
                                    -> safety_manager    -> odometry
                                    -> mpu_driver        -> bsp
TIM1 200 Hz interrupt             -> motor_control -> speed_pi, encoder_driver -> bsp
SysTick 1 kHz interrupt           -> motor command lease -> bsp STBY
```

`RobotFsm` owns state transitions, `MotionController` owns waypoint and turn/drive phases, `Odometry` owns the pose, `MpuDriver` owns calibration and stale time, and `SafetyManager` owns motion health. The sole shared ISR/main command state is in `motor_control.c`; updates and snapshots mask interrupts briefly. The encoder driver keeps separate previous counters for speed and pose. Fixed-size arrays and structs are used throughout; no runtime allocation is requested by application code.

Startup performs blocking MPU calibration while STBY is LOW. Runtime I2C1 reads have a 3 ms timeout. TIM1 IRQ does no I2C, geometry, or route work. V1 leaves I2C2 and all XSHUT pins unused. The control loop skips missed periods and faults if elapsed time exceeds 100 ms. SysTick lowers STBY after 150 ms without a command. Hardware IWDG resets a wedged CPU; its reset flag latches a fault on the next boot.

The baseline used C++ class instances, a `Navigator` with runtime A* and pure pursuit, three ToF sensors and a 33 ms main loop. This design uses C11 state structs, a fixed route with full-footprint preflight, and a 20 ms main loop. Hardware constants were carried over; the new timing and navigation behaviour need real-car measurement.
