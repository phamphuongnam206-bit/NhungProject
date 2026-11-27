# Hardware map

| Function | MCU pin / peripheral | Status |
| --- | --- | --- |
| Left motor IN1/IN2/PWM | PB14/PB15/PB9 TIM4 CH4 | Baseline PCB port |
| Right motor IN1/IN2/PWM | PB12/PB13/PB8 TIM4 CH3 | Baseline PCB port |
| TB6612 standby | PB0, active high | Baseline PCB port |
| Left encoder | PA6/PA7 TIM3 quadrature | Baseline PCB port |
| Right encoder | PA0/PA1 TIM2 quadrature | Baseline PCB port |
| MPU6050 | PB6/PB7 I2C1, 0x68 | Baseline PCB port |
| PCB START/STOP | PB1, active low | Baseline PCB port; release then press |
| LED | PC13, active low | Baseline PCB port |
| V2 ToF bus | PB10/PB11 I2C2 | Not initialized in V1 |
| V2 XSHUT | PA2/PA3/PA4/PA5 | Fourth mount unverified; not driven in V1 |
| SWD | PA13/PA14 | Kept free |

HSE 8 MHz × 9 gives 72 MHz SYSCLK; APB1 36 MHz with timer clock 72 MHz. TIM4 CH3/CH4 PWM uses 3600 timer counts, 20 kHz. TIM1 update interrupt is 200 Hz. All motors are held at zero duty with PB0 LOW during boot, calibration, READY, countdown, GOAL and FAULT.
