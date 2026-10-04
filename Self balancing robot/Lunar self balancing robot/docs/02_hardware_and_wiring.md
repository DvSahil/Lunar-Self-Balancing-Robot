# 2. Hardware and wiring

The bill of materials is in `hardware/BOM.md` and the pin table in `hardware/wiring.md`.
This document explains the design choices behind them.

## Mechanical design rules that matter

1. **Centre of mass high.** A taller robot falls more slowly (time constant roughly √(l/g)), which
   gives the controller more time to react. A tiny, low robot is *harder* to balance. Put the
   battery on top.
2. **Rigid chassis.** Any flex between the IMU and the motor mounts becomes a phase lag in the
   control loop. Use screws, not tape. The IMU should be mounted firmly and close to the axle
   line.
3. **Wheels with grip.** Rubber tyres. Slipping wheels break the model completely.
4. **Backlash.** Cheap gearboxes have play. It shows up as a small steady oscillation. Higher-ratio
   gearboxes make it worse; 1:30 to 1:50 is a good range for a small robot.
5. **Mount the IMU flat** with its X arrow pointing **forward** and Z pointing **up**. If you cannot,
   change `IMU_SIGN` in `config.h` (or adapt the axis math in `imu.h`).

## Power

```
LiPo 2S (7.4 V) ──┬── switch ──┬── TB6612FNG VM  (motor supply)
                  │            └── Arduino VIN / buck converter to 5 V
```

* Motors are noisy loads. If the IMU or Arduino reboots when the motors reverse, add a 470 µF
  capacitor across VM–GND at the driver and keep the motor wires short and twisted.
* Choose a motor voltage the motors are rated for, and keep logic ground and motor ground joined
  at a single point.
* Do not power the motors from the Arduino 5 V pin.

## Why these parts

| Choice | Reason |
|--------|--------|
| Arduino Uno / Nano | Cheap, well documented, fast enough for a 200 Hz loop with float maths |
| MPU6050 | Accelerometer and gyro on one chip, I²C, very widely documented |
| TB6612FNG | More efficient than the L298N (much lower voltage drop), and logic-level compatible |
| Geared DC motors with encoders | Gearing gives torque, and encoders give speed and position for the outer loop |

## Upgrade path

* ESP32 for Bluetooth or WiFi control and a faster loop (the structure of the code ports directly)
* Stepper motors with a driver (no encoders needed, but lower speed)
* A better IMU (e.g. MPU9250 or ICM-42688)
