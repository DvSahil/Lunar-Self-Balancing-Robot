# 3. Sensing: getting a clean tilt angle

The controller needs two things: the tilt **angle** θ and the tilt **rate** θ'. One IMU gives both,
but neither sensor on its own is good enough.

| Sensor | Gives | Good | Bad |
|--------|-------|------|-----|
| Accelerometer | angle from the direction of gravity: `atan2(ax, az)` | no drift | noisy; the robot's own acceleration looks like tilt |
| Gyroscope | rotation *rate*; integrate it for an angle | smooth, fast | has a small offset (bias), so the integrated angle drifts |

## Complementary filter

```
angle = α · (angle + gyro · dt) + (1 − α) · accel_angle        α = 0.99
```

Short term (< 0.5 s) the **gyro** dominates, so you get a smooth, fast angle. Long term
the **accelerometer** slowly pulls the angle back to the truth, which cancels drift. The crossover
time constant is `τ = α·dt / (1 − α)` ≈ 0.5 s at α = 0.99 and dt = 5 ms.

Trade-off: a *larger* α rejects the accelerometer's vibration noise better, but takes longer to
correct a wrong starting angle. If your robot jitters at the motor frequency, raise α. If the
angle drifts after a hard push, lower it a little.

(A Kalman filter or the Mahony/Madgwick filters do the same job more elegantly. For a robot that
only tilts about one axis, the complementary filter is just as good and much easier to explain.)

## Gyro bias calibration

At start-up the robot sits still for one second while 500 gyro readings are averaged. That average is
the zero offset and is subtracted from every later reading. If you skip this, the angle drifts by
several degrees per minute and the robot slowly wanders. **Keep the robot still while the
serial monitor prints "calibrating gyro".**

## Why use the gyro directly for the D term

The D term of the PID needs θ'. The firmware uses the measured **gyro rate** rather than
differentiating the angle. Differentiating a noisy signal amplifies the noise; the gyro already
*is* the derivative, measured cleanly.

## The balance-point offset (`trim`)

The mechanical balance point is rarely exactly 0°, because the centre of mass is not perfectly above
the axle. If the robot always creeps in one direction, the angle where it is truly in equilibrium
is not zero. Find it by holding the robot at the angle where it feels neutral, read the angle in
`mode imu`, and set `trim` to that value (`trim 1.2`, then `save`).

## Checks in `mode imu`

* Tilt the robot forward: `acc_angle` and `angle` should both **increase**. If not, change `IMU_SIGN`.
* Hold still: `angle` should settle within about ±0.3° and `rate` within about ±1°/s. If `rate` sits far from
  zero, calibration happened while the robot was moving.
* Shake the robot lightly: `acc_angle` jumps around while `angle` stays smooth. That is the filter doing
  its job, and a nice plot for your README.
