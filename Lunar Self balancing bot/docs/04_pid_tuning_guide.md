# 4. Control and PID tuning

## Structure: two nested loops (cascade control)

```
speed target ─► [speed PI] ─► angle offset ─┐
                                            ▼
balance trim ───────────────────────────► (+) ─► angle target
                                                    │
fused tilt ─────────────────────────────────────► (−) ─► error ─► [PID] ─► motor PWM
```

* **Inner loop (fast, 200 Hz): keeps the robot upright.** It compares tilt to the target and drives
  the wheels. This is the loop that must work first.
* **Outer loop (slow, 50 Hz): stops the robot wandering.** It measures wheel speed and *shifts the
  angle target* slightly. Too fast forward → ask for a small backward lean → the inner loop brakes
  to achieve it. (A robot cannot just "slow down": physics forces it to lean first, see doc 1.)

The outer loop must be much slower than the inner one. If both are fast they fight each other and oscillate.

## The PID, in plain terms

```
u = Kp · error  +  Ki · ∫error dt  +  Kd · tilt_rate        error = tilt − target
```

| Term | What it does | Too little | Too much |
|------|--------------|-----------|----------|
| **Kp** | pushes harder the farther it leans: the "spring" | falls over, too soft | fast shaking, large oscillation |
| **Kd** | resists fast motion: the "damper" | oscillates, overshoots | buzzing, motor whine, amplifies noise and backlash |
| **Ki** | removes a small constant lean over time | creeps in one direction | slow growing oscillation (windup) |

Safeguards already in the firmware: the integrator is clamped (`I_TERM_MAX`), the D term uses the gyro (no
noisy derivative), and a dead-band compensation (`MOTOR_DEADBAND`) skips the range where the motors only hum.

## Tuning procedure (do it in this order)

Always keep a hand on the robot, and start with the wheels **off the ground** or the robot on a stand.
Type `x` to stop the motors instantly. Use `python tools/log_serial.py` so every run is recorded.

1. **Prepare.** `outer 0`, `ki 0`, `trim 0`, set `kd` to about 0.5. Check signs first (doc 5, step 4).
2. **Measure the dead-band.** `mode motor`, send `mtr 10 10`, `mtr 15 15`, ... until the wheels just turn
   with the robot on a stand. Put that value in `MOTOR_DEADBAND` and re-upload.
3. **Kp.** In `mode balance`, hold the robot upright to arm it. Raise `kp` in steps of about 10% (`kp 20`,
   `kp 25`, ...) until it can stand for a moment but oscillates. Then reduce it by about 30%.
4. **Kd.** Raise `kd` until the oscillation is damped. Stop when you hear or feel high-frequency buzzing,
   then back off slightly. Revisit `kp` once: a bit more `kd` often allows more `kp`.
5. **Trim.** If the robot keeps creeping one way, adjust `trim` in steps of 0.2° until it stands still
   (the correct value is usually between −3° and +3°).
6. **Ki.** Add a very small `ki` (start at 1–5) only if it still drifts slowly or loses balance as the battery drains.
7. **Save.** `save` stores the gains in EEPROM. Copy the final numbers into `data/tuning_log.csv`.
8. **Outer loop.** Only now: `outer 1`. Raise `vp` until it resists being pushed along the floor, then add
   `vi` to make it return to where it started. Drive with `speed 0.3` and `turn 20`.

## Symptom guide

| What you see | Likely cause | Fix |
|--------------|--------------|-----|
| Wheels drive *away* from the direction of the fall | a sign is wrong | flip `IMU_SIGN` or `MOTOR_*_INVERT` (doc 5) |
| Falls over immediately, no reaction | `kp` too low, battery low, or disarmed | raise `kp`, charge battery, check `# ARMED` |
| Slow wobble, about 1–2 swings per second, growing | too little damping | raise `kd` or lower `kp` |
| Fast buzzing or shaking | `kd` too high, loose mechanics, dead-band too big | lower `kd`, tighten screws, re-measure dead-band |
| Stands but creeps in one direction | `trim` off | adjust `trim` |
| Stands but wanders slowly around the floor | no position feedback | enable outer loop |
| Good for a minute, then gets worse | battery voltage dropping | recharge, or retune at a lower voltage |
| `u` pinned at ±255 in the log | gains too high or motors too weak | lower gains, higher-torque motors or ratio |
| Random twitches | IMU noise or I²C errors | check wiring, raise `COMP_ALPHA`, add capacitors on the motor supply |

## Where to go next: LQR

`simulation/pendulum_sim.py` computes LQR gains for the linear model. LQR feeds back **all four states**
(position, speed, tilt, tilt rate) with gains that come from a weighting you choose (`Q` and `R`),
instead of hand tuning two nested loops. To use it on hardware you need a decent model
(doc 1) and a force-to-PWM calibration. Present it as "future work" unless you actually implement it.
