# 5. Bring-up, testing and validation

Test one subsystem at a time. When something fails you then know exactly which part is responsible.
Use the serial monitor (115200 baud, line ending **Newline**) or `python tools/log_serial.py`.

## Safety

* Wheels **off the ground** (robot on a stand or held up) for every test except the final balance runs.
* Type `x` to stop everything. The `mtr` command times out by itself after 1.5 s.
* LiPo batteries: use a proper charger, never short them, and do not leave them charging unattended.
* Check polarity before connecting the battery: reversed power destroys the driver and the Arduino.

## Step 0: power-off wiring check
Compare with `hardware/wiring.md`. Common grounds joined, battery polarity right, no loose strands.

## Step 1: IMU (`mode imu`)
Expected: `# gyro calibrated`, then lines `t_ms,acc_angle,angle,rate`.
* Tilt the robot forward → both angles must go **up**. If they go down, set `IMU_SIGN = -1.0f`.
* Hold still → `rate` near 0 (about ±1°/s), `angle` steady.
* Tap the chassis → `acc_angle` is spiky, `angle` stays smooth. Save a plot for the README.

## Step 2: encoders (`mode enc`)
* Spin each wheel **forward** by hand → its count must increase. If it decreases, flip `ENC_L_INVERT` / `ENC_R_INVERT`.
* Turn a wheel exactly one revolution (mark it with tape), `zero` first. The count should be close to `COUNTS_PER_REV`.
  If it is not, correct `COUNTS_PER_REV` in `config.h`.

## Step 3: motors (`mode motor`)
* `mtr 80 0`: only the **left** wheel should turn, **forward**. Then `mtr 0 80` for the right.
* Wheel turns backward → flip `MOTOR_L_INVERT` / `MOTOR_R_INVERT`. Wrong wheel turns → swap the pins in `config.h`.

## Step 4: sign check in balance mode
Robot held with wheels off the ground, `kp 18 ki 0 kd 0.7`, `mode balance`, then hold it near upright until `# ARMED`.
* Tilt the top **forward** → both wheels must spin **forward**.
* Tilt **backward** → wheels spin backward.
If this is wrong the robot will fall over immediately and violently. Fix signs before putting it on the floor.

## Step 5: first balance
Follow doc 4. Record each attempt in `data/tuning_log.csv` (gains, what happened).

## Step 6: add the speed loop
`outer 1`, tune `vp` then `vi`, test `speed` and `turn`.

## Step 7: validation experiments
Run each at least 3 times with a fresh battery, log with `log_serial.py`, summarise with `plot_log.py`,
and write the numbers into the README results table and `data/experiment_log.md`.

| Experiment | How | What to record |
|------------|-----|----------------|
| Stand still | Let it balance on the floor | RMS tilt, how long until it falls or you stop it |
| Push recovery | Tap the top with a finger, harder each time | largest push survived, peak tilt, recovery time |
| Drift | `outer 0` vs `outer 1`, 30 s | distance wandered (mark the floor) |
| Surface test | Table, carpet, slight slope | does it still balance, which gains were needed |
| Battery | Run from full to low | time until performance degrades |
| Speed / turn | `speed 0.5`, `turn 25` | does it hold balance, steady-state speed |

Be honest in the write-up. A robot that balances for 30 s with measured numbers and an explanation of the
failure modes is a stronger project than one with claims and no data.
