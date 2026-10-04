# Firmware

Arduino sketch: `balance_robot/balance_robot.ino` (open that file in the Arduino IDE; the other files
open as tabs).

* Board: Arduino Uno or Nano (ATmega328P). No extra libraries; `Wire` and `EEPROM` ship with the IDE.
* Upload, open Serial Monitor at **115200 baud** with line ending **Newline**, type `help`.
* The robot boots in `idle` (motors off). Bring it up with `mode imu`, `mode enc`, `mode motor`, then `mode balance`.
  Full procedure: `docs/05_bringup_and_testing.md`.

| File | Purpose |
|------|---------|
| `config.h` | pins, sign flips, limits, filter and loop constants |
| `imu.h` | MPU6050 over I²C, gyro calibration, complementary filter |
| `motors.h` | TB6612FNG driver, dead-band compensation |
| `encoders.h` | interrupt-based quadrature counting |
| `balance_robot.ino` | modes, serial commands, PID + speed loop, telemetry, EEPROM |

## Serial commands

| Command | Meaning |
|---------|---------|
| `mode idle|imu|enc|motor|balance` | select mode |
| `x` | emergency stop, back to idle |
| `kp` `ki` `kd` `trim` `<v>` | inner loop gains and balance point |
| `vp` `vi` `<v>`, `outer 0|1` | speed loop gains and enable |
| `speed <rev/s>`, `turn <pwm>` | drive command (balance mode) |
| `mtr <l> <r>` | raw motor test, PWM -255..255 (motor mode, auto-stops) |
| `zero`, `show`, `save`, `load`, `log 0|1`, `help` | utilities |

## Telemetry format (balance mode, 20 Hz)

`t_ms,angle,rate,u,speed_rps,angle_offset,armed`

Lines beginning with `#` are messages. `tools/log_serial.py` keeps the rest as CSV.

## Status of this code

The code was syntax-checked and the control logic was reviewed, and the controller structure was
validated in `simulation/pendulum_sim.py`, but it has **not** been run on your hardware. The default gains are generic
starting points. Treat steps 1 to 4 of the bring-up procedure as mandatory, and expect to flip a sign or two.
