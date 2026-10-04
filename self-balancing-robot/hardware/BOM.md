# Bill of materials

Fill in the **Actual part** column with what you really bought or used, so the README matches your build.

| Qty | Part | Typical choice | Actual part | Notes |
|-----|------|----------------|-------------|-------|
| 1 | Microcontroller | Arduino Uno or Nano (ATmega328P) | | Encoder channel A needs interrupt pins D2/D3 |
| 1 | IMU | MPU6050 breakout (GY-521) | | I²C address 0x68 |
| 1 | Motor driver | TB6612FNG breakout | | Dual H-bridge, 1.2 A continuous per channel |
| 2 | Geared DC motors with encoders | 6–12 V, about 1:30 to 1:50 gearbox, quadrature encoder | | Check stall current against the driver rating |
| 2 | Wheels | 60–70 mm with rubber tyres | | |
| 1 | Battery | 2S LiPo 7.4 V (or 3S for more torque) | | Use a proper charger |
| 1 | 5 V regulator | Buck converter, or the Arduino's own regulator | | Do not run the motors from Arduino 5 V |
| 1 | Capacitor | 470 µF electrolytic across the motor supply | | Reduces noise and resets |
| 1 | Switch | Power switch | | |
| 1 | Chassis | Laser-cut acrylic / 3D-printed / plywood | | Rigid, with room for a high battery |
| – | Wires, headers, screws, standoffs | | | |
| 1 | (optional) Bluetooth module | HC-05 / HC-06 | | Connect to the serial pins to send the same text commands |

## Measured data (fill in)

| Quantity | Value |
|----------|-------|
| Body mass `m` | |
| Wheel + motor mass `M` | |
| CoM height `l` | |
| Wheel radius | |
| Encoder counts per wheel revolution | |
| Motor dead-band (PWM) | |
| Battery voltage (full / low) | |
