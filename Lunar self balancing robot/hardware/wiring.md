# Wiring (Arduino Uno / Nano)

These pins match `firmware/balance_robot/config.h`. If you change a pin there, change it here.

## MPU6050 (GY-521 board)

| MPU6050 | Arduino |
|---------|---------|
| VCC | 5 V (the GY-521 board has its own 3.3 V regulator) |
| GND | GND |
| SDA | A4 |
| SCL | A5 |
| AD0 | GND (address 0x68) |

Mount flat, **X arrow pointing forward, Z up** (otherwise see `IMU_SIGN`).

## TB6612FNG motor driver

| TB6612FNG | Connects to |
|-----------|-------------|
| VM | Battery + (via switch) |
| VCC | Arduino 5 V |
| GND | Common ground (battery −, Arduino GND) |
| STBY | D4 |
| PWMA | D5 (left motor speed) |
| AIN1 / AIN2 | D7 / D8 |
| PWMB | D6 (right motor speed) |
| BIN1 / BIN2 | D12 / D13 |
| AO1 / AO2 | Left motor terminals |
| BO1 / BO2 | Right motor terminals |

## Encoders

| Encoder pin | Arduino |
|-------------|---------|
| Left A | D2 (interrupt INT0) |
| Left B | D10 |
| Right A | D3 (interrupt INT1) |
| Right B | D11 |
| VCC / GND | 5 V / GND (check your encoder's voltage rating) |

## Overview

```
                 ┌───────────┐  I2C (A4,A5)   ┌──────────┐
 Battery ─┬────► │  Arduino  │ ◄────────────► │ MPU6050  │
 7.4 V    │      │ Uno / Nano│                └──────────┘
          │      │           │ D4,D5,D6,D7,D8,D12,D13   ┌───────────┐   ┌───────┐
          │      │           │ ───────────────────────► │ TB6612FNG │──►│Motor L│
          │      │           │                          │           │──►│Motor R│
          │      │           │ ◄── D2,D3,D10,D11 ─ encoders on motors   └───────┘
          └──────┴───────────┴── VM to driver, common GND everywhere
```

## Before powering up
1. Battery polarity correct, and the switch is off.
2. All grounds connected together.
3. No bare wire touching the chassis.
4. The first power-up should be with the wheels off the ground.
