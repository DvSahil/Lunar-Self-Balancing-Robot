// config.h - every pin, sign and tuning constant lives here.
// Change things HERE, not inside the other files.
#pragma once
#include <Arduino.h>

// ---------------- Boot behaviour ----------------
// false : boot into IDLE (motors off, wait for serial commands) - safest while developing
// true  : boot straight into BALANCE mode (final demo without a laptop attached)
const bool AUTOSTART_BALANCE = false;

// ---------------- Pins (Arduino Uno / Nano) ----------------
// Motor driver: TB6612FNG
const uint8_t PIN_STBY  = 4;
const uint8_t PIN_L_PWM = 5,  PIN_L_IN1 = 7,  PIN_L_IN2 = 8;
const uint8_t PIN_R_PWM = 6,  PIN_R_IN1 = 12, PIN_R_IN2 = 13;
// Encoders: channel A MUST be on an interrupt pin (D2 / D3 on Uno / Nano)
const uint8_t PIN_ENC_L_A = 2, PIN_ENC_L_B = 10;
const uint8_t PIN_ENC_R_A = 3, PIN_ENC_R_B = 11;
// MPU6050 uses the hardware I2C pins: SDA = A4, SCL = A5

// ---------------- Direction fixes ----------------
// Use bring-up steps 2 and 3 (docs/05_bringup_and_testing.md) to find out which to flip.
const bool MOTOR_L_INVERT = false;  // true if +PWM spins the left wheel backwards
const bool MOTOR_R_INVERT = false;
const bool ENC_L_INVERT   = false;  // true if pushing the robot forward makes the count fall
const bool ENC_R_INVERT   = false;
// +1.0 : IMU mounted flat, X axis pointing forward, Z axis pointing up
// -1.0 : IMU mounted rotated 180 degrees about Z (X pointing backward)
const float IMU_SIGN = 1.0f;

// ---------------- Timing and filter ----------------
const unsigned long LOOP_US = 5000;   // 5 ms -> 200 Hz control loop
const float COMP_ALPHA = 0.99f;       // complementary filter: trust in gyro (time constant ~ 0.5 s)

// ---------------- Safety ----------------
const float FALL_ANGLE_DEG = 35.0f;   // beyond this tilt the motors are cut
const float ARM_ANGLE_DEG  = 3.0f;    // must be this close to the balance point to (re)arm
const float ARM_HOLD_S     = 0.5f;    // ... for this long

// ---------------- Motor ----------------
const int   MOTOR_DEADBAND = 20;      // smallest PWM that really turns the wheels (measure it!)
const float I_TERM_MAX     = 80.0f;   // anti-windup clamp, in PWM units

// ---------------- Encoders / speed loop ----------------
// Rising edges of channel A per WHEEL revolution = encoder PPR (motor shaft) x gear ratio.
const float COUNTS_PER_REV = 11.0f * 30.0f;
const uint8_t SPEED_DIV = 4;          // speed is estimated every 4 loops (20 ms)
const float SPEED_LPF = 0.7f;         // low-pass on the speed estimate (higher = smoother, slower)
const float MAX_ANGLE_OFFSET_DEG = 4.0f;  // the speed loop may shift the balance target by this much
const float SPEED_INT_LIMIT = 10.0f;      // anti-windup for the speed integral (wheel revolutions)
const float MAX_SPEED_TARGET = 1.5f;      // rev/s
const int   MAX_TURN = 60;                // PWM difference added to / removed from each wheel

// ---------------- Misc ----------------
const unsigned long LOG_MS = 50;           // telemetry period (20 Hz)
const unsigned long MOTOR_TEST_MS = 1500;  // 'mtr' test command auto-stops after this long
const uint16_t PARAMS_MAGIC = 0xBA1A;      // marks valid gains stored in EEPROM
