// imu.h - MPU6050 over raw I2C + complementary filter (no external library needed).
#pragma once
#include <Wire.h>
#include "config.h"

namespace imu {

const uint8_t ADDR = 0x68;                // AD0 pin low. Use 0x69 if AD0 is tied high.
const float GYRO_LSB_PER_DPS = 65.5f;     // sensitivity at the +/-500 deg/s range

float angle = 0.0f;       // fused tilt, degrees (+ = leaning forward)
float rate = 0.0f;        // tilt rate, deg/s (bias removed)
float accAngle = 0.0f;    // accelerometer-only tilt, degrees (noisy, no drift)
float gyroBiasRaw = 0.0f; // gyro offset measured at start-up, deg/s

struct Raw { int16_t ax, ay, az, gx, gy, gz; };

inline bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// Two separate statements: C++ does not define the order of two Wire.read() in one expression.
inline int16_t read16() {
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

inline bool readRaw(Raw &r) {
  Wire.beginTransmission(ADDR);
  Wire.write((uint8_t)0x3B);                       // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)ADDR, (uint8_t)14) != 14) return false;
  r.ax = read16(); r.ay = read16(); r.az = read16();
  read16();                                        // temperature (unused)
  r.gx = read16(); r.gy = read16(); r.gz = read16();
  return true;
}

inline bool begin() {
  Wire.begin();
  Wire.setClock(400000);
  if (!writeReg(0x6B, 0x01)) return false;  // wake up, clock from gyro X PLL
  writeReg(0x1A, 0x02);                     // digital low-pass filter ~94 Hz accel / ~98 Hz gyro
  writeReg(0x1B, 0x08);                     // gyro full scale +/-500 deg/s
  writeReg(0x1C, 0x00);                     // accel full scale +/-2 g
  writeReg(0x19, 0x00);                     // sample rate divider 0 -> 1 kHz internal rate
  return true;
}

// Robot must be perfectly still. Averages the gyro to find its zero offset and
// initialises the angle from the accelerometer.
inline bool calibrate(uint16_t n = 500) {
  float sum = 0.0f;
  uint16_t ok = 0;
  Raw r;
  for (uint16_t i = 0; i < n; i++) {
    if (readRaw(r)) { sum += (float)r.gy; ok++; }
    delay(2);
  }
  if (ok < n / 2) return false;
  gyroBiasRaw = sum / ok / GYRO_LSB_PER_DPS;
  if (readRaw(r)) {
    accAngle = IMU_SIGN * atan2f((float)r.ax, (float)r.az) * RAD_TO_DEG;
    angle = accAngle;
    rate = 0.0f;
  }
  return true;
}

// Call once per control loop with the real elapsed time dt (seconds).
//   angle = a * (angle + gyro * dt) + (1 - a) * accel_angle
// The gyro is smooth but drifts; the accelerometer is noisy but has no drift.
inline bool update(float dt) {
  Raw r;
  if (!readRaw(r)) return false;
  accAngle = IMU_SIGN * atan2f((float)r.ax, (float)r.az) * RAD_TO_DEG;
  rate = IMU_SIGN * ((float)r.gy / GYRO_LSB_PER_DPS - gyroBiasRaw);
  angle = COMP_ALPHA * (angle + rate * dt) + (1.0f - COMP_ALPHA) * accAngle;
  return true;
}

}  // namespace imu
