// motors.h - TB6612FNG dual motor driver.
#pragma once
#include "config.h"

namespace motors {

inline void begin() {
  const uint8_t pins[] = {PIN_STBY, PIN_L_PWM, PIN_L_IN1, PIN_L_IN2, PIN_R_PWM, PIN_R_IN1, PIN_R_IN2};
  for (uint8_t i = 0; i < sizeof(pins); i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);        // everything off at boot
  }
}

inline void enable(bool on) { digitalWrite(PIN_STBY, on ? HIGH : LOW); }

// cmd in [-255, 255]; positive = forward
inline void driveOne(uint8_t pwmPin, uint8_t in1, uint8_t in2, int cmd, bool invert) {
  if (invert) cmd = -cmd;
  if (cmd > 255) cmd = 255;
  if (cmd < -255) cmd = -255;
  if (cmd > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (cmd < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else              { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); }   // coast
  analogWrite(pwmPin, cmd < 0 ? -cmd : cmd);
}

inline void set(int left, int right) {
  driveOne(PIN_L_PWM, PIN_L_IN1, PIN_L_IN2, left,  MOTOR_L_INVERT);
  driveOne(PIN_R_PWM, PIN_R_IN1, PIN_R_IN2, right, MOTOR_R_INVERT);
}

inline void stop() { set(0, 0); }

// Controller output u (-255..255) -> PWM that skips the dead zone where the motors only hum.
inline int withDeadband(float u) {
  float m = fabsf(u);
  if (m < 1.0f) return 0;
  int pwm = MOTOR_DEADBAND + (int)(m * (255 - MOTOR_DEADBAND) / 255.0f);
  if (pwm > 255) pwm = 255;
  return u > 0 ? pwm : -pwm;
}

}  // namespace motors
