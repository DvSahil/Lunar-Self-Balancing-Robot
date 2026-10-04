// encoders.h - quadrature encoders, counting rising edges of channel A.
// Channel B tells the direction. Positive count = robot moving forward.
#pragma once
#include "config.h"

namespace enc {

volatile long countL = 0;
volatile long countR = 0;

inline void isrL() { if (digitalRead(PIN_ENC_L_B)) countL++; else countL--; }
inline void isrR() { if (digitalRead(PIN_ENC_R_B)) countR++; else countR--; }

inline void begin() {
  pinMode(PIN_ENC_L_A, INPUT_PULLUP); pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP); pinMode(PIN_ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrR, RISING);
}

// A 32-bit read is not atomic on an 8-bit AVR, so block interrupts while copying.
inline void read(long &left, long &right) {
  noInterrupts();
  left = countL;
  right = countR;
  interrupts();
  if (ENC_L_INVERT) left = -left;
  if (ENC_R_INVERT) right = -right;
}

inline void zero() {
  noInterrupts();
  countL = 0;
  countR = 0;
  interrupts();
}

}  // namespace enc
