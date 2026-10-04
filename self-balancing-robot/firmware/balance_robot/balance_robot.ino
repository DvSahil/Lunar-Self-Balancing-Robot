/*
 * Self-balancing robot - Arduino Uno / Nano + MPU6050 + TB6612FNG + encoder motors
 *
 * One sketch, several modes, so you can bring the robot up one subsystem at a time:
 *
 *   mode idle     motors off (default at boot)
 *   mode imu      stream raw accelerometer angle, fused angle and rate
 *   mode enc      stream encoder counts (spin the wheels by hand)
 *   mode motor    'mtr <left> <right>' spins the wheels for 1.5 s (PWM -255..255)
 *   mode balance  the real thing: inner PID on tilt + optional outer speed loop
 *
 * Control structure (see docs/04_pid_tuning_guide.md):
 *
 *   speed target --> [speed PI] --> angle offset --+
 *                                                  v
 *   balance trim -----------------------------> (+) --> angle target
 *                                                  |
 *   fused tilt angle ---------------------------> (-) --> error --> [PID] --> motor PWM
 *
 * Sign convention: tilt + = leaning forward, u + = drive wheels forward.
 * Type 'help' in the serial monitor (115200 baud, line ending: Newline).
 */
#include <EEPROM.h>
#include "config.h"
#include "imu.h"
#include "motors.h"
#include "encoders.h"

// NOTE: no custom types in function signatures - the Arduino IDE's auto-prototype step dislikes that.
enum Mode : uint8_t { MODE_IDLE, MODE_IMU, MODE_ENC, MODE_MOTOR, MODE_BALANCE };
Mode mode = MODE_IDLE;

// Tunable gains (change live over serial, 'save' stores them in EEPROM).
// These are only STARTING POINTS - every robot needs its own values.
struct Params {
  uint16_t magic;
  float kp;     // PWM per degree of tilt error
  float ki;     // PWM per (degree * second)
  float kd;     // PWM per (degree / second)
  float vp;     // speed loop: degrees of lean per (rev/s) of speed error
  float vi;     // speed loop: degrees of lean per revolution of position error
  float trim;   // mechanical balance point, degrees
};
Params P = { PARAMS_MAGIC, 18.0f, 0.0f, 0.7f, 1.0f, 0.2f, 0.0f };

bool imuOk = false;
bool armed = false;
bool outerLoop = false;   // speed loop off until the inner loop balances well
bool logOn = true;

float armHold = 0.0f, iTerm = 0.0f, angleOffset = 0.0f, uOut = 0.0f;
float speedRps = 0.0f, speedTarget = 0.0f, speedInt = 0.0f, speedAccDt = 0.0f;
int turnCmd = 0;
long prevEncL = 0, prevEncR = 0;
uint8_t speedTick = 0;
unsigned long lastLoopUs = 0, lastLogMs = 0, motorDeadline = 0;

char lineBuf[48];
uint8_t lineLen = 0;

// ------------------------------------------------------------------ helpers
void printParams() {
  Serial.print(F("# kp=")); Serial.print(P.kp, 3);
  Serial.print(F(" ki="));  Serial.print(P.ki, 3);
  Serial.print(F(" kd="));  Serial.print(P.kd, 3);
  Serial.print(F(" vp="));  Serial.print(P.vp, 3);
  Serial.print(F(" vi="));  Serial.print(P.vi, 3);
  Serial.print(F(" trim=")); Serial.print(P.trim, 3);
  Serial.print(F(" outer=")); Serial.println(outerLoop ? F("on") : F("off"));
}

void saveParams() { EEPROM.put(0, P); Serial.println(F("# gains saved to EEPROM")); }

void loadParams() {
  Params t;
  EEPROM.get(0, t);
  if (t.magic == PARAMS_MAGIC) { P = t; Serial.println(F("# gains loaded from EEPROM")); }
}

void printHelp() {
  Serial.println(F("# mode idle|imu|enc|motor|balance   x = emergency stop"));
  Serial.println(F("# kp|ki|kd|vp|vi|trim <value>        set a gain"));
  Serial.println(F("# outer 0|1    speed loop off/on     log 0|1  telemetry off/on"));
  Serial.println(F("# speed <rev/s>  turn <pwm>  (balance mode)   mtr <l> <r> (motor mode)"));
  Serial.println(F("# zero  zero encoders   show  print gains   save | load  EEPROM"));
}

void disarm(bool say) {
  if (armed && say) Serial.println(F("# DISARMED (fell over or left balance mode)"));
  armed = false;
  armHold = 0.0f; iTerm = 0.0f; angleOffset = 0.0f; speedInt = 0.0f;
  motors::stop();
  motors::enable(false);
}

void arm() {
  armed = true;
  iTerm = 0.0f; angleOffset = 0.0f; speedInt = 0.0f; speedRps = 0.0f;
  speedTick = 0; speedAccDt = 0.0f;
  enc::read(prevEncL, prevEncR);
  motors::enable(true);
  Serial.println(F("# ARMED"));
}

void setMode(uint8_t m) {
  if ((m == MODE_IMU || m == MODE_BALANCE) && !imuOk) {
    Serial.println(F("# refused: IMU not available"));
    return;
  }
  disarm(false);
  turnCmd = 0; speedTarget = 0.0f;
  mode = (Mode)m;
  switch (mode) {
    case MODE_IDLE:    Serial.println(F("# mode idle")); break;
    case MODE_IMU:     Serial.println(F("# mode imu"));
                       Serial.println(F("t_ms,acc_angle,angle,rate")); break;
    case MODE_ENC:     Serial.println(F("# mode enc"));
                       Serial.println(F("t_ms,enc_l,enc_r")); break;
    case MODE_MOTOR:   Serial.println(F("# mode motor - use: mtr <left> <right>")); break;
    case MODE_BALANCE: Serial.println(F("# mode balance - hold the robot upright to arm"));
                       Serial.println(F("t_ms,angle,rate,u,speed_rps,angle_offset,armed")); break;
  }
}

// ------------------------------------------------------------------ serial commands
void parseCommand(char *line) {
  char *cmd = strtok(line, " ");
  if (!cmd) return;
  char *a1 = strtok(NULL, " ");
  char *a2 = strtok(NULL, " ");
  float v1 = a1 ? atof(a1) : 0.0f;
  float v2 = a2 ? atof(a2) : 0.0f;

  if (!strcmp(cmd, "mode")) {
    if (!a1) { Serial.println(F("# usage: mode idle|imu|enc|motor|balance")); return; }
    if (!strcmp(a1, "idle")) setMode(MODE_IDLE);
    else if (!strcmp(a1, "imu")) setMode(MODE_IMU);
    else if (!strcmp(a1, "enc")) setMode(MODE_ENC);
    else if (!strcmp(a1, "motor")) setMode(MODE_MOTOR);
    else if (!strcmp(a1, "balance")) setMode(MODE_BALANCE);
    else Serial.println(F("# unknown mode"));
  }
  else if (!strcmp(cmd, "x"))     { setMode(MODE_IDLE); }
  else if (!strcmp(cmd, "kp"))    { P.kp = v1;   printParams(); }
  else if (!strcmp(cmd, "ki"))    { P.ki = v1;   printParams(); }
  else if (!strcmp(cmd, "kd"))    { P.kd = v1;   printParams(); }
  else if (!strcmp(cmd, "vp"))    { P.vp = v1;   printParams(); }
  else if (!strcmp(cmd, "vi"))    { P.vi = v1;   printParams(); }
  else if (!strcmp(cmd, "trim"))  { P.trim = v1; printParams(); }
  else if (!strcmp(cmd, "outer")) { outerLoop = (v1 != 0.0f); speedInt = 0.0f; printParams(); }
  else if (!strcmp(cmd, "log"))   { logOn = (v1 != 0.0f); }
  else if (!strcmp(cmd, "speed")) {
    if (v1 > MAX_SPEED_TARGET) v1 = MAX_SPEED_TARGET;
    if (v1 < -MAX_SPEED_TARGET) v1 = -MAX_SPEED_TARGET;
    speedTarget = v1;
  }
  else if (!strcmp(cmd, "turn")) {
    int t = (int)v1;
    if (t > MAX_TURN) t = MAX_TURN;
    if (t < -MAX_TURN) t = -MAX_TURN;
    turnCmd = t;
  }
  else if (!strcmp(cmd, "mtr")) {
    if (mode != MODE_MOTOR) { Serial.println(F("# enter 'mode motor' first")); return; }
    motors::enable(true);
    motors::set((int)v1, (int)v2);
    motorDeadline = millis() + MOTOR_TEST_MS;
  }
  else if (!strcmp(cmd, "zero"))  { enc::zero(); }
  else if (!strcmp(cmd, "show"))  { printParams(); }
  else if (!strcmp(cmd, "save"))  { saveParams(); }
  else if (!strcmp(cmd, "load"))  { loadParams(); printParams(); }
  else if (!strcmp(cmd, "help"))  { printHelp(); }
  else Serial.println(F("# unknown command, type 'help'"));
}

void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) { lineBuf[lineLen] = 0; parseCommand(lineBuf); lineLen = 0; }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

// ------------------------------------------------------------------ the controller
void runBalance(float dt) {
  float a = imu::angle;
  float w = imu::rate;

  // ---- arming / fall detection
  if (armed) {
    if (fabsf(a) > FALL_ANGLE_DEG) { disarm(true); return; }
  } else {
    if (fabsf(a - P.trim) < ARM_ANGLE_DEG) {
      armHold += dt;
      if (armHold >= ARM_HOLD_S) arm();
    } else {
      armHold = 0.0f;
    }
    if (!armed) return;
  }

  // ---- outer loop: wheel speed -> small shift of the angle target (every SPEED_DIV loops)
  speedAccDt += dt;
  if (++speedTick >= SPEED_DIV) {
    long l, r;
    enc::read(l, r);
    float counts = ((float)(l - prevEncL) + (float)(r - prevEncR)) * 0.5f;
    prevEncL = l; prevEncR = r;
    float inst = counts / COUNTS_PER_REV / speedAccDt;                  // rev/s
    speedRps = SPEED_LPF * speedRps + (1.0f - SPEED_LPF) * inst;
    if (outerLoop) {
      float errV = speedRps - speedTarget;                              // + = too fast forward
      speedInt += errV * speedAccDt;                                    // = position error (rev)
      if (speedInt > SPEED_INT_LIMIT) speedInt = SPEED_INT_LIMIT;
      if (speedInt < -SPEED_INT_LIMIT) speedInt = -SPEED_INT_LIMIT;
      angleOffset = -(P.vp * errV + P.vi * speedInt);                   // too fast -> lean back
      if (angleOffset > MAX_ANGLE_OFFSET_DEG) angleOffset = MAX_ANGLE_OFFSET_DEG;
      if (angleOffset < -MAX_ANGLE_OFFSET_DEG) angleOffset = -MAX_ANGLE_OFFSET_DEG;
    } else {
      angleOffset = 0.0f;
      speedInt = 0.0f;
    }
    speedTick = 0;
    speedAccDt = 0.0f;
  }

  // ---- inner loop: PID on tilt. D acts on the gyro rate directly (no noisy differentiation).
  float err = a - (P.trim + angleOffset);
  iTerm += P.ki * err * dt;
  if (iTerm > I_TERM_MAX) iTerm = I_TERM_MAX;
  if (iTerm < -I_TERM_MAX) iTerm = -I_TERM_MAX;
  uOut = P.kp * err + iTerm + P.kd * w;
  if (uOut > 255.0f) uOut = 255.0f;
  if (uOut < -255.0f) uOut = -255.0f;

  int base = motors::withDeadband(uOut);
  motors::set(base + turnCmd, base - turnCmd);
}

// ------------------------------------------------------------------ telemetry
void logTelemetry() {
  unsigned long ms = millis();
  if (ms - lastLogMs < LOG_MS) return;
  lastLogMs = ms;
  switch (mode) {
    case MODE_IMU:
      Serial.print(ms); Serial.print(',');
      Serial.print(imu::accAngle, 2); Serial.print(',');
      Serial.print(imu::angle, 2); Serial.print(',');
      Serial.println(imu::rate, 1);
      break;
    case MODE_ENC: {
      long l, r;
      enc::read(l, r);
      Serial.print(ms); Serial.print(',');
      Serial.print(l); Serial.print(',');
      Serial.println(r);
      break;
    }
    case MODE_BALANCE:
      if (logOn) {
        Serial.print(ms); Serial.print(',');
        Serial.print(imu::angle, 2); Serial.print(',');
        Serial.print(imu::rate, 1); Serial.print(',');
        Serial.print(uOut, 0); Serial.print(',');
        Serial.print(speedRps, 2); Serial.print(',');
        Serial.print(angleOffset, 2); Serial.print(',');
        Serial.println(armed ? 1 : 0);
      }
      break;
    default:
      break;
  }
}

// ------------------------------------------------------------------ Arduino entry points
void setup() {
  Serial.begin(115200);
  motors::begin();   // motors off, driver in standby
  enc::begin();

  imuOk = imu::begin();
  if (!imuOk) {
    Serial.println(F("# ERROR: MPU6050 not found - check wiring/address. Motor and encoder tests still work."));
  } else {
    Serial.println(F("# Keep the robot still: calibrating gyro..."));
    if (imu::calibrate()) Serial.println(F("# gyro calibrated"));
    else { imuOk = false; Serial.println(F("# ERROR: IMU read failed during calibration")); }
  }
  loadParams();
  printParams();
  Serial.println(F("# type 'help' for commands"));

  lastLoopUs = micros();
  if (AUTOSTART_BALANCE && imuOk) setMode(MODE_BALANCE);
}

void loop() {
  handleSerial();

  // Safety first: the motor test command always times out, even if something else goes wrong.
  if (mode == MODE_MOTOR && millis() > motorDeadline) motors::stop();

  unsigned long now = micros();
  if (now - lastLoopUs < LOOP_US) return;
  float dt = (float)(now - lastLoopUs) * 1e-6f;
  lastLoopUs = now;

  if (imuOk && !imu::update(dt)) {            // I2C hiccup: never keep driving on stale data
    if (mode == MODE_BALANCE) disarm(true);
    return;
  }
  if (mode == MODE_BALANCE) runBalance(dt);
  logTelemetry();
}
