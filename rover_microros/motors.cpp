#include "motors.h"

namespace Motors {

namespace {

constexpr int PWM_FREQ  = 1000;   // Hz — L298N is a slow BJT bridge, 1-5 kHz
constexpr int PWM_BITS  = 8;
constexpr int RAMP_STEP = 25;    // duty units per tick — higher = snappier response
constexpr int RAMP_MS   = 10;

constexpr int CH_LEFT  = 0;
constexpr int CH_RIGHT = 1;

int targetL = 0, targetR = 0;
int actualL = 0, actualR = 0;
unsigned long lastRamp = 0;

// ESP32 Arduino core 3.x dropped ledcSetup/ledcAttachPin in favour of
// ledcAttach. Support both so the sketch compiles on whichever core is
// installed.
inline void pwmSetup(int pin, int ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(ch, PWM_FREQ, PWM_BITS);
  ledcAttachPin(pin, ch);
#endif
}

inline void pwmWrite(int pin, int ch, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(ch, duty);
#endif
}

void applySide(int speed, int inA, int inB, int enPin, int enCh, float trim) {
  const bool forward = (speed >= 0);
  int duty = abs(speed);

  if (duty < DEADBAND) duty = 0;
  duty = constrain((int)(duty * trim), 0, MAX_SPEED);

  if (duty == 0) {
    // Both LOW = coast. Both HIGH would be an active brake: stops harder but
    // dumps the motor's stored energy back through the bridge.
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
  } else {
    digitalWrite(inA, forward ? HIGH : LOW);
    digitalWrite(inB, forward ? LOW  : HIGH);
  }
  pwmWrite(enPin, enCh, duty);
}

inline int stepToward(int value, int goal) {
  if (value < goal) return min(value + RAMP_STEP, goal);
  if (value > goal) return max(value - RAMP_STEP, goal);
  return value;
}

// Route logical left/right onto whichever physical channel they are wired to,
// applying the inversion flags on the way. Everything above this function
// thinks in terms of "left" and "right"; only this one knows about terminals.
void applyOutputs(int l, int r) {
  if (INVERT_LEFT)  l = -l;
  if (INVERT_RIGHT) r = -r;

  if (SWAP_SIDES) {
    // left motors on OUT3/OUT4, right motors on OUT1/OUT2
    applySide(l, PIN_IN3, PIN_IN4, PIN_ENB, CH_RIGHT, TRIM_LEFT);
    applySide(r, PIN_IN1, PIN_IN2, PIN_ENA, CH_LEFT,  TRIM_RIGHT);
  } else {
    applySide(l, PIN_IN1, PIN_IN2, PIN_ENA, CH_LEFT,  TRIM_LEFT);
    applySide(r, PIN_IN3, PIN_IN4, PIN_ENB, CH_RIGHT, TRIM_RIGHT);
  }
}

} // anonymous namespace

void begin() {
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pwmSetup(PIN_ENA, CH_LEFT);
  pwmSetup(PIN_ENB, CH_RIGHT);
  stopNow();
}

void drive(int left, int right) {
  targetL = constrain(left,  -MAX_SPEED, MAX_SPEED);
  targetR = constrain(right, -MAX_SPEED, MAX_SPEED);
}

void stopNow() {
  targetL = targetR = 0;
  actualL = actualR = 0;
  applyOutputs(0, 0);
}

void update() {
  if (millis() - lastRamp < RAMP_MS) return;
  lastRamp = millis();

  actualL = stepToward(actualL, targetL);
  actualR = stepToward(actualR, targetR);
  applyOutputs(actualL, actualR);
}

int leftOutput()  { return actualL; }
int rightOutput() { return actualR; }

} // namespace Motors
