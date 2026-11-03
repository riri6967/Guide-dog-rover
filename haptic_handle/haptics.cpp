#include "haptics.h"

namespace Haptics {

namespace {

constexpr int PWM_FREQ = 20000;   // 20 kHz — above hearing, so the motors
                                  // hum rather than whine in the hand
constexpr int PWM_BITS = 8;
constexpr int PWM_MAX  = 255;

// Below this an ERM coin motor does not overcome its own friction; it just
// draws current and warms up. Anything weaker is better expressed as off.
constexpr int MIN_DUTY = 70;

// Full tilt in the hand is unpleasant and masks direction. Guidance sits
// well below it; only alerts go loud.
constexpr int GUIDE_DUTY = 150;
constexpr int ALERT_DUTY = 255;

// Deadbands on the incoming velocity. Below these the rover is effectively
// going straight or standing still, and the handle should stay silent.
constexpr float LINEAR_DEADBAND  = 0.03f;   // m/s
constexpr float ANGULAR_DEADBAND = 0.10f;   // rad/s

// Angular rate at which a turn is "as sharp as it gets" for feedback.
constexpr float ANGULAR_FULL = 1.00f;       // rad/s

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define HAP_SETUP(pin, ch)  ledcAttach((pin), PWM_FREQ, PWM_BITS)
  #define HAP_WRITE(pin, ch, d) ledcWrite((pin), (d))
#else
  #define HAP_SETUP(pin, ch)  do { ledcSetup((ch), PWM_FREQ, PWM_BITS); \
                                   ledcAttachPin((pin), (ch)); } while (0)
  #define HAP_WRITE(pin, ch, d) ledcWrite((ch), (d))
#endif

// LEDC channels 2..5 — the drive motors already own 0 and 1.
constexpr int CH_FRONT = 2;
constexpr int CH_BACK  = 3;
constexpr int CH_LEFT  = 4;
constexpr int CH_RIGHT = 5;

Pattern  current   = PATTERN_NONE;
float    urgency   = 0.0f;
unsigned long patternStart = 0;

// What velocity guidance most recently asked for, held so a finished
// pattern can hand control straight back without waiting for a new command.
int guideFront = 0, guideBack = 0, guideLeft = 0, guideRight = 0;

void write(int pin, int ch, int duty) {
  if (duty < MIN_DUTY) duty = 0;
  HAP_WRITE(pin, ch, constrain(duty, 0, PWM_MAX));
}

void apply(int f, int b, int l, int r) {
  write(PIN_HAP_FRONT, CH_FRONT, f);
  write(PIN_HAP_BACK,  CH_BACK,  b);
  write(PIN_HAP_LEFT,  CH_LEFT,  l);
  write(PIN_HAP_RIGHT, CH_RIGHT, r);
}

// Square wave on/off, for pulsing patterns.
bool phase(unsigned long periodMs) {
  return ((millis() - patternStart) / (periodMs / 2)) % 2 == 0;
}

} // anonymous namespace

void begin() {
  HAP_SETUP(PIN_HAP_FRONT, CH_FRONT);
  HAP_SETUP(PIN_HAP_BACK,  CH_BACK);
  HAP_SETUP(PIN_HAP_LEFT,  CH_LEFT);
  HAP_SETUP(PIN_HAP_RIGHT, CH_RIGHT);
  allOff();
}

void fromVelocity(float linear, float angular) {
  guideFront = guideBack = guideLeft = guideRight = 0;

  // Turning dominates. A person needs to know which way they are being
  // steered far more urgently than that they are moving at all, so a turn
  // silences the fore/aft cue rather than competing with it.
  if (fabs(angular) > ANGULAR_DEADBAND) {
    float sharpness = fabs(angular) / ANGULAR_FULL;
    int duty = MIN_DUTY + (int)((GUIDE_DUTY - MIN_DUTY) * constrain(sharpness, 0.0f, 1.0f));
    if (angular > 0) guideLeft  = duty;   // +z is counter-clockwise = left
    else             guideRight = duty;
  }
  else if (linear < -LINEAR_DEADBAND) {
    // Reversing is unusual and worth announcing clearly.
    guideBack = GUIDE_DUTY;
  }
  // Moving straight forward is the expected state, so it says nothing.
  // Silence is the signal that everything is fine.
}

void setPattern(Pattern p) {
  if (p == current) return;
  current = p;
  patternStart = millis();
}

void setUrgency(float u) {
  urgency = constrain(u, 0.0f, 1.0f);
}

void update() {
  switch (current) {

    case PATTERN_NONE:
      apply(guideFront, guideBack, guideLeft, guideRight);
      break;

    case PATTERN_STOP:
      // All four, hard, 5 Hz. Unmistakable and unlike anything else.
      if (phase(200)) apply(ALERT_DUTY, ALERT_DUTY, ALERT_DUTY, ALERT_DUTY);
      else            allOff();
      break;

    case PATTERN_OBSTACLE: {
      // Front only, pulsing faster as the obstacle closes — the same
      // language as a reversing sensor, which people read without training.
      unsigned long period = 600 - (unsigned long)(450 * urgency);
      if (phase(period)) apply(ALERT_DUTY, 0, 0, 0);
      else               allOff();
      break;
    }

    case PATTERN_BACKUP:
      apply(0, ALERT_DUTY, 0, 0);
      break;

    case PATTERN_LOST: {
      // Three short pulses, then a gap, repeating. Deliberately distinct
      // from STOP: this means the handle no longer knows anything, which is
      // a different and more frightening situation than "stop walking".
      unsigned long t = (millis() - patternStart) % 1600;
      bool on = (t < 120) || (t > 240 && t < 360) || (t > 480 && t < 600);
      if (on) apply(ALERT_DUTY, ALERT_DUTY, ALERT_DUTY, ALERT_DUTY);
      else    allOff();
      break;
    }

    case PATTERN_ARRIVED: {
      unsigned long t = millis() - patternStart;
      if (t < 1200) {
        bool on = (t < 300) || (t > 600 && t < 900);
        if (on) apply(GUIDE_DUTY, GUIDE_DUTY, GUIDE_DUTY, GUIDE_DUTY);
        else    allOff();
      } else {
        current = PATTERN_NONE;   // one-shot, then back to guidance
      }
      break;
    }
  }
}

void allOff() {
  apply(0, 0, 0, 0);
}

} // namespace Haptics
