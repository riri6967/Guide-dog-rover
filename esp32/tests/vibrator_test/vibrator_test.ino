/*
 * vibrator_test — bench test for the four handle vibration motors.
 *
 * No ROS, no WiFi, no motor driver. Just the ESP32, the four vibrator
 * modules and the serial monitor. Flash this, open the monitor, press keys.
 *
 * It answers the four questions you cannot answer by looking:
 *
 *   1. Is every module wired to the pin you think it is?   -> 1 2 3 4
 *   2. Is IN active HIGH or active LOW?                    -> i
 *   3. Does PWM vary the strength, or is it on/off only?   -> p
 *   4. What is the lowest duty that reliably starts it?    -> d
 *
 * That last one matters as much as it did for the L298N. An ERM motor has
 * a start-up threshold — below it the motor buzzes faintly or not at all,
 * and a "gentle" guidance cue that does nothing is worse than no cue.
 *
 *   Board:        ESP32 Dev Module
 *   Monitor:      115200 baud, line ending "No line ending"
 *   Power:        modules from the MP1584 5V rail, ESP32 from USB.
 *                 Module GND must reach ESP32 GND.
 */
#include <Arduino.h>

// ------------------------------------------------------------------ pins
// Same as PIN_HAP_* in esp32/rover_microros/pins.h. 21/22 are free for this
// because the MPU6050 is on the Raspberry Pi's I2C, not the ESP32's. Avoiding
// 12 (held HIGH at reset the board will not boot) and 16/17 (not broken out
// on this DevKit).
const int PIN_FRONT = 23;
const int PIN_BACK  = 32;
const int PIN_LEFT  = 21;
const int PIN_RIGHT = 22;

const int PINS[4]   = { PIN_FRONT, PIN_BACK, PIN_LEFT, PIN_RIGHT };
const char *NAMES[4] = { "FRONT", "BACK ", "LEFT ", "RIGHT" };

// ------------------------------------------------------------------ pwm
// 1 kHz: high enough not to growl audibly, low enough that the module's
// transistor switches it cleanly. Channels 4-7 leave 0-3 free for the
// drive motors, so this sketch's numbering matches the real firmware.
const int PWM_FREQ = 1000;
const int PWM_BITS = 8;
const int PWM_MAX  = 255;
const int CH[4]    = { 4, 5, 6, 7 };

// ------------------------------------------------------------------ state
int  intensity = 200;        // 0..255, the level 1/2/3/4 and the demos use
bool activeLow = false;      // flipped by 'i' if the modules idle ON
int  deadband  = 0;          // discovered by 'd', printed at the end

// --------------------------------------------------------------- hardware

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

// One place that knows about polarity. Everything else asks for "strength
// 0..255" and this decides what the pin actually has to do.
void setMotor(int i, int duty) {
  duty = constrain(duty, 0, PWM_MAX);
  pwmWrite(PINS[i], CH[i], activeLow ? (PWM_MAX - duty) : duty);
}

void allOff() {
  for (int i = 0; i < 4; i++) setMotor(i, 0);
}

// Wait, but abort early if a key arrives. Every long-running test uses this
// so you are never stuck waiting for a sequence you have seen enough of.
bool waitOrKey(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (Serial.available()) return true;
    delay(5);
  }
  return false;
}

// ------------------------------------------------------------------ tests

void pulse(int i) {
  Serial.printf("  %s  duty %d\n", NAMES[i], intensity);
  setMotor(i, intensity);
  waitOrKey(800);
  setMotor(i, 0);
}

void sequence() {
  Serial.println("\nSequence — front, back, left, right.");
  Serial.println("Touch each module as it runs and check the name matches.\n");
  for (int i = 0; i < 4; i++) {
    pulse(i);
    if (waitOrKey(400)) break;
  }
  Serial.println("\nAny that stayed silent are a wiring or threshold problem.");
}

// Does duty cycle actually change the strength? Hold a module between finger
// and thumb through this — if it steps up in strength, PWM works and the
// guidance code can scale intensity with how sharply the rover is turning.
// If it is simply off then full-on, the module has a driver IC that only
// does on/off and the firmware falls back to pulse patterns.
void rampTest() {
  Serial.println("\nPWM ramp on FRONT, 0 -> 255 -> 0.");
  Serial.println("Hold the module. Feel for steps, not just on/off.\n");

  for (int d = 0; d <= PWM_MAX; d += 15) {
    Serial.printf("  duty %3d\n", d);
    setMotor(0, d);
    if (waitOrKey(350)) { setMotor(0, 0); return; }
  }
  for (int d = PWM_MAX; d >= 0; d -= 15) {
    setMotor(0, d);
    if (waitOrKey(120)) { setMotor(0, 0); return; }
  }
  setMotor(0, 0);
  Serial.println("\nStrength varied smoothly? PWM works — intensity is usable.");
  Serial.println("Jumped straight from off to full? On/off only. Tell me and");
  Serial.println("the firmware will use pulse rate instead of strength.");
}

// The haptic equivalent of the L298N deadband. Steps up slowly from zero;
// press any key the instant you feel it start. Anything below that number
// is a cue the person will not notice.
void findDeadband() {
  Serial.println("\nThreshold hunt on FRONT. Hold the module lightly.");
  Serial.println("Press any key THE MOMENT you feel it start.\n");

  while (Serial.available()) Serial.read();   // clear anything pending

  for (int d = 5; d <= PWM_MAX; d += 5) {
    Serial.printf("  duty %3d\n", d);
    setMotor(0, d);
    if (waitOrKey(1200)) {
      while (Serial.available()) Serial.read();
      setMotor(0, 0);
      deadband = d;
      Serial.printf("\n  Start threshold: %d  (%d%% duty)\n", d, (d * 100) / PWM_MAX);
      Serial.println("  Guidance cues must sit above this. Tell me the number");
      Serial.println("  and it goes into haptics.cpp as HAPTIC_DEADBAND.");
      return;
    }
  }
  setMotor(0, 0);
  Serial.println("\n  Never felt it start. Check the 5V rail under load, the");
  Serial.println("  module's own rating, and that GND reaches the ESP32.");
}

// What the handle will actually feel like in use. Not a wiring test — this
// is so you know whether the cues are distinguishable by touch before the
// handle is built and the motors are buried in it.
void guidanceDemo() {
  Serial.println("\nGuidance preview — what the person feels.\n");

  struct Cue { const char *label; int idx; int duty; int ms; };
  const Cue cues[] = {
    { "forward, gentle   ", 0, 120, 900 },
    { "forward, firm     ", 0, 230, 900 },
    { "turning left      ", 2, 200, 900 },
    { "turning left hard ", 2, 255, 900 },
    { "turning right     ", 3, 200, 900 },
    { "reversing         ", 1, 200, 900 },
  };

  for (unsigned i = 0; i < sizeof(cues) / sizeof(cues[0]); i++) {
    Serial.printf("  %s\n", cues[i].label);
    setMotor(cues[i].idx, cues[i].duty);
    bool stop = waitOrKey(cues[i].ms);
    setMotor(cues[i].idx, 0);
    if (stop || waitOrKey(500)) break;
  }

  Serial.println("\n  STOP — all four, pulsing");
  for (int p = 0; p < 4; p++) {
    for (int i = 0; i < 4; i++) setMotor(i, 255);
    if (waitOrKey(180)) break;
    allOff();
    if (waitOrKey(180)) break;
  }
  allOff();

  Serial.println("\n  LINK LOST — three short pulses, repeating");
  for (int rep = 0; rep < 2; rep++) {
    for (int p = 0; p < 3; p++) {
      for (int i = 0; i < 4; i++) setMotor(i, 220);
      if (waitOrKey(110)) { allOff(); return; }
      allOff();
      if (waitOrKey(110)) return;
    }
    if (waitOrKey(500)) return;
  }
  allOff();
  Serial.println("\nCould you tell the four directions apart by feel alone?");
  Serial.println("If not, that is a mounting problem — the motors need to be");
  Serial.println("further apart or decoupled from each other in the handle.");
}

// ------------------------------------------------------------------- menu

void help() {
  Serial.println();
  Serial.println("=========== vibrator test ===========");
  Serial.println(" 1 2 3 4   pulse front / back / left / right");
  Serial.println(" s         sequence all four in order");
  Serial.println(" h         hold all four on  (x to stop)");
  Serial.println(" x         everything off");
  Serial.println();
  Serial.println(" p         PWM ramp — does intensity vary?");
  Serial.println(" d         find the start threshold");
  Serial.println(" g         guidance preview — what it feels like");
  Serial.println();
  Serial.println(" i         flip polarity (active HIGH <-> LOW)");
  Serial.println(" + -       intensity");
  Serial.println(" ?         this menu");
  Serial.println("=====================================");
  Serial.printf(" intensity %d   polarity %s   threshold %s\n\n",
                intensity,
                activeLow ? "ACTIVE LOW" : "ACTIVE HIGH",
                deadband ? String(deadband).c_str() : "unmeasured");
}

void setup() {
  Serial.begin(115200);
  delay(400);

  for (int i = 0; i < 4; i++) {
    pwmSetup(PINS[i], CH[i]);
    setMotor(i, 0);
  }

  Serial.println();
  Serial.println("Vibration motor test — handle motors only.");
  Serial.println("Drive motors and encoders are not touched by this sketch.");
  Serial.println();
  Serial.println("If all four are buzzing right now, the modules are");
  Serial.println("active LOW. Press 'i' and they should go quiet.");
  help();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\n' || c == '\r' || c == ' ') return;

  switch (c) {
    case '1': case '2': case '3': case '4':
      pulse(c - '1');
      break;

    case 's': sequence();      break;
    case 'p': rampTest();      break;
    case 'd': findDeadband();  break;
    case 'g': guidanceDemo();  break;

    case 'h':
      Serial.printf("All four on at %d. Press x.\n", intensity);
      for (int i = 0; i < 4; i++) setMotor(i, intensity);
      break;

    case 'x':
      allOff();
      Serial.println("off");
      break;

    case 'i':
      activeLow = !activeLow;
      allOff();
      Serial.printf("Polarity now %s\n",
                    activeLow ? "ACTIVE LOW" : "ACTIVE HIGH");
      Serial.println("Correct if everything is silent now.");
      break;

    case '+':
      intensity = min(255, intensity + 25);
      Serial.printf("intensity %d\n", intensity);
      break;

    case '-':
      intensity = max(0, intensity - 25);
      Serial.printf("intensity %d\n", intensity);
      break;

    case '?': help(); break;

    default:
      Serial.println("? for the menu");
      break;
  }
}
