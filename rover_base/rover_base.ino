/*
 * rover_base.ino — ESP32 + L298N skid-steer drive layer
 *
 * Board:  ESP32 Dev Module (ESP-WROOM-32)
 * Driver: L298N (ZX-040 red module)
 * Motors: 4 x 12V DC, wired as two pairs (left pair -> OUT1/OUT2, right pair -> OUT3/OUT4)
 *
 * Bench test over Serial Monitor @ 115200, "No line ending":
 *   w = forward    s = reverse
 *   a = spin left  d = spin right
 *   x / space = stop
 *   + / -     = speed up / down
 *   t         = run automatic test sequence
 *   ?         = print status
 *
 * NOTE: there is deliberately NO failsafe timeout in this sketch — you are
 * driving it by hand from a serial terminal. The moment you add radio control
 * (WiFi/BLE), a watchdog that stops the motors on signal loss is mandatory.
 * The hook for it is marked TODO in loop().
 */

// ---------------------------------------------------------------- pin map
// Chosen to avoid ESP32 strapping pins (0, 2, 5, 12, 15) and flash pins (6-11).
// All four IN pins are plain GPIOs that sit LOW during boot, so the motors
// cannot twitch while the chip starts up.
const int PIN_ENA = 13;   // left  pair PWM   -> L298N ENA (remove jumper)
const int PIN_IN1 = 26;   // left  direction  -> L298N IN1
const int PIN_IN2 = 27;   // left  direction  -> L298N IN2

const int PIN_ENB = 14;   // right pair PWM   -> L298N ENB (remove jumper)
const int PIN_IN3 = 25;   // right direction  -> L298N IN3
const int PIN_IN4 = 33;   // right direction  -> L298N IN4

// ---------------------------------------------------------------- tuning
const int  PWM_FREQ  = 1000;   // Hz. L298N is a slow BJT bridge; 1-5 kHz is the
                               // usable window. Higher = quieter but more loss.
const int  PWM_BITS  = 8;      // 8-bit -> duty range 0..255
const int  PWM_MAX   = 255;

// L298N + a loaded 12V gearmotor will not start turning at low duty. Anything
// below this just makes the motor buzz and heat up, so we snap it to zero.
// Measure yours: lower it until the wheels stop moving, then back off ~10.
const int  DEADBAND  = 60;

// Slew limiting. Jumping 0 -> full duty slams the battery with inrush and can
// trip the BMS or brown out the ESP32. Ramp instead.
const int  RAMP_STEP = 12;     // duty change allowed per tick
const int  RAMP_MS   = 10;     // tick period

// If one side runs faster than the other and the rover curves, trim it here.
// 1.00 = no correction. Drop the fast side to ~0.95 and re-test.
const float TRIM_LEFT  = 1.00f;
const float TRIM_RIGHT = 1.00f;

// ------------------------------------------------- wiring correction flags
// Fix directions here rather than at the screw terminals.
// true = left motors on OUT3/OUT4 and right motors on OUT1/OUT2.
const bool SWAP_SIDES   = true;
// Flip a side that runs backwards. Left and right motors are mirror images on
// the chassis, so usually exactly one of these ends up true.
const bool INVERT_LEFT  = false;
const bool INVERT_RIGHT = true;

// ---------------------------------------------------------------- state
int targetLeft  = 0;   // -255..255, negative = reverse
int targetRight = 0;
int actualLeft  = 0;   // what the ramp has actually applied
int actualRight = 0;
int speedLevel  = 180; // current commanded magnitude

unsigned long lastRamp = 0;

// ------------------------------------------------ PWM core compatibility
// ESP32 Arduino core 3.x replaced ledcSetup/ledcAttachPin with ledcAttach.
// This keeps the sketch compiling on both 2.x and 3.x.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_SETUP(pin, ch)   ledcAttach((pin), PWM_FREQ, PWM_BITS)
  #define PWM_WRITE(pin, ch,d) ledcWrite((pin), (d))
#else
  #define PWM_SETUP(pin, ch)   do { ledcSetup((ch), PWM_FREQ, PWM_BITS); \
                                    ledcAttachPin((pin), (ch)); } while (0)
  #define PWM_WRITE(pin, ch,d) ledcWrite((ch), (d))
#endif

const int CH_LEFT  = 0;
const int CH_RIGHT = 1;

// ---------------------------------------------------------------- helpers

// Apply a signed speed to one side. speed: -255..255
void applySide(int speed, int inA, int inB, int enPin, int enCh, float trim) {
  bool forward = (speed >= 0);
  int  duty    = abs(speed);

  if (duty < DEADBAND) duty = 0;
  duty = (int)(duty * trim);
  duty = constrain(duty, 0, PWM_MAX);

  if (duty == 0) {
    // Both inputs low = coast (motor terminals floating).
    // Both inputs HIGH would be an active brake — harder stop, more current.
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
  } else {
    digitalWrite(inA, forward ? HIGH : LOW);
    digitalWrite(inB, forward ? LOW  : HIGH);
  }

  PWM_WRITE(enPin, enCh, duty);
}

// Route logical left/right onto whichever terminals they are actually wired to,
// applying the inversion flags on the way. Only this function knows about
// physical channels; everything else thinks in "left" and "right".
void applyOutputs(int l, int r) {
  if (INVERT_LEFT)  l = -l;
  if (INVERT_RIGHT) r = -r;

  if (SWAP_SIDES) {
    applySide(l, PIN_IN3, PIN_IN4, PIN_ENB, CH_RIGHT, TRIM_LEFT);
    applySide(r, PIN_IN1, PIN_IN2, PIN_ENA, CH_LEFT,  TRIM_RIGHT);
  } else {
    applySide(l, PIN_IN1, PIN_IN2, PIN_ENA, CH_LEFT,  TRIM_LEFT);
    applySide(r, PIN_IN3, PIN_IN4, PIN_ENB, CH_RIGHT, TRIM_RIGHT);
  }
}

// Set the desired speeds. The ramp in loop() walks the actual output toward these.
void drive(int left, int right) {
  targetLeft  = constrain(left,  -PWM_MAX, PWM_MAX);
  targetRight = constrain(right, -PWM_MAX, PWM_MAX);
}

// Immediate stop, bypassing the ramp. Use for emergencies / failsafe.
void stopNow() {
  targetLeft = targetRight = 0;
  actualLeft = actualRight = 0;
  applyOutputs(0, 0);
}

// Move `value` toward `goal` by at most RAMP_STEP.
int stepToward(int value, int goal) {
  if (value < goal) return min(value + RAMP_STEP, goal);
  if (value > goal) return max(value - RAMP_STEP, goal);
  return value;
}

void printStatus() {
  Serial.printf("speed=%d  target L/R=%d/%d  actual L/R=%d/%d\n",
                speedLevel, targetLeft, targetRight, actualLeft, actualRight);
}

// ---------------------------------------------------------------- test run
void testSequence() {
  struct { const char *name; int l; int r; int ms; } steps[] = {
    {"forward",     speedLevel,  speedLevel,  1200},
    {"stop",                 0,           0,   600},
    {"reverse",    -speedLevel, -speedLevel,  1200},
    {"stop",                 0,           0,   600},
    {"spin left",  -speedLevel,  speedLevel,  1000},
    {"spin right",  speedLevel, -speedLevel,  1000},
    {"stop",                 0,           0,   300},
  };

  for (auto &s : steps) {
    Serial.printf("  %s\n", s.name);
    drive(s.l, s.r);
    unsigned long until = millis() + s.ms;
    while (millis() < until) {
      // keep the ramp running during the hold
      if (millis() - lastRamp >= RAMP_MS) {
        lastRamp = millis();
        actualLeft  = stepToward(actualLeft,  targetLeft);
        actualRight = stepToward(actualRight, targetRight);
        applyOutputs(actualLeft, actualRight);
      }
    }
  }
  stopNow();
  Serial.println("test done");
}

// ---------------------------------------------------------------- commands
void handleCommand(char c) {
  switch (c) {
    case 'w': drive( speedLevel,  speedLevel); Serial.println("forward");    break;
    case 's': drive(-speedLevel, -speedLevel); Serial.println("reverse");    break;
    case 'a': drive(-speedLevel,  speedLevel); Serial.println("spin left");  break;
    case 'd': drive( speedLevel, -speedLevel); Serial.println("spin right"); break;

    case 'x':
    case ' ': stopNow(); Serial.println("stop"); break;

    case '+':
    case '=':
      speedLevel = min(speedLevel + 15, PWM_MAX);
      Serial.printf("speed = %d\n", speedLevel);
      break;

    case '-':
    case '_':
      speedLevel = max(speedLevel - 15, 0);
      Serial.printf("speed = %d\n", speedLevel);
      break;

    case 't': Serial.println("test sequence:"); testSequence(); break;
    case '?': printStatus(); break;
    default: break;   // ignore newlines and stray characters
  }
}

// ---------------------------------------------------------------- setup
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  PWM_SETUP(PIN_ENA, CH_LEFT);
  PWM_SETUP(PIN_ENB, CH_RIGHT);

  stopNow();   // guarantee wheels are dead before anything else runs

  Serial.println();
  Serial.println("rover_base ready");
  Serial.println("w/s/a/d = drive   x = stop   +/- = speed   t = test   ? = status");
  printStatus();
}

// ---------------------------------------------------------------- loop
void loop() {
  while (Serial.available()) {
    handleCommand((char)Serial.read());
  }

  // TODO failsafe: once this is driven over WiFi/BLE instead of Serial, track
  // the timestamp of the last received packet and call stopNow() if it is
  // older than ~500 ms. A rover that keeps driving after losing its link is
  // the single most common way these projects break something.

  if (millis() - lastRamp >= RAMP_MS) {
    lastRamp = millis();
    actualLeft  = stepToward(actualLeft,  targetLeft);
    actualRight = stepToward(actualRight, targetRight);
    applyOutputs(actualLeft, actualRight);
  }
}
