/*
 * drive_and_measure.ino — drive the motors and read the encoders, together
 *
 * No ROS, no Pi, no micro-ROS. Just the ESP32 over USB. This is the sketch for
 * answering one question: when the motors turn, do the encoders measure it?
 *
 * It is also how you get three of the four calibration constants that
 * rover_microros needs.
 *
 * Serial Monitor @ 115200.
 *
 *   w / s     forward / reverse
 *   a / d     spin left / right
 *   x         stop
 *   + / -     speed up / down (steps of 15)
 *   c         CALIBRATE: counts raw pulses. Turn one wheel exactly 10 turns
 *             by hand, then pulses / 10 = SLOTS_PER_REV
 *   m         MEASURE: steps through duty levels, reports RPM at each.
 *             Gives you the duty-to-speed curve and the top speed.
 *   ?         print settings
 */

// ---------------------------------------------------------------- pins
const int PIN_ENA = 13;   // right board ENA + ENB
const int PIN_IN1 = 26;   // right board IN1 + IN3
const int PIN_IN2 = 27;   // right board IN2 + IN4
const int PIN_ENB = 14;   // left  board ENA + ENB
const int PIN_IN3 = 25;   // left  board IN1 + IN3
const int PIN_IN4 = 33;   // left  board IN2 + IN4

const int PIN_ENC_L = 18;
const int PIN_ENC_R = 19;

// ------------------------------------------------- SET THESE AS YOU LEARN THEM
// Start with the slot count you printed, then replace it with what 'c' tells
// you. The printed count and the measured count often differ — reflections and
// double-triggering are common, and the measured number is the true one.
float SLOTS_PER_REV = 140.0f;   // measured: 1392 and 1402 pulses over 10 wheel turns
float WHEEL_DIAM_MM = 65.0f;

// ---------------------------------------------------------------- tuning
const int PWM_FREQ = 1000;
const int PWM_BITS = 8;
const int PWM_MAX  = 255;
const int DEADBAND = 60;     // below this an L298N + loaded motor only buzzes

// Direction correction — same flags as the rover firmware.
const bool SWAP_SIDES   = true;
const bool INVERT_LEFT  = false;
const bool INVERT_RIGHT = true;

const unsigned long REPORT_MS = 250;

// ------------------------------------------------ core compatibility
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_SETUP(pin, ch)    ledcAttach((pin), PWM_FREQ, PWM_BITS)
  #define PWM_WRITE(pin, ch, d) ledcWrite((pin), (d))
#else
  #define PWM_SETUP(pin, ch)    do { ledcSetup((ch), PWM_FREQ, PWM_BITS); \
                                     ledcAttachPin((pin), (ch)); } while (0)
  #define PWM_WRITE(pin, ch, d) ledcWrite((ch), (d))
#endif

const int CH_A = 0;
const int CH_B = 1;

// ---------------------------------------------------------------- state
volatile uint32_t pulsesL = 0;
volatile uint32_t pulsesR = 0;
uint32_t lastPulsesL = 0, lastPulsesR = 0;

int  speedLevel = 180;
int  curLeft = 0, curRight = 0;
bool calibrating = false;
unsigned long lastReport = 0;

void IRAM_ATTR isrL() { pulsesL++; }
void IRAM_ATTR isrR() { pulsesR++; }

// ---------------------------------------------------------------- motors
void applySide(int speed, int inA, int inB, int enPin, int enCh) {
  bool forward = (speed >= 0);
  int duty = abs(speed);
  if (duty < DEADBAND) duty = 0;
  duty = constrain(duty, 0, PWM_MAX);

  if (duty == 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
  } else {
    digitalWrite(inA, forward ? HIGH : LOW);
    digitalWrite(inB, forward ? LOW  : HIGH);
  }
  PWM_WRITE(enPin, enCh, duty);
}

void drive(int l, int r) {
  curLeft = l;
  curRight = r;
  if (INVERT_LEFT)  l = -l;
  if (INVERT_RIGHT) r = -r;

  if (SWAP_SIDES) {
    applySide(l, PIN_IN3, PIN_IN4, PIN_ENB, CH_B);
    applySide(r, PIN_IN1, PIN_IN2, PIN_ENA, CH_A);
  } else {
    applySide(l, PIN_IN1, PIN_IN2, PIN_ENA, CH_A);
    applySide(r, PIN_IN3, PIN_IN4, PIN_ENB, CH_B);
  }
}

// ---------------------------------------------------------------- maths
float toRPM(uint32_t pulses, unsigned long ms) {
  if (ms == 0) return 0.0f;
  return (pulses / SLOTS_PER_REV) * (60000.0f / ms);
}

float toMS(float rpm) {
  return (rpm / 60.0f) * ((WHEEL_DIAM_MM / 1000.0f) * PI);
}

void snapshot(uint32_t &dL, uint32_t &dR) {
  noInterrupts();
  uint32_t nL = pulsesL, nR = pulsesR;
  interrupts();
  dL = nL - lastPulsesL;
  dR = nR - lastPulsesR;
  lastPulsesL = nL;
  lastPulsesR = nR;
}

// ---------------------------------------------------------- measure sweep
// Steps the duty up, holds each level, and reports the steady-state RPM.
// Gives you the duty-to-speed curve, where the motors actually start turning,
// and the top speed — which is MAX_WHEEL_SPEED_MS for the rover firmware.
void measureSweep() {
  Serial.println();
  Serial.println("duty   L rpm   R rpm   L m/s   R m/s");
  Serial.println("-----------------------------------");

  for (int duty = 60; duty <= 255; duty += 25) {
    drive(duty, duty);
    delay(1200);                 // let the speed settle

    uint32_t dL, dR;
    snapshot(dL, dR);            // discard the spin-up
    delay(800);
    snapshot(dL, dR);            // this one counts

    float rpmL = toRPM(dL, 800);
    float rpmR = toRPM(dR, 800);
    Serial.printf("%4d  %6.1f  %6.1f  %6.2f  %6.2f\n",
                  duty, rpmL, rpmR, toMS(rpmL), toMS(rpmR));
  }

  drive(0, 0);
  Serial.println("-----------------------------------");
  Serial.println("Top-row m/s is MAX_WHEEL_SPEED_MS for rover_microros.");
  Serial.println("Lowest duty with non-zero rpm is your real deadband.");
  Serial.println();
}

// ---------------------------------------------------------------- commands
void handle(char c) {
  switch (c) {
    case 'w': drive( speedLevel,  speedLevel); Serial.println("forward");    break;
    case 's': drive(-speedLevel, -speedLevel); Serial.println("reverse");    break;
    case 'a': drive(-speedLevel,  speedLevel); Serial.println("spin left");  break;
    case 'd': drive( speedLevel, -speedLevel); Serial.println("spin right"); break;
    case 'x':
    case ' ': drive(0, 0); Serial.println("stop"); break;

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

    case 'c':
      calibrating = !calibrating;
      drive(0, 0);
      noInterrupts(); pulsesL = pulsesR = 0; interrupts();
      lastPulsesL = lastPulsesR = 0;
      Serial.println(calibrating
        ? "\nCALIBRATE: turn ONE wheel exactly 10 full turns by hand.\n"
          "           pulses / 10 = SLOTS_PER_REV. Press c again when done."
        : "calibration off");
      break;

    case 'm':
      // Leave calibration mode first, or its raw-pulse printout buries the
      // sweep table underneath it.
      calibrating = false;
      Serial.println("MEASURE: wheels OFF THE GROUND. Starting in 2s...");
      delay(2000);
      measureSweep();
      break;

    case '?':
      Serial.printf("slots/rev=%.1f  wheel=%.0fmm  speed=%d  deadband=%d\n",
                    SLOTS_PER_REV, WHEEL_DIAM_MM, speedLevel, DEADBAND);
      break;

    default: break;
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
  PWM_SETUP(PIN_ENA, CH_A);
  PWM_SETUP(PIN_ENB, CH_B);
  drive(0, 0);

  pinMode(PIN_ENC_L, INPUT_PULLUP);
  pinMode(PIN_ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R), isrR, RISING);

  Serial.println();
  Serial.println("drive_and_measure ready");
  Serial.println("w/s/a/d drive   x stop   +/- speed   c calibrate   m measure   ? info");
  Serial.println();
  lastReport = millis();
}

// ---------------------------------------------------------------- loop
void loop() {
  while (Serial.available()) handle((char)Serial.read());

  if (millis() - lastReport < REPORT_MS) return;
  unsigned long interval = millis() - lastReport;
  lastReport = millis();

  uint32_t dL, dR;
  snapshot(dL, dR);

  if (calibrating) {
    noInterrupts();
    uint32_t tL = pulsesL, tR = pulsesR;
    interrupts();
    Serial.printf("raw pulses   L=%lu   R=%lu\n", tL, tR);
    return;
  }

  float rpmL = toRPM(dL, interval);
  float rpmR = toRPM(dR, interval);

  // Commanded duty alongside measured speed. A non-zero duty with zero rpm is
  // the failure worth seeing immediately — the motor is being driven and the
  // encoder is seeing nothing.
  Serial.printf("duty L%+4d R%+4d | L %6.1f rpm %5.2f m/s | R %6.1f rpm %5.2f m/s%s\n",
                curLeft, curRight,
                rpmL, toMS(rpmL),
                rpmR, toMS(rpmR),
                (curLeft || curRight) && !dL && !dR ? "   <-- DRIVEN BUT NO PULSES" : "");
}
