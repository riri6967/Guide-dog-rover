/*
 * encoder_test.ino — read two slot-type optical encoders and report speed
 *
 * For LM393-based slot sensors (HC-89, HC-020K, FC-03 and friends): a U-shaped
 * IR gate with a comparator. A slotted disc spins through the gap; every slot
 * edge produces a pulse on the DO pin.
 *
 * This sketch does nothing but count. Drive the wheels by hand or with a
 * separate sketch — the point is to confirm the sensors work and to find out
 * how many pulses your disc really gives per revolution.
 *
 * Wiring, per sensor:
 *   VCC -> ESP32 3V3      (NOT 5V — DO would then swing to 5V and damage a GPIO)
 *   GND -> ESP32 GND
 *   DO  -> the GPIO below
 *
 * Serial Monitor @ 115200. Commands:
 *   r = reset counters
 *   c = calibration mode: counts raw pulses so you can turn the wheel exactly
 *       10 times by hand and work out pulses-per-revolution
 */

// ---------------------------------------------------------------- pins
// Plain GPIOs, interrupt-capable, not strapping pins, and free now that the
// LiDAR and IMU live on the Raspberry Pi.
const int PIN_ENC_L = 18;
const int PIN_ENC_R = 19;

// ---------------------------------------------------------------- geometry
// SET THESE. Count the slots in your disc and measure the wheel.
// If the disc is on the wheel/axle, SLOTS_PER_REV is just the slot count.
// If it is on the motor shaft before the gearbox, multiply by the gear ratio.
const float SLOTS_PER_REV   = 20.0f;    // <-- change to match your disc
const float WHEEL_DIAM_MM   = 65.0f;    // <-- measure your wheel

// How often to compute and print speed.
const unsigned long REPORT_MS = 250;

// ---------------------------------------------------------------- state
volatile uint32_t pulsesL = 0;
volatile uint32_t pulsesR = 0;

uint32_t lastPulsesL = 0;
uint32_t lastPulsesR = 0;
unsigned long lastReport = 0;

bool calibrating = false;

// ---------------------------------------------------------------- ISRs
// IRAM_ATTR puts the handler in RAM rather than flash. Without it the ISR can
// fault if it fires while the flash cache is busy.
void IRAM_ATTR isrLeft()  { pulsesL++; }
void IRAM_ATTR isrRight() { pulsesR++; }

// ---------------------------------------------------------------- helpers

// Pulses over an interval -> RPM of the wheel.
float toRPM(uint32_t pulses, unsigned long intervalMs) {
  if (intervalMs == 0) return 0.0f;
  float revs = pulses / SLOTS_PER_REV;
  return revs * (60000.0f / intervalMs);
}

// RPM -> linear speed at the rim, m/s.
float toMetresPerSec(float rpm) {
  float circumference_m = (WHEEL_DIAM_MM / 1000.0f) * PI;
  return (rpm / 60.0f) * circumference_m;
}

void resetCounts() {
  noInterrupts();
  pulsesL = pulsesR = 0;
  interrupts();
  lastPulsesL = lastPulsesR = 0;
  Serial.println("counters reset");
}

// ---------------------------------------------------------------- setup
void setup() {
  Serial.begin(115200);
  delay(300);

  // The LM393 module drives DO actively, so no internal pull-up is required.
  // INPUT_PULLUP is harmless though and helps if a sensor is unplugged.
  pinMode(PIN_ENC_L, INPUT_PULLUP);
  pinMode(PIN_ENC_R, INPUT_PULLUP);

  // RISING only: one count per slot. Use CHANGE to double the resolution,
  // but then halve SLOTS_PER_REV... or rather, double it. Keep it simple here.
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L), isrLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R), isrRight, RISING);

  Serial.println();
  Serial.println("encoder_test ready");
  Serial.printf("slots/rev = %.0f   wheel = %.0f mm\n", SLOTS_PER_REV, WHEEL_DIAM_MM);
  Serial.println("r = reset    c = toggle calibration mode");
  Serial.println();

  lastReport = millis();
}

// ---------------------------------------------------------------- loop
void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'r') resetCounts();
    if (ch == 'c') {
      calibrating = !calibrating;
      resetCounts();
      Serial.println(calibrating
        ? "CALIBRATION: turn ONE wheel exactly 10 full turns by hand, then read the count.\n"
          "             pulses / 10 = your SLOTS_PER_REV"
        : "calibration off");
    }
  }

  if (millis() - lastReport < REPORT_MS) return;

  unsigned long now = millis();
  unsigned long interval = now - lastReport;
  lastReport = now;

  // Snapshot the volatile counters with interrupts off so a pulse arriving
  // mid-read cannot tear the value.
  noInterrupts();
  uint32_t nowL = pulsesL;
  uint32_t nowR = pulsesR;
  interrupts();

  if (calibrating) {
    Serial.printf("raw pulses   L=%lu   R=%lu\n", nowL, nowR);
    return;
  }

  uint32_t deltaL = nowL - lastPulsesL;
  uint32_t deltaR = nowR - lastPulsesR;
  lastPulsesL = nowL;
  lastPulsesR = nowR;

  float rpmL = toRPM(deltaL, interval);
  float rpmR = toRPM(deltaR, interval);

  Serial.printf("L: %6.1f rpm  %5.2f m/s  |  R: %6.1f rpm  %5.2f m/s  |  total L=%lu R=%lu\n",
                rpmL, toMetresPerSec(rpmL),
                rpmR, toMetresPerSec(rpmR),
                nowL, nowR);
}
