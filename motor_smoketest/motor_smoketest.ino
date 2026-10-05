/*
 * motor_smoketest.ino — diagnostic. Not part of the rover.
 *
 * Everything clever is removed: no PWM, no deadband, no ramp, no serial input.
 * ENA/ENB are driven HIGH with digitalWrite, so the motors get full battery
 * voltage whenever a direction is set. If nothing moves while running this,
 * the fault is electrical — wiring, driver, motors or supply — and not code.
 *
 * It exercises one channel at a time so you can see WHICH half works.
 *
 * Upload, open Serial Monitor at 115200, and watch. It repeats forever.
 * Wheels off the ground.
 */

const int PIN_ENA = 13;   // -> L298N ENA   (jumper OFF)
const int PIN_IN1 = 26;   // -> L298N IN1
const int PIN_IN2 = 27;   // -> L298N IN2

const int PIN_ENB = 14;   // -> L298N ENB   (jumper OFF)
const int PIN_IN3 = 25;   // -> L298N IN3
const int PIN_IN4 = 33;   // -> L298N IN4

const int HOLD_MS = 2500;

void allOff() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_ENB, LOW);
}

// Drive one channel flat out in one direction.
void runChannel(const char *label, int en, int inA, int inB, bool forward) {
  Serial.printf("%-28s  EN=HIGH  %s\n", label, forward ? "A=HIGH B=LOW" : "A=LOW B=HIGH");
  allOff();
  digitalWrite(inA, forward ? HIGH : LOW);
  digitalWrite(inB, forward ? LOW  : HIGH);
  digitalWrite(en, HIGH);
  delay(HOLD_MS);
  allOff();
  delay(600);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  allOff();

  Serial.println();
  Serial.println("motor_smoketest — full voltage, no PWM");
  Serial.println("Watch which of the four steps moves a motor.");
  Serial.println();
}

void loop() {
  runChannel("CH-A (OUT1/OUT2) forward", PIN_ENA, PIN_IN1, PIN_IN2, true);
  runChannel("CH-A (OUT1/OUT2) reverse", PIN_ENA, PIN_IN1, PIN_IN2, false);
  runChannel("CH-B (OUT3/OUT4) forward", PIN_ENB, PIN_IN3, PIN_IN4, true);
  runChannel("CH-B (OUT3/OUT4) reverse", PIN_ENB, PIN_IN3, PIN_IN4, false);

  Serial.println("--- cycle complete, repeating ---");
  Serial.println();
}
