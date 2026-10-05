#include "encoders.h"

namespace Encoders {

namespace {

volatile uint32_t pulsesL = 0;
volatile uint32_t pulsesR = 0;

uint32_t lastL = 0;
uint32_t lastR = 0;

// IRAM_ATTR keeps the handler in RAM. Without it an interrupt arriving while
// the flash cache is busy faults the chip.
void IRAM_ATTR isrLeft()  { pulsesL++; }
void IRAM_ATTR isrRight() { pulsesR++; }

} // anonymous namespace

void begin() {
  // The LM393 drives its output actively; the pull-up only matters if a
  // sensor is unplugged, where it keeps the pin from floating and spraying
  // phantom counts.
  pinMode(PIN_ENC_L, INPUT_PULLUP);
  pinMode(PIN_ENC_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L), isrLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R), isrRight, RISING);
}

void takeDelta(uint32_t &leftPulses, uint32_t &rightPulses) {
  // Snapshot both counters with interrupts off, so a pulse landing mid-read
  // cannot tear a 32-bit value on a dual-core chip.
  noInterrupts();
  uint32_t nowL = pulsesL;
  uint32_t nowR = pulsesR;
  interrupts();

  // Unsigned subtraction wraps correctly, so the 32-bit rollover after
  // ~4 billion pulses costs nothing.
  leftPulses  = nowL - lastL;
  rightPulses = nowR - lastR;

  lastL = nowL;
  lastR = nowR;
}

uint32_t totalLeft() {
  noInterrupts();
  uint32_t v = pulsesL;
  interrupts();
  return v;
}

uint32_t totalRight() {
  noInterrupts();
  uint32_t v = pulsesR;
  interrupts();
  return v;
}

} // namespace Encoders
