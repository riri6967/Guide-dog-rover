/*
 * encoders.h — pulse counting for two LM393 slot sensors
 *
 * Single-channel sensors, so these count pulses but cannot tell which way the
 * wheel turned. The caller applies the sign from whatever it last commanded.
 * That holds as long as nothing pushes the rover by hand or drags a wheel
 * backwards against the motor.
 */
#pragma once
#include <Arduino.h>
#include "pins.h"

namespace Encoders {

void begin();

// Pulses accumulated since the last call to this function, then cleared.
// Unsigned — direction is the caller's business.
void takeDelta(uint32_t &leftPulses, uint32_t &rightPulses);

// Lifetime totals, for calibration and debugging.
uint32_t totalLeft();
uint32_t totalRight();

} // namespace Encoders
