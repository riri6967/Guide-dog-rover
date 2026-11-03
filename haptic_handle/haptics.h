/*
 * haptics.h — the handle the person holds
 *
 * Four coin motors at 90 degrees in a rigid handle: front, back, left, right.
 * The person feels WHERE the grip vibrates, and that is the instruction. No
 * code to memorise, no sound, no screen.
 *
 * Two sources drive it:
 *   1. Velocity   — automatic. Whatever the rover is about to do, the handle
 *                   says first. Turning left buzzes left. This is the normal
 *                   state and it is how a real guide dog harness behaves.
 *   2. Patterns   — explicit alerts that override velocity: stop, obstacle,
 *                   link lost. These are events, not continuous guidance.
 *
 * Silence means "carry on". A handle that buzzes constantly gets tuned out
 * within minutes, so the default state is nothing at all.
 */
#pragma once
#include <Arduino.h>
#include "pins.h"

namespace Haptics {

enum Pattern {
  PATTERN_NONE = 0,    // velocity guidance resumes
  PATTERN_STOP,        // all four, strong, pulsing — stop walking
  PATTERN_OBSTACLE,    // front pulsing, faster as it gets closer
  PATTERN_BACKUP,      // back motor steady — step backwards
  PATTERN_LOST,        // three short pulses on all four, repeating.
                       // The person must know the handle has gone blind.
  PATTERN_ARRIVED      // two soft pulses, then silence
};

void begin();

// Translate what the rover is about to do into where the grip buzzes.
// linear m/s, angular rad/s — the same numbers that reach the motors.
void fromVelocity(float linear, float angular);

// Override velocity guidance with an explicit alert. PATTERN_NONE returns
// control to fromVelocity().
void setPattern(Pattern p);

// For PATTERN_OBSTACLE: 0.0 = far, 1.0 = imminent. Sets the pulse rate.
void setUrgency(float urgency);

// Must be called every loop — patterns are time-based.
void update();

// Everything off, immediately.
void allOff();

} // namespace Haptics
