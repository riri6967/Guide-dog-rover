/*
 * motors.h — L298N skid-steer drive layer for ESP32
 *
 * Owns all four IN pins and both EN PWM channels. Nothing else in the sketch
 * should touch those GPIOs directly.
 *
 * Speeds are signed: -255 (full reverse) .. 0 (coast) .. +255 (full forward).
 * drive() only sets a target; update() walks the real output toward it so the
 * battery never sees a 0-to-full current step. Call update() every loop.
 */
#pragma once
#include <Arduino.h>
#include "pins.h"

namespace Motors {

constexpr int MAX_SPEED = 255;

// Below this duty the L298N + a loaded 12V gearmotor only buzzes and heats.
// Measure yours and lower it until the wheels genuinely stop turning.
constexpr int DEADBAND = 60;

// Per-side correction for a rover that curves when told to go straight.
// Trim the FAST side down. 1.00 = untouched.
constexpr float TRIM_LEFT  = 1.00f;
constexpr float TRIM_RIGHT = 1.00f;

// ------------------------------------------------- wiring correction flags
// Set these from what the rover actually does on the bench, so you never have
// to unscrew a terminal to fix a direction. See "Direction test" in WIRING.md.

// true  = left motors are on OUT3/OUT4 and right motors on OUT1/OUT2
// false = the other way round
constexpr bool SWAP_SIDES = true;

// Flip a side that spins backwards. Left and right motors are mirror images on
// the chassis, so one side usually needs this and the other does not.
constexpr bool INVERT_LEFT  = false;
constexpr bool INVERT_RIGHT = true;

void begin();

// Request a new speed. Takes effect gradually via update().
void drive(int left, int right);

// Cut output immediately, bypassing the ramp. Use for e-stop and failsafe.
void stopNow();

// Advance the ramp. Must be called continuously from loop().
void update();

int leftOutput();    // duty actually being applied right now
int rightOutput();

} // namespace Motors
