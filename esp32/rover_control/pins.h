/*
 * pins.h — single authority for every GPIO on the rover.
 *
 * Nothing else in the project should hardcode a pin number. If you move a
 * wire, you change it here and only here.
 *
 * ESP32 pin rules this layout obeys:
 *   GPIO 6-11    flash memory. Using them bricks the boot. Not broken out on
 *                most DevKits anyway.
 *   GPIO 34-39   input ONLY. No output, no internal pullup. Good for sensor
 *                inputs and ADC, useless for driving anything.
 *   GPIO 0,2,5,12,15   strapping pins, sampled at boot. 12 is the dangerous
 *                one: held HIGH at reset it sets flash to 1.8V and the board
 *                stops booting. All four are avoided here.
 *   GPIO 1,3     USB serial (TX0/RX0). Leave alone or you lose the monitor.
 *   ADC2 (0,2,4,12-15,25-27) cannot do analogRead() while WiFi is running.
 *                Every analog input below is therefore on ADC1 (32-39).
 */
#pragma once

// ---------------------------------------------------------------- L298N
// All four IN pins are plain GPIOs that sit LOW through boot, so the motors
// cannot twitch while the chip starts.
#define PIN_ENA   13    // left  pair PWM      -> ENA  (jumper removed)
#define PIN_IN1   26    // left  direction A   -> IN1
#define PIN_IN2   27    // left  direction B   -> IN2
#define PIN_ENB   14    // right pair PWM      -> ENB  (jumper removed)
#define PIN_IN3   25    // right direction A   -> IN3
#define PIN_IN4   33    // right direction B   -> IN4

// ------------------------------------------------------- battery monitor
// ADC1, so it keeps working with WiFi up. Input-only pin, which is correct
// for an ADC input. Needs the divider described in docs/WIRING.md.
#define PIN_VBAT  34

// ------------------------------------------------------- the other pins
// This sketch only drives the motors and reads the battery. The encoders
// (18/19) and the haptic handle (21/22/23/32, 36/39) are defined in
// esp32/rover_microros/pins.h, the full map. The LiDAR, IMU and camera are on the
// Raspberry Pi, not here.

// ------------------------------------------------------------- indicators
#define PIN_LED      2    // onboard LED on most DevKits. Output-only use of a
                          // strapping pin after boot is fine.

// Still free: 0 (BOOT button), 4, 5, 12, 15, 16, 17, 35
