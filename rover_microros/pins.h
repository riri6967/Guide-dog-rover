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
// for an ADC input. Needs the divider described in WIRING.md.
#define PIN_VBAT  34

// ------------------------------------------------------------ encoders
// LM393 slot sensors, one per side, reading a slotted disc on each axle.
// Printed as D18/D19 on the 38-pin DevKit and G18/G19 on the 30-pin.
// Power the sensor modules from 3V3 so OUT never exceeds 3.3V.
#define PIN_ENC_L   18
#define PIN_ENC_R   19

// ------------------------------------------------------------ IMU (I2C)
// ESP32 default I2C pins. MPU6050 is 3.3V-safe on both lines.
#define PIN_SDA     21
#define PIN_SCL     22

// ------------------------------------------- haptic handle (guide grip)
// Four ERM coin motors set into the rigid handle at 90 degrees, driven
// through a ULN2003 Darlington array. A GPIO cannot source 80mA; the
// ULN2003 sinks it and has the flyback diodes built in.
// All four are PWM-capable, so intensity is controllable.
#define PIN_HAP_FRONT  23
#define PIN_HAP_BACK   32
#define PIN_HAP_LEFT   21
#define PIN_HAP_RIGHT  22

// ------------------------------------------------------------- indicators
#define PIN_LED      2    // onboard LED on most DevKits. Output-only use of a
                          // strapping pin after boot is fine.

// Still free: 0 (BOOT button), 5, 12, 15
