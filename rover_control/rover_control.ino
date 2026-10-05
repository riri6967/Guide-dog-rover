/*
 * rover_control.ino — ESP32 guide-dog rover: mode machine + manual override
 *
 * Board: ESP32 Dev Module.  Driver: L298N.  See ../WIRING.md
 *
 * The rover boots into STOPPED and stays there. Nothing moves until a human
 * selects a mode. That ordering is deliberate: a brownout or a reset must
 * never drop the machine straight back into driving itself.
 *
 * Modes
 *   STOPPED  motors dead, ignores everything but a mode change
 *   MANUAL   driven from the phone; motors cut if commands stop arriving
 *   AUTO     autonomousStep() drives. Currently a stub — no sensors fitted yet.
 *
 * To connect: join WiFi network "GuideDog-Rover" (password below),
 * then open http://192.168.4.1 in any browser.
 */

#include <WiFi.h>
#include <WebServer.h>
#include "pins.h"
#include "motors.h"
#include "webui.h"

// ------------------------------------------------------------------ config
const char *AP_SSID = "GuideDog-Rover";
const char *AP_PASS = "roverpass";        // must be 8+ characters

// If no manual command arrives within this window, the link is presumed dead
// and the motors stop. The browser repeats commands every 150 ms, so 600 ms
// tolerates a few dropped packets without nuisance stops.
const unsigned long CONTROL_TIMEOUT_MS = 600;

// Battery monitoring. Wire a divider from pack+ to PIN_VBAT (GPIO34) to GND:
//   pack+ --[100k]--+--[22k]-- GND
//                   |
//                 GPIO34
// GPIO34 is input-only and on ADC1, so it still reads correctly with WiFi up.
// Leave disabled until that divider physically exists.
#define ENABLE_BATTERY_MONITOR 0
const float DIVIDER_RATIO = 5.545f;   // (100k + 22k) / 22k
const float LOW_BATT_V    = 9.6f;     // 3.2 V/cell — stop before cell damage
const float CRIT_BATT_V   = 9.0f;     // 3.0 V/cell — hard cutoff

// ------------------------------------------------------------------ state
enum Mode { MODE_STOPPED, MODE_MANUAL, MODE_AUTO };
Mode mode = MODE_STOPPED;

WebServer server(80);
unsigned long lastCmdMs = 0;
float batteryV = 0.0f;
bool  battCritical = false;

const char *modeName() {
  switch (mode) {
    case MODE_MANUAL: return "MANUAL";
    case MODE_AUTO:   return "AUTO";
    default:          return "STOPPED";
  }
}

void setMode(Mode m) {
  if (m == mode) return;
  Motors::stopNow();          // always pass through a stop when switching
  mode = m;
  lastCmdMs = millis();
  Serial.printf("mode -> %s\n", modeName());
}

// ------------------------------------------------------------- battery
void updateBattery() {
#if ENABLE_BATTERY_MONITOR
  static unsigned long last = 0;
  if (millis() - last < 1000) return;
  last = millis();

  // Average a few reads — the ESP32 ADC is noisy, especially with motors running.
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(PIN_VBAT);
  float pinV = (sum / 16.0f) * (3.3f / 4095.0f);
  batteryV = pinV * DIVIDER_RATIO;

  if (batteryV < CRIT_BATT_V && batteryV > 5.0f) {   // >5V guards a floating pin
    if (!battCritical) {
      Serial.printf("BATTERY CRITICAL %.2f V — shutting down\n", batteryV);
      battCritical = true;
    }
    setMode(MODE_STOPPED);
  }
#endif
}

// --------------------------------------------------------- autonomous stub
/*
 * Called continuously while in AUTO. Right now it does nothing but hold the
 * rover still, because no sensors are fitted — an autonomous mode that drives
 * blind is worse than no autonomous mode at all.
 *
 * TODO once sensors are chosen, this becomes roughly:
 *
 *   d = readForwardDistance();
 *   if (cliffDetected())        { Motors::drive(-120, -120); return; }  // back off a drop
 *   if (d < STOP_DISTANCE_CM)   { Motors::drive(0, 0); alertUser(); return; }
 *   if (d < SLOW_DISTANCE_CM)   { Motors::drive(100, 100); return; }
 *   Motors::drive(cruise, cruise);
 *
 * Read the sensors on a timer, not free-running: HC-SR04s cross-talk if two
 * fire at once, so trigger them in sequence, one per cycle.
 */
void autonomousStep() {
  static unsigned long lastWarn = 0;
  Motors::drive(0, 0);

  if (millis() - lastWarn > 3000) {
    lastWarn = millis();
    Serial.println("AUTO: no sensors configured — holding position");
  }
}

// ---------------------------------------------------------------- handlers
void handleRoot() {
  server.send_P(200, "text/html", PAGE_INDEX);
}

void handleCmd() {
  lastCmdMs = millis();     // any /cmd counts as proof the link is alive

  if (mode != MODE_MANUAL || battCritical) {
    server.send(200, "text/plain", "ignored");
    return;
  }
  int l = server.arg("l").toInt();
  int r = server.arg("r").toInt();
  Motors::drive(l, r);
  server.send(200, "text/plain", "ok");
}

void handleMode() {
  String m = server.arg("m");
  if (battCritical && m != "stop") {
    server.send(200, "text/plain", "battery critical");
    return;
  }
  if      (m == "manual") setMode(MODE_MANUAL);
  else if (m == "auto")   setMode(MODE_AUTO);
  else                    setMode(MODE_STOPPED);

  server.send(200, "text/plain", modeName());
}

void handleEstop() {
  setMode(MODE_STOPPED);
  Motors::stopNow();
  Serial.println("E-STOP");
  server.send(200, "text/plain", "stopped");
}

void handleStatus() {
  char buf[160];
  snprintf(buf, sizeof(buf),
           "{\"mode\":\"%s\",\"l\":%d,\"r\":%d,\"v\":%.2f,\"lowbat\":%s,\"up\":%lu}",
           modeName(), Motors::leftOutput(), Motors::rightOutput(), batteryV,
           (batteryV > 5.0f && batteryV < LOW_BATT_V) ? "true" : "false",
           millis() / 1000);
  server.send(200, "application/json", buf);
}

// ------------------------------------------------------------------ setup
void setup() {
  Serial.begin(115200);
  delay(300);

  Motors::begin();          // wheels are guaranteed dead from here on

#if ENABLE_BATTERY_MONITOR
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);   // full ~0-3.1 V input range
#endif

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/",       handleRoot);
  server.on("/cmd",    handleCmd);
  server.on("/mode",   handleMode);
  server.on("/estop",  handleEstop);
  server.on("/status", handleStatus);
  server.begin();

  Serial.println();
  Serial.printf("AP    : %s / %s\n", AP_SSID, AP_PASS);
  Serial.printf("open  : http://%s\n", WiFi.softAPIP().toString().c_str());
  Serial.println("mode  : STOPPED (select a mode in the browser to move)");
}

// ------------------------------------------------------------------- loop
void loop() {
  server.handleClient();
  updateBattery();

  switch (mode) {
    case MODE_MANUAL:
      // Failsafe. Phone out of range, browser closed, tab backgrounded,
      // battery pulled from the phone — all present as commands simply
      // stopping. Treat silence as "stop", never as "carry on".
      if (millis() - lastCmdMs > CONTROL_TIMEOUT_MS) {
        Motors::drive(0, 0);
      }
      break;

    case MODE_AUTO:
      autonomousStep();
      break;

    case MODE_STOPPED:
    default:
      Motors::drive(0, 0);
      break;
  }

  Motors::update();
}
