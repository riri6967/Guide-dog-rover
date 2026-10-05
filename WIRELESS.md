# Driving the rover from your phone

`rover_control` turns the ESP32 into its own WiFi access point and serves a
control page. No router, no internet, no app to install. Works in a corridor,
a field, anywhere.

---

## 1. Upload the sketch

Open **`rover_control/rover_control.ino`**.

Along the top of the editor you should see five tabs:

```
rover_control.ino | motors.cpp | motors.h | pins.h | webui.h
```

Arduino IDE pulls in every file sitting in the sketch folder automatically.
You never open the other four yourself — if you do, they load as separate
sketches and nothing works.

**If you see fewer than five tabs**, the files aren't all in the same folder.
All five must sit directly inside `rover_control/`.

Then press Upload. It is a bigger sketch than `rover_base` (72% of flash,
because the WiFi stack is large), so it takes noticeably longer to compile
and upload. That is normal.

---

## 2. Confirm it started

Serial Monitor, **115200 baud**. Press EN to reset if the output has scrolled.

```
AP    : GuideDog-Rover / roverpass
open  : http://192.168.4.1
mode  : STOPPED (select a mode in the browser to move)
```

`mode : STOPPED` is correct and intentional. The rover always boots refusing
to move. If a brownout or reset happened mid-drive, the last thing you want
is for it to come back up and carry on driving.

---

## 3. Join the network from your phone

The ESP32 is broadcasting its own WiFi network. Your phone joins it the same
way it joins any other.

| | |
|---|---|
| Network | `GuideDog-Rover` |
| Password | `roverpass` |

### The "no internet" warning

This network has no route to the internet — it is one small radio on a
microcontroller. Your phone will notice and complain:

- **Android:** a notification appears offering to switch back to mobile data.
  You must decline it. Look for "Stay connected", "Keep using this network",
  or turn mobile data off for a moment. If you ignore this, Android silently
  drops the WiFi and the page will not load.
- **iPhone:** shows "Unsecured Network" or "No Internet Connection" but stays
  connected. Usually nothing to do.

This catches nearly everyone the first time. If the page will not load, this
is the reason more often than not.

### If the network does not appear at all

- Is the ESP32 powered? The serial banner must have printed.
- Pull down to refresh the WiFi list; ESP32 APs sometimes take 10-20 seconds
  to show up.
- Some phones hide 2.4GHz-only networks in a "more networks" section.
- Press EN to reset the board and watch the serial output again.

---

## 4. Open the control page

In any browser:

```
http://192.168.4.1
```

Type the `http://` part. Without it most browsers assume you are searching
and hand it to Google, which then fails because there is no internet.

If it does not load: confirm you are still joined to `GuideDog-Rover` (see
the "no internet" section above — this is almost always it), then reload.

---

## 5. The controls

### Mode buttons, across the top

| Button | What it does |
|---|---|
| **STOP** | Motors dead. Ignores everything except a mode change. Boots here. |
| **Manual** | You drive from the D-pad. |
| **Auto** | Rover drives itself. Currently holds position — no sensors fitted. |

Switching modes always passes through a stop first, so nothing carries over
from the previous mode.

### D-pad

Four arrows plus a red STOP in the middle.

**Press and hold** to drive. **Release to stop.** It is not a toggle — you
hold the arrow for as long as you want to move.

Left and right arrows spin the rover in place (one side forward, one side
back). This is skid steer; there is no steering linkage.

The red centre button is an emergency stop: it cuts the motors and drops the
rover back to STOP mode in one tap.

### Speed slider

60 to 255. Starts at 180.

Below about 60 the motors only buzz without turning, which is why the slider
does not go lower.

### Status panel

Refreshes twice a second:

```
mode       MANUAL
motor L/R  180 / 180
battery    not monitored
uptime     42 s
```

`link lost` in orange means the page cannot reach the rover — out of range,
or the phone dropped the WiFi.

Battery shows "not monitored" until you wire the divider described in
WIRING.md and set `ENABLE_BATTERY_MONITOR` to 1.

---

## 6. The failsafe

While driving, your browser sends the current command every 150 ms. If the
ESP32 hears nothing for 600 ms it cuts the motors.

That single mechanism covers every way control can be lost:

- you walk out of WiFi range
- your phone's battery dies
- you close the tab or switch apps
- the browser crashes
- you simply let go of the button

All of them look identical from the rover's side: commands stop arriving.
Treating silence as "stop" rather than "carry on" is the whole point.

Test it deliberately once, wheels off the ground: hold forward, then lock
your phone. The motors should stop within about half a second.

---

## 7. Going untethered

Right now the ESP32 is powered by USB from your laptop, which limits how far
the rover can go. To cut it loose:

1. Unplug USB
2. Connect **L298N 5V -> ESP32 VIN**

The GND wire between the two boards stays connected the whole time. It is
never removed.

### Never both at once

| Stage | ESP32 power | 5V wire to VIN |
|---|---|---|
| Flashing / debugging | USB | disconnected |
| Driving around | L298N 5V | connected, USB unplugged |

USB and the 5V pin are two power sources. Connected together, your laptop's
5V rail and the L298N's regulator fight each other.

### If it resets randomly on battery power

Symptoms: the rover reboots when the motors start, WiFi drops under load, or
the serial monitor shows `Brownout detector was triggered`.

That is the L298N's little 78M05 linear regulator running out of headroom —
it is rated around 500mA and ESP32 WiFi transmit peaks near that.

The fix is an **LM2596 buck module** (~Rs 60): set its output to exactly 5.0V
with a multimeter first, feed it from the 12V rail, take the 5V-EN jumper off
the L298N, and run its output into ESP32 VIN.

---

## 8. Troubleshooting

| Symptom | Cause |
|---|---|
| Network not in the WiFi list | Board not powered, or list needs refreshing |
| Joined, but page will not load | Phone dropped to mobile data — see section 3 |
| Page loads, motors do not move | Still in STOP mode. Tap **Manual** first |
| Motors stop while still holding | Failsafe fired — weak signal or the tab lost focus |
| Rover drifts instead of going straight | Set `TRIM_LEFT` / `TRIM_RIGHT` in `motors.h` |
| Wrong direction | Set `INVERT_LEFT` / `INVERT_RIGHT` in `motors.h` |
| Resets when motors start | Regulator brownout — see section 7 |

---

## Changing the network name or password

Top of `rover_control.ino`:

```cpp
const char *AP_SSID = "GuideDog-Rover";
const char *AP_PASS = "roverpass";        // must be 8+ characters
```

A password under 8 characters makes the AP fail to start silently.
