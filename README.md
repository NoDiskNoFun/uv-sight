# UV-Sight

Automatic UV illumination, cant indicator and shot counter for a compound bow hunting sight, with a companion web app.

> **Disclaimer: AI-generated project**
>
> The firmware, the web app, this README and the hardware design (part selection and wiring) were created with **Claude**, an AI assistant by Anthropic, in a conversation with the project owner.
>
> | | |
> |---|---|
> | Model | Claude Opus 5.5 (Anthropic) |
> | Interface | claude.ai, chat with code execution and web search |
> | Period | September 2026, finished on 25 September 2026 |
> | Language of the conversation | German; code, app and README in English |
> | Token usage | Not available. The assistant has no access to token counts or cost data of the conversation. |
> | Human role | Requirements and feature decisions, choice and purchase of parts, soldering and assembly, compiling and flashing, testing on the real bow, bug reports |
> | AI role | Part selection and wiring, all firmware and app code, protocol design, research of datasheets and pinouts, documentation |
> | Testing by the AI | Syntax and type checks of the firmware against mock libraries, JSON validity checks, browser tests of the app with simulated data. The AI never ran the code on real hardware. |
> | Firmware / protocol / app | Firmware 1.5, protocol 4, app 1.1 |
>
> Several values in this project are estimates or were only checked in the field by the owner (runtimes, thresholds, shot detection). Treat them as starting points, not as guarantees. LiPo batteries can be dangerous if handled wrongly. Build and use this at your own risk.

---

## Contents

1. [Features](#features)
2. [Hardware](#hardware)
3. [Wiring](#wiring)
4. [Assembly and mounting](#assembly-and-mounting)
5. [Firmware: build and flash](#firmware-build-and-flash)
6. [Operation](#operation)
7. [Battery and charging](#battery-and-charging)
8. [Bluetooth commands](#bluetooth-commands)
9. [Settings reference](#settings-reference)
10. [Compile-time constants](#compile-time-constants)
11. [Web app](#web-app)
12. [JSON protocol (version 4)](#json-protocol-version-4)
13. [Troubleshooting](#troubleshooting)
14. [Known limitations](#known-limitations)

---

## Features

- **Automatic illumination:** The UV LED lights the sight's fluorescent fibre when it is dark and the bow is in use. Brightness is held steady over the whole battery discharge via PWM.
- **Motion-based power management:** When the bow lies still, LED, light sensor and Bluetooth switch off and the board sleeps. Picking the bow up wakes it within about half a second.
- **Cant indicator:** The LED blinks when the bow is canted sideways, faster the more it is tilted. Aiming up or down does not count. Modes: off, auto (during a session) and on (always).
- **Shot counter:** Detects every shot through the impact on the riser (IMU tap detection via hardware interrupt). Shots are grouped into ends and training sessions.
- **Scoring:** Enter the scores of each end (0–10, X, M). Each end is checked against the counted shots. The last 32 sessions are stored on the device.
- **Battery:** USB-C charging at a fixed 50 mA, charge status, battery percentage and a deep-discharge cutoff for the LED.
- **Bluetooth:** Everything can be read and configured over Bluetooth LE (Nordic UART Service), either with a serial terminal app or with the included web app.
- **Web app (PWA):** Status, training keypad, history with chart and CSV export, settings with guided calibration. Works offline once installed.
- **Update mode:** Reboot into the UF2 bootloader from the app, no double click on the reset button needed.

---

## Hardware

### Bill of materials

| # | Part | Type / value | Qty | Purpose |
|---|---|---|---|---|
| 1 | Microcontroller | Seeed Studio XIAO nRF52840 **Sense** | 1 | Control, Bluetooth, built-in LiPo charger (BQ25101), USB-C, 6-axis IMU (LSM6DS3TR-C) |
| 2 | LiPo battery | 3.7 V, 85 mAh, about 4 × 12 × 25 mm, with protection circuit | 1 | Power supply |
| 3 | UV LED | 5 mm, 390–405 nm (the owner reused the LED of the sight's original light, forward voltage measured 2.873 V at about 1 mA) | 1 | Lights the sight fibre, cant indicator |
| 4 | Resistor | 68 Ω, 1/4 W | 1 | LED current limit (about 16 mA peak at 4.2 V) |
| 5 | NPN transistor | BC547 (or BC337) | 1 | Switches the LED from the battery |
| 6 | Resistor | 2.2 kΩ, 1/4 W | 1 | Transistor base resistor |
| 7 | Photoresistor (LDR) | GL5528, 5 mm | 1 | Ambient light sensor |
| 8 | Resistor | 10 kΩ, 1/4 W | 1 | Voltage divider for the LDR |
| 9 | Wire | Thin flexible silicone wire, 2 cores, about 15–20 cm | 1 | Housing to LED in the sight |
| 10 | Heat shrink | Assorted sizes | – | Insulation, strain relief |
| 11 | Housing | Rigid, screwed to the riser | 1 | See [assembly](#assembly-and-mounting) |

No separate battery voltage divider, charger module or switch is needed: the XIAO board has a battery divider, charger and USB-C on board. Power management is done entirely in software.

### Why a transistor for the LED

A GPIO pin only delivers the regulated 3.3 V of the board. A UV LED needs about 3.0–3.4 V, which leaves nearly nothing for current regulation. The LED is therefore powered directly from the battery (3.4–4.2 V) and the GPIO only switches the transistor.

### Battery notes

- The battery used here has **three wires**. The third one is a temperature sensor (NTC). The XIAO has no pad for it: **cut it short and insulate it**, only plus and minus are connected.
- Use a cell with a built-in protection circuit. It is the last line of defence against deep discharge.

---

## Wiring

### Pin assignment

| Function | XIAO pin | nRF52840 | Notes |
|---|---|---|---|
| UV LED (PWM, via transistor) | D6 | P1.11 | Base of BC547 via 2.2 kΩ |
| LDR supply | D2 | P0.28 | Only HIGH during a measurement (saves power) |
| LDR measurement | D0 / A0 | P0.02 | Midpoint of the LDR divider |
| Battery voltage | internal | P0.31 | On-board divider 1 MΩ / 510 kΩ |
| Battery measurement enable | internal | P0.14 | Kept LOW permanently (must not be HIGH while charging) |
| Charge current select | internal | P0.13 | High-impedance input = 50 mA (set in code) |
| Charge status | internal | P0.17 | LOW = charging (board charge LED) |
| IMU interrupt INT1 | internal | P0.11 | Shot detection |
| IMU power | internal | P1.08 | Switched on by the firmware |

D1, D3–D5 and D7–D10 remain free.

### UV LED

```
BAT+ ---[68 Ω]--- LED anode (+, long leg) --- LED cathode (-, short leg) --- collector (C)
                                                                             BC547
D6 -----[2.2 kΩ]------------------------------------------------------------ base (B)

GND ------------------------------------------------------------------------ emitter (E)
```

BC547 pinout, flat side facing you, legs down: **C – B – E** from left to right. Check against the datasheet of your part.

### Light sensor

```
D2 ---[LDR GL5528]---+---[10 kΩ]--- GND
                     |
                     A0
```

The LDR has no polarity. Bright light means a high reading, darkness a low reading.

### Battery

```
Battery (+) --- BAT+ pad (bottom of the XIAO)
Battery (-) --- BAT- pad
Battery NTC (3rd wire) --- not connected, insulated
```

### Overview

```
                 +--------------------------------------+
   USB-C ------> |  XIAO nRF52840 Sense                 |
                 |   (charger, IMU, Bluetooth on board) |
                 |                                      |
   LiPo 85 mAh --| BAT+ / BAT-                          |
                 |                                      |
                 | D6 ---[2.2k]--- BC547 --- UV LED ----+---- to the sight (2-core wire)
                 | D2 ---[LDR]---+                      |
                 | A0 -----------+---[10k]--- GND       |
                 +--------------------------------------+
```

---

## Assembly and mounting

- **Mount the housing rigidly.** Shot detection relies on the impact travelling through the riser. Screw the housing on, ideally to an existing accessory thread. Foam double-sided tape and magnets dampen the impact and cause missed shots. Thin structural acrylic tape without a foam core is the fallback if screwing is impossible.
- **Orientation does not matter.** All three IMU axes are evaluated for shots, and the cant indicator is calibrated in place.
- **The LED sits in the sight,** the electronics in the housing, connected by thin silicone wire with strain relief. Mark the LED polarity before desoldering an original light.
- **Place the LDR so that no UV light from the sight falls on it,** otherwise the light could switch itself off.
- **Keep the transistor and the 68 Ω resistor in the housing,** not in the sight.

---

## Firmware: build and flash

### Setup (Arduino IDE)

1. Add this boards manager URL in the preferences:
   `https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json`
2. Install the board package **"Seeed nRF52 Boards"** (not the "mbed-enabled" variant).
3. Select the board **"Seeed XIAO nRF52840 Sense"**.
4. Install the library **"Seeed Arduino LSM6DS3"** via the library manager. Bluefruit and LittleFS are part of the board package.

### Setup (arduino-cli)

```sh
arduino-cli config add board_manager.additional_urls \
  https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
arduino-cli core update-index
arduino-cli core install Seeeduino:nrf52
arduino-cli lib install "Seeed Arduino LSM6DS3"

arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense path/to/uv_visier/
arduino-cli upload  --fqbn Seeeduino:nrf52:xiaonRF52840Sense -p /dev/ttyACM0 path/to/uv_visier/
```

The sketch file must have the same name as its folder, for example `uv_visier/uv_visier.ino`.

### Update mode (UF2)

Instead of double-clicking the reset button you can reboot the board into its UF2 bootloader:

- in the app: *Settings → Firmware and app → Update mode*, or
- in a terminal: send `dfu`.

The USB cable must be connected, otherwise the command is refused (the board would sit in the bootloader and drain the battery). The board appears as a USB drive. Copy a `.uf2` firmware onto it, or upload with the Arduino tools.

### What is stored in flash

| File | Content | Saved |
|---|---|---|
| `/uvcfg.bin` | Settings | With `save` |
| `/shots.bin` | Shot counter on/off, session log (32 slots) | Automatically |
| `/level.bin` | Cant indicator mode and calibration | Automatically |

When a firmware update changes the layout of a file, that file starts over with defaults. The log has been reset like this several times during development. Export your history from the app before updating.

---

## Operation

### Active and idle

- **Active:** The bow was moved within the timeout (default 5 min). LED logic, light sensor and Bluetooth run.
- **Idle:** The bow lies still. Everything is off, the board wakes every 0.5 s briefly to check the IMU. A Bluetooth connection is dropped when the board goes idle; a connection alone does not keep it awake.
- A running session survives idle phases. It is only ended by 1 h without a shot, by `shots stop`, or by a restart of the board.

### Illumination

- Every 2 s the light sensor is read (0 = dark, 4095 = bright).
- Below `dark_on` it counts as dark, above `dark_off` as bright again. The gap prevents flicker.
- `confirm` readings in a row are needed to switch. Right after picking up the bow the decision is immediate.
- The LED brightness (`ma`) is kept steady by PWM as the battery voltage drops.
- Test modes: `mode on` (always on), `mode off` (always off), `mode auto` (normal). The mode is not saved.

### Cant indicator

| | level | canted |
|---|---|---|
| bright | LED off | LED blinks |
| dark | LED on | LED blinks |

- Blinking uses the reduced current `tilt_ma`. Just outside the tolerance it blinks at 1 Hz, from 10° on at 8 Hz, in between the rate rises linearly. A new rate takes effect at the start of the next blink cycle.
- Only sideways cant counts. Aiming up or down is ignored thanks to the two-step calibration.
- After a shot the indicator pauses for `lockout` (default 1.5 s).

| Mode | Command | Behaviour |
|---|---|---|
| Off | `level off` | No blinking |
| Auto | `level auto` | Only during a running session (needs `shots on`) |
| On | `level on` | Always while the bow is active |

**Calibration (once, saved immediately):**

1. Put the bow in a stand, level it with the bubble and aim horizontally. Send `level cal`.
2. Keep the bow level, but aim clearly up or down, at least 20°. Send `level cal2`.

Each step waits 3 s, then measures for about 0.6 s. If the two positions differ by less than 15°, step 2 has to be repeated.

### Shot counter, sessions and ends

- Turn the counter on with `shots on` (saved immediately).
- **Session start:** automatically with the first detected shot, or manually with `shots start`.
- **Shot detection:** The IMU reports impacts above `tap_ths` via interrupt. After a shot further impacts are ignored for `lockout`. Setting the bow down at the end often counts as a shot; this is handled below.
- **Ends:** Each `score` line closes an end and is compared with the shots counted since the last entry.
  - Match: the end is saved.
  - Mismatch: the device asks `save anyway? (yes/no)`. `yes` saves your values and your arrow count wins over the sensor count. `no` discards the input and the end stays open. Any other command also discards the input.
  - `score skip` closes an end without scores: it is stored as invalid, the sensor's arrow count is used, all arrows count 0 points and are left out of the average.
- **Scores:** `0`–`10` and `x` (inner ten: 10 points, counted separately). Several values per line, for example `score x 10 9 9 8 7`. If one value is invalid, the whole line is rejected. The app shows 0 as M (miss).
- **Session end:** automatically after 1 h without a shot, or with `shots stop`. An open last end with exactly 1 shot is ignored (bow set down); with more shots it is stored as invalid. Sessions without any end are discarded.
- **Stored per session:** ends, shots, scored arrows, average, X count, duration in minutes, invalid ends and arrows, flags (corrected by hand, ended by hand). There is no clock on the board; the app derives the date.
- The log holds 32 sessions and overwrites the oldest one when full.

---

## Battery and charging

| Item | Value |
|---|---|
| Charge current | Fixed 50 mA (about 0.6 C for 85 mAh), about 2 h for a full charge |
| Charge termination | 4.2 V (charger chip) |
| LED cutoff | `cutoff`, default 3.40 V; released again at cutoff + 0.25 V |
| Percentage | 0 % = `cutoff`, 100 % = 4.2 V, following a typical LiPo curve |
| Charge status | `charging`, `full` (USB connected, done), `no USB` |

- While charging the voltage reads higher than at rest, so the percentage is optimistic. After charging a full cell settles to about 4.10–4.15 V.
- The ADC uses a 40 µs acquisition time because the on-board battery divider has a very high impedance. With the default 3 µs the voltage read about 0.1 V too low.
- The microcontroller keeps running below the cutoff; the battery's own protection circuit disconnects at roughly 2.5–3.0 V. The cutoff is what protects the cell from the LED.

### Runtime estimates

These are estimates, not measurements.

| Mode | Current (approx.) | Runtime with 85 mAh |
|---|---|---|
| Idle (bow lies still) | 0.04–0.08 mA | several weeks up to about 2 months |
| Active at night, LED at 5 mA | about 6 mA | about 14 h |
| Active in daylight, training, no LED | 0.5–1 mA | several days |

Measurements taken with a USB power meter are not meaningful for battery runtime: with USB connected, the USB interface of the chip is active and the LED draws from the battery, not from USB.

---

## Bluetooth commands

Connect with any Nordic UART terminal (for example the Android app "Serial Bluetooth Terminal", line ending LF or CR+LF), with `ble-serial` on Linux, or with the web app. The board advertises as **UV-Sight** only while it is active.

| Command | Effect | Saved |
|---|---|---|
| `help` or `h` | List of commands | – |
| `status` or `?` | Current readings | – |
| `get` or `config` | All settings with range | – |
| `set <name> <value>` | Change a setting, active immediately | Only with `save` |
| `save` | Save settings to flash | Yes |
| `defaults` | Load default settings | Only with `save` |
| `live on` / `live off` | Status every 2 s | No |
| `mode auto` / `on` / `off` | LED test mode | No |
| `shots` | Counter status and running session | – |
| `shots on` / `shots off` | Shot counter on/off (off only without a running session) | Immediately |
| `shots start` / `shots stop` | Start / end a session manually | Slot on end |
| `score 9 x 7 0` | Score an end | With the session |
| `score skip` | Close the end without scores (invalid) | With the session |
| `yes` / `no` | Answer a mismatch question | – |
| `log` | Stored sessions, 1 = newest | – |
| `level` | Cant indicator status | – |
| `level off` / `auto` / `on` | Cant indicator mode | Immediately |
| `level cal` / `level cal2` | Calibration step 1 / 2 | After step 2 |
| `app on` / `app off` | JSON output for the app / human-readable output | No (off on disconnect) |
| `dfu` | Reboot into update mode (USB needed) | – |

Anyone within radio range (about 10 m) can connect; there is no PIN.

---

## Settings reference

Change with `set <name> <value>`, keep with `save`. Out-of-range values are rejected.

| Name | Meaning | Unit | Default | Min | Max |
|---|---|---|---|---|---|
| `dark_on` | Light reading below which it is dark | 0–4095 | 900 | 0 | 4095 |
| `dark_off` | Light reading above which it is bright again | 0–4095 | 1200 | 0 | 4095 |
| `confirm` | Consecutive readings (2 s each) before switching | count | 2 | 1 | 10 |
| `ma` | Average LED current, steady light | mA | 5.0 | 0.5 | 15 |
| `tilt_ma` | Average LED current while cant-blinking | mA | 1.0 | 0.2 | 15 |
| `vf` | Forward voltage of the UV LED (for the brightness control) | V | 3.00 | 2.50 | 3.60 |
| `cutoff` | Battery cutoff for the LED, equals 0 % | V | 3.40 | 3.00 | 3.70 |
| `level_tol` | Cant tolerance | degrees | 1.0 | 0.2 | 10 |
| `tap_ths` | Shot threshold | × 0.25 g | 20 (5 g) | 1 | 31 |
| `ths` | Wake-up motion threshold | × 125 mg | 1 | 1 | 63 |
| `lockout` | After a shot: no counting, cant indicator paused | ms | 1500 | 200 | 5000 |
| `timeout` | Time without movement until idle | s | 300 | 10 | 3600 |
| `report` | Automatic status while connected, 0 = off | s | 10 | 0 or 2 | 600 |

**Calibrating the light thresholds:** at dusk send `live on`, watch the `Light=` value, set `dark_on` where the fibre no longer glows by itself and `dark_off` about 300 higher. Then `live off` and `save`.

**Calibrating the shot threshold:** every detected shot is reported. Shoot a few arrows and carry the bow around. Lower `tap_ths` if shots are missed, raise it if carrying counts as a shot. Then `save`.

---

## Compile-time constants

These are set at the top of the sketch and need a rebuild.

| Constant | Value | Meaning |
|---|---|---|
| `SLOT_COUNT` | 32 | Stored sessions (1–255). Changing it resets the log. |
| `SESSION_IDLE_MS` | 1 h | Time without a shot until the session ends automatically |
| `MAX_SCORES_LINE` | 40 | Maximum values per `score` line |
| `TILT_BLINK_MIN_HZ` | 1.0 Hz | Blink rate just outside the tolerance |
| `TILT_BLINK_MAX_HZ` | 8.0 Hz | Blink rate at strong cant (above ~10 Hz it looks like steady light) |
| `TILT_FULL_DEG` | 10° | Cant at which the maximum rate is reached |
| `TILT_HYST_DEG` | 0.3° | Hysteresis at the tolerance edge |
| `TILT_SMOOTH_MS` | 400 ms | Smoothing against hand tremor |
| `CAL_MIN_ANGLE_DEG` | 15° | Minimum angle between the calibration steps |
| `CAL_COUNTDOWN_MS` | 3000 ms | Wait before a calibration measurement |
| `SENSOR_INTERVAL_MS` | 2000 ms | Light and battery reading interval |
| `ACTIVE_POLL_MS` / `LEVEL_LOOP_MS` / `IDLE_POLL_MS` | 250 / 30 / 500 ms | Loop interval active / with cant indicator / idle |
| `BLE_TX_POWER` | 0 dBm | Radio power (about 10 m) |
| `ADV_FAST_INTERVAL` / `ADV_SLOW_INTERVAL` | 100 ms / ~1 s | Advertising for the first 10 s / afterwards |
| `CONN_INT_MIN` / `CONN_INT_MAX` | 200 / 400 ms | Requested connection interval |
| `BLE_NAME` | `UV-Sight` | Bluetooth name |

---

## Web app

### Files

All six files belong in one folder:

```
index.html
sw.js
manifest.webmanifest
icon-192.png
icon-512.png
icon-maskable-512.png
```

The main file must be named exactly `index.html`.

### Hosting and installing

Web Bluetooth only works in **Chrome on Android** and only on pages loaded via **https** or **localhost**. Opening the file directly from storage does not work.

- **Local server on the phone:** serve the folder with a local web server app and open `http://localhost:8080/`.
- **GitHub Pages:** create a repository, upload the six files, enable *Settings → Pages* for the `main` branch. The app is then at `https://<user>.github.io/<repo>/`.
- **Own server:** any static https web server.

Open the app once completely, then install it via *Settings → Install app* or the Chrome menu. After that it starts from the home screen without internet. When the server is reachable it loads updates automatically (network first, cached copy after 2.5 s).

If the device chooser stays empty, give Chrome the permission *Nearby devices* (Android 12 and later) or *Location* (older Android).

### Tabs

- **Status:** battery ring with percentage and voltage, session, LED, ambient light, cant, charging. Refreshes every 5 s while open.
- **Training:** current end with live shot count, session statistics, keypad in target ring colours (`1 2 3 / 4 5 6 / 7 8 9 / 10 X M`, undo), save end, skip end, end session. Mismatch questions appear as a dialog; *Edit scores* keeps your entries for correction.
- **History:** sessions from the device are archived on the phone without duplicates, with date, chart of the average and CSV export. Removing an entry only affects the phone.
- **Settings:** shot counter, cant indicator (off/auto/on), LED test, all settings grouped with slider, number field, description, default and range, save bar, restore defaults, guided cant calibration, update mode, install button, versions.
- **Console:** raw lines and free command input for troubleshooting.

The archive is stored in Chrome's site data. Clearing Chrome's site data deletes it; export CSV now and then as a backup.

---

## JSON protocol (version 4)

After `app on` the device sends **only JSON, one object per line**, until `app off` or disconnect. Commands stay the same text commands. Every object has a type field `t`. Lines that do not start with `{` (for example the greeting before `app on`) can be ignored.

On `app on` the device sends `hello`, the settings (`cfgStart`, `cfgItem` …, `cfgEnd`), `status`, `session` and `level`.

| Type | Fields | Sent |
|---|---|---|
| `hello` | `proto`, `fw`, `name`, `imu` | On `app on` |
| `status` | `light`, `dark`, `vbat`, `pct`, `chg` (`charging`/`full`/`no USB`), `led` (`on`/`off`/`blinking`), `duty`, `mode`, `lowbat`, `tilt` (degrees or `null`), `session`, `end`, `endShots` | On `status`, with `live on` every 2 s, every `report` s |
| `cfgStart` | `n` | Start of the settings list |
| `cfgItem` | `k`, `v`, `def`, `min`, `max`, `dec`, `zero`, `unit`, `d` | One per setting |
| `cfgEnd` | – | End of the settings list |
| `session` | `counter`, `active`; if active also `end`, `endShots`, `ends`, `invalidEnds`, `shots`, `scored`, `sum`, `x`, `avg`, `min`, `pending` | On `shots`, after every change |
| `shot` | `end`, `endShots`, `total` | Every detected shot |
| `end` | `n`, `valid`; valid: `arrows`, `sum`, `x`, `avg`; invalid: `arrows`, `reason` | An end was closed |
| `confirm` | `end`, `counted`, `entered` | Mismatch, answer with `yes` or `no` |
| `discarded` | `end` | Input discarded, end still open |
| `sessionEnd` | `manual`, `stored`, `epoch`, `slot` | Session ended (`stored:false` for empty sessions) |
| `logStart` | `count`, `epoch` | Start of the log |
| `slot` | `i`, `id`, `ago`, `ends`, `shots`, `scored`, `avg`, `x`, `min`, `invalidEnds`, `invalidArrows`, `mismatch`, `manual` | One per stored session |
| `logEnd` | – | End of the log |
| `level` | `mode` (`off`/`auto`/`on`), `on`, `cal`, `tol`, `active`, `tilt` | On `level`, after changes |
| `cal` | `step`, `state` (`countdown`/`ok`/`error`), `text` | During calibration |
| `event` | `e`: `idle`, `lowbat`, `charge` (+`state`), `light` (+`dark`) | Device events |
| `ack` | `cmd`: `set` (+`k`, `v`), `save`, `live` (+`on`), `mode` (+`mode`), `dfu` | Confirmation of a command |
| `err` | `text` | Error |
| `msg` | `text` | Any other human-readable message |

**Session identity:** `epoch` is a random number of the log, newly created whenever the log starts empty. `id` is a running session number. `epoch` + `id` identify a session uniquely. `ago` is the number of minutes since the session was saved, or `null` after a reboot of the board.

**Why short lines:** long lines (about 1,500 characters) were lost or corrupted over Bluetooth in practice. The settings are therefore sent as one short line per item. No line is longer than about 350 characters.

---

## Troubleshooting

| Problem | Cause and fix |
|---|---|
| App or terminal does not find `UV-Sight` | The board is idle. Move the bow, then search within 10 s. |
| Connection drops | After `timeout` without movement the board disconnects on purpose. Move the bow or raise `timeout`. |
| Settings tab says "Loading settings…" for long | Lines got lost. The app retries three times; move closer and tap *Try again*. |
| Protocol warning in the app | Firmware and app versions do not match. Update both. |
| Status reports keep coming after `live off` | That is the periodic `report`. Set `report 0` to switch it off. |
| LED switches nervously at dusk | Raise `confirm` or widen the gap between `dark_on` and `dark_off`. |
| Shots are not counted | Check `shots` is on and the housing is screwed rigidly. Lower `tap_ths`. |
| Carrying the bow counts as a shot | Raise `tap_ths`. Setting the bow down at the end of a session is handled automatically. |
| Cant indicator never blinks | Check `level`: mode, calibration, and in auto mode a running session. |
| Cant blinks although the bubble is centred | Repeat the calibration or raise `level_tol`. |
| Battery percentage too high while charging | Normal, read it some minutes after unplugging. |
| LED stays off, "BATTERY EMPTY" | Deep-discharge protection. Charge the battery. |
| Settings lost after restart | `save` was missing, or the settings layout changed with a firmware update. |
| Compile error "does not name a type" | Arduino generates function prototypes before the first function; all types must be defined above it. Keep the order of the sketch. |
| Warning "PIN_LSM6DS3TR_C_INT1 not defined" | Board package without the interrupt pin; shot detection falls back to polling and may miss shots. |

---

## Known limitations

- **No clock:** sessions have no timestamp on the device. Dates come from the app and are unknown after a board reboot.
- **Shot detection by impact threshold:** setting the bow down hard can count as a shot. The threshold has to be tuned per bow and mounting position.
- **No Bluetooth security:** anyone in range can connect and change settings while the bow is active.
- **Cold:** below 0 °C LiPo capacity drops noticeably; runtime will be shorter than estimated.
- **Estimates:** runtimes and some thresholds are estimates and were not measured by the author of the code.
- **Web Bluetooth:** only Chrome on Android is supported; iPhone browsers do not support Web Bluetooth.
- **Possible current bug of the chip:** a documented nRF52840 erratum can add up to about 0.4 mA while I2C and pin interrupts are used together. This only applies while the shot counter is on and the bow is active.
