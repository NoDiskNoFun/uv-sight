/*
  ============================================================================
  UV sight illumination + shot counter for compound bows
  Board: Seeed XIAO nRF52840 Sense
  ============================================================================

  Illumination
  ------------
  - Motion detection via the built-in IMU (accelerometer only).
  - If the bow lies still (timeout), everything is off: LED, light sensor, radio.
  - If the bow is moved and it is dark, the UV LED turns on.
  - Brightness is set in percent (perceptual scale).
    Mode "fixed": the LED switches on below dark_on with one brightness and
    off above dark_off.
    Mode "auto": the LED comes on at dark_off with bright_min and gets
    brighter as it gets darker, reaching "bright" at dark_on and staying
    there below. Getting brighter it dims the same way and goes off above
    dark_off.
  - Brightness is held steady over the battery discharge via PWM.
  - Battery level is shown via Bluetooth only ("status").
  - Below the cutoff voltage the LED turns off to protect the battery.
    The percentage display is tied to it: 0 % = cutoff voltage.

  Shot counter
  ------------
  - Shot = impact above the tap threshold (IMU tap detection). The IMU
    signals a tap on its INT1 line (P0.11), which triggers a hardware
    interrupt, so no shot is missed between two loop passes. The interrupt
    is only enabled while the shot counter is on and the bow is active.
    After a shot there is a lockout ("lockout", default 1.5 s): further
    impacts are ignored (vibration) and the tilt indicator pauses.
  - A training session starts with the first shot or with "shots start".
  - A session ends after 1 h without a detected shot or with "shots stop".
    Normal movement does not count.
  - Scores: 0-10 or "x" (inner ten: 10 points, counted separately).
  - A session consists of ends. Each "score" line closes an end and is
    compared with the shots counted since the last entry. On mismatch the
    device asks: yes = accept the entered values (and their arrow count),
    no = discard the input, the end stays open.
  - "score skip" closes an end without scores (invalid, arrow count from
    the sensor, all arrows 0 points, not included in the overall average).
  - Last end without entry at session end: 1 shot is ignored (setting the
    bow down), more than 1 shot = invalid end.
  - 32 sessions are stored (rotating): ends, shots, scored arrows, average,
    invalid ends/arrows, duration.

  Tilt indicator (cant)
  ---------------------
  - If the bow is canted sideways, the LED blinks at reduced brightness,
    regardless of bright/dark. Style "normal": the more tilted, the faster.
    Style "inverted": the closer to level, the faster.
        bright + level = off        dark + level = on
        bright + tilted = blinks    dark + tilted = blinks
  - Only sideways cant counts, not aiming up or down.
  - Three modes: "level off", "level auto" (only during a running session,
    needs "shots on") and "level on" (always while the bow is active).
  - One-time calibration in two steps ("level cal", "level cal2").

  Bluetooth (Android app "Serial Bluetooth Terminal")
  ---------------------------------------------------
  - The board is only visible while it is active (bow moved).
  - A connection does NOT keep the board active: after the timeout without
    movement it is disconnected. Move the bow now and then while configuring.
  - Command "help" lists all commands.

  App mode (JSON protocol)
  ------------------------
  - "app on" switches ALL output to one JSON object per line, "app off"
    back to human-readable text. Disconnecting also switches back.
  - Commands stay the same text commands in both modes.
  - Stored sessions carry an "id" (running number) and the log an "epoch"
    (random, new whenever the log starts empty), so the app can archive
    them without duplicates. "ago" = minutes since saving (null after reboot).
  - Every line has a type field "t": hello, status, cfgStart/cfgItem/cfgEnd,
    session, shot, end,
    confirm, discarded, sessionEnd, logStart/slot/logEnd, level, cal,
    event, ack, err, msg. See the protocol description in the manual.

  Wiring
  ------
  UV LED via NPN transistor (BC547):
    BAT+ --[68 Ohm]-- UV LED anode(+) -- UV LED cathode(-) -- collector
    D6   --[2.2 kOhm]-- base
    GND  -- emitter

  Light sensor:
    D2 --[LDR]--+--[10 kOhm]-- GND
                |
                A0

  Battery: directly on the BAT+ / BAT- pads (voltage is measured internally).
  Charge current: fixed 50 mA (suits the 85 mAh battery, set in code).
  Charge status: P0.17 (board charge LED, LOW = charging) + USB detection.

  Arduino IDE setup
  -----------------
  1. Boards manager URL:
       https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
  2. Install board package "Seeed nRF52 Boards" (NOT "mbed-enabled").
  3. Board: "Seeed XIAO nRF52840 Sense"
  4. Library "Seeed Arduino LSM6DS3" via the library manager.
     (Bluefruit and LittleFS are included in the board package.)
  ============================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <LSM6DS3.h>
#include <bluefruit.h>
#include <nrf_nvic.h>   // sd_nvic_SystemReset
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

#if !defined(PIN_VBAT) || !defined(VBAT_ENABLE)
  #error "Wrong board: select 'Seeed XIAO nRF52840 Sense' from 'Seeed nRF52 Boards' (not mbed)."
#endif

#ifndef PIN_LSM6DS3TR_C_INT1
  #warning "PIN_LSM6DS3TR_C_INT1 not defined: shot detection falls back to polling only."
#endif

// ============================================================================
// FIXED VALUES
// ============================================================================

// Pins (Arduino pin n = XIAO Dn)
const uint8_t PIN_UV      = 6;    // D6 -> 2.2k -> base of BC547
const uint8_t PIN_LDR_PWR = 2;    // D2 powers the LDR only during a measurement
const uint8_t PIN_LDR     = A0;   // midpoint LDR / 10k

// Timing
const uint32_t IDLE_POLL_MS       = 500;   // loop interval while idle
const uint32_t ACTIVE_POLL_MS     = 250;   // loop interval while active
const uint32_t SENSOR_INTERVAL_MS = 2000;  // check light + battery every 2 s
const uint32_t IMU_SETTLE_MS      = 1000;  // ignore motion after a mode change
const uint32_t LEVEL_LOOP_MS      = 30;    // loop interval while the tilt indicator runs

// Circuit
const float R_LED_OHM      = 68.0;
const float V_CE_SAT       = 0.1;
const float VBAT_DIVIDER   = 1510.0 / 510.0;   // internal divider 1M / 510k
const float ADC_MV_PER_LSB = 3600.0 / 4096.0;  // 12 bit, 3.6 V reference

// Battery protection
const float VBAT_RECOVER_OFFSET = 0.25;

// Brightness: percent -> LED current on a perceptual curve
const float    BRIGHT_MAX_MA   = 15.0;  // current at 100 %
const float    BRIGHT_GAMMA    = 2.2;   // 50 % looks about half as bright
const float    BRIGHT_SMOOTH   = 0.35;  // auto mode: share of the change per reading (every 2 s)
const uint8_t  PWM_BITS        = 12;    // fine steps at low brightness
const uint16_t PWM_MAX         = (1u << PWM_BITS) - 1;

// Tilt indicator (adjustable here in the sketch)
const float    TILT_BLINK_MIN_HZ = 1.0;   // blink rate just outside the tolerance
const float    TILT_BLINK_MAX_HZ = 8.0;   // blink rate at strong tilt (max ~10)
const float    TILT_FULL_DEG     = 10.0;  // full blink rate from this tilt on
const float    TILT_HYST_DEG     = 0.3;   // hysteresis against flicker at the tolerance edge
const uint32_t TILT_SMOOTH_MS    = 400;   // smoothing against hand tremor
const float    CAL_MIN_ANGLE_DEG = 15.0;  // minimum angle between calibration steps
const uint32_t CAL_COUNTDOWN_MS  = 3000;  // wait time before a calibration measurement

// Shot counter
const uint8_t  SLOT_COUNT      = 32;
const uint32_t SESSION_IDLE_MS = 60UL * 60UL * 1000UL; // 1 h without a shot = end of session
const uint8_t  MAX_SCORES_LINE = 40;

// Firmware / protocol
const char*    FW_VERSION        = "1.8";
const uint8_t  PROTO_VERSION     = 4;

// Bluetooth
const char*    BLE_NAME          = "UV-Sight";
const uint16_t ADV_FAST_INTERVAL = 160;   // 100 ms (unit 0.625 ms), first 10 s
const uint16_t ADV_SLOW_INTERVAL = 1636;  // ~1 s afterwards
const uint16_t ADV_FAST_TIMEOUT  = 10;    // s
const uint16_t CONN_INT_MIN      = 160;   // 200 ms (unit 1.25 ms)
const uint16_t CONN_INT_MAX      = 320;   // 400 ms
const uint16_t BLE_MTU           = 247;   // max. packet size (default would be 23)
const uint8_t  BLE_SEND_RETRIES  = 50;    // retries per packet when the queue is full
const uint16_t BLE_RETRY_MS      = 20;    // wait between retries

// ============================================================================
// SETTINGS (changeable via Bluetooth, stored in flash with "save")
// ============================================================================
struct Config {
  uint32_t magic;
  uint16_t darkOn;       // LDR value: below = dark
  uint16_t darkOff;      // LDR value: above = bright again
  uint8_t  darkConfirm;  // consecutive readings needed to switch
  uint8_t  wakeThs;      // motion threshold, 1 LSB = 125 mg (1..63)
  uint8_t  tapThs;       // shot threshold,   1 LSB = 250 mg (1..31)
  float    targetMa;     // legacy (firmware < 1.8), only read for migration
  float    vf;           // forward voltage of the UV LED
  float    cutoff;       // battery cutoff voltage
  uint32_t timeoutS;     // s without movement until idle
  uint32_t reportS;      // s between status reports (when connected), 0 = off
  float    tiltMa;       // legacy (firmware < 1.8), only read for migration
  float    levelTol;     // tilt tolerance in degrees
  uint32_t lockoutMs;    // after a shot: ignore impacts + pause tilt indicator, ms
  int8_t   txPower;      // Bluetooth transmit power in dBm
  uint8_t  brightMode;   // 0 = fixed, 1 = auto (follows the light sensor)
  float    brightMax;    // % : fixed brightness, or brightness at dark_on and darker (auto)
  float    brightMin;    // % : auto: brightness at dark_off, where the light starts
  float    tiltOffset;   // %-points below brightMax while tilt-blinking
                         // (new fields are appended at the end so older stored
                         //  settings stay valid and get the default for them)
};

// Increase this number when the structure layout changes,
// old stored values are then discarded.
const uint32_t CFG_MAGIC = 0x55560003;
const char*    CFG_FILE  = "/uvcfg.bin";

const Config DEFAULTS = {
  CFG_MAGIC,
  900,     // darkOn
  1200,    // darkOff
  2,       // darkConfirm
  1,       // wakeThs (125 mg)
  20,      // tapThs (5 g)
  5.0f,    // targetMa
  3.0f,    // vf (measured 2.873 V at ~1 mA + margin)
  3.40f,   // cutoff
  300,     // timeoutS (5 min)
  10,      // reportS
  1.0f,    // tiltMa
  1.0f,    // levelTol (degrees)
  1500,    // lockoutMs
  0,       // txPower (dBm, about 10 m)
  0,       // brightMode (fixed)
  61.0f,   // brightMax (= 5 mA, the former default)
  25.0f,   // brightMin
  32.0f    // tiltOffset (tilt blink at about 1 mA, the former default)
};

Config cfg = DEFAULTS;

// One table for all settings: used by "set", "get" and the JSON cfg list
enum SType : uint8_t { S_U8, S_I8, S_U16, S_U32, S_F };
struct SettingDef {
  const char* key;
  SType       type;
  size_t      off;
  float       mn, mx;
  uint8_t     dec;      // decimals for display
  bool        zeroOk;   // 0 allowed outside the range (= off)
  const char* unit;
  const char* desc;
};

const SettingDef SETTINGS[] = {
  {"dark_on",   S_U16, offsetof(Config, darkOn),      0,    4095, 0, false, "",       "light below = dark"},
  {"dark_off",  S_U16, offsetof(Config, darkOff),     0,    4095, 0, false, "",       "light above = bright"},
  {"confirm",   S_U8,  offsetof(Config, darkConfirm), 1,    10,   0, false, "",       "consecutive readings"},
  {"bright_mode", S_U8, offsetof(Config, brightMode), 0,    1,    0, false, "",       "0 = fixed, 1 = auto (follows the light sensor)"},
  {"bright",    S_F,   offsetof(Config, brightMax),   1,    100,  0, false, "%",      "brightness (fixed), or at dark_on and darker (auto)"},
  {"bright_min", S_F,  offsetof(Config, brightMin),   1,    100,  0, false, "%",      "auto: brightness at dark_off, where the light starts"},
  {"tilt_offset", S_F, offsetof(Config, tiltOffset),  0,    99,   0, false, "%",      "tilt blinking: points below the brightest level"},
  {"vf",        S_F,   offsetof(Config, vf),          2.5,  3.6,  2, false, "V",      "LED forward voltage"},
  {"cutoff",    S_F,   offsetof(Config, cutoff),      3.0,  3.7,  2, false, "V",      "battery cutoff"},
  {"level_tol", S_F,   offsetof(Config, levelTol),    0.2,  10,   1, false, "deg",    "tilt tolerance"},
  {"tap_ths",   S_U8,  offsetof(Config, tapThs),      1,    31,   0, false, "x250mg", "shot threshold"},
  {"ths",       S_U8,  offsetof(Config, wakeThs),     1,    63,   0, false, "x125mg", "motion threshold"},
  {"lockout",   S_U32, offsetof(Config, lockoutMs),   200,  5000, 0, false, "ms",     "after a shot: no counting, no tilt"},
  {"timeout",   S_U32, offsetof(Config, timeoutS),    10,   3600, 0, false, "s",      "without movement until idle"},
  {"report",    S_U32, offsetof(Config, reportS),     2,    600,  0, true,  "s",      "between status reports, 0 = off"},
  {"tx_power",  S_I8,  offsetof(Config, txPower),     -20,  8,    0, false, "dBm",    "radio power, higher = more range and current"},
};
const uint8_t SETTING_COUNT = sizeof(SETTINGS) / sizeof(SETTINGS[0]);

// ============================================================================
// SHOT LOG (stored in flash automatically)
// ============================================================================
#define SLOT_USED     0x01
#define SLOT_MISMATCH 0x02   // at least one end invalid or accepted with "yes" despite mismatch
#define SLOT_MANUAL   0x04

struct ShotSlot {
  uint32_t id;             // running session number (unique per log epoch)
  uint16_t shots;          // total shots (sensor, corrected by "yes")
  uint16_t scores;         // validly scored arrows
  uint16_t avgX100;        // average of valid arrows x 100
  uint16_t minutes;        // duration
  uint16_t ends;           // total ends (valid + invalid)
  uint16_t invalidEnds;    // invalid ends
  uint16_t invalidArrows;  // arrows with 0 points (invalid)
  uint16_t xCount;         // inner tens (X), included in scores
  uint8_t  flags;
  uint8_t  pad;
};

struct ShotLog {
  uint32_t magic;
  uint8_t  shotsOn;   // counter on/off
  uint8_t  next;      // next slot to write
  uint8_t  pad[2];
  uint32_t epoch;     // random id of this log, new whenever the log starts empty
  uint32_t seq;       // last session number handed out
  ShotSlot slots[SLOT_COUNT];
};

const uint32_t LOG_MAGIC = 0x53484F04;
const char*    LOG_FILE  = "/shots.bin";

ShotLog shotLog;

// RAM only: when each slot was saved (lost on reboot -> age unknown)
uint32_t slotSavedMs[SLOT_COUNT];
bool     slotSavedKnown[SLOT_COUNT];

// ============================================================================
// TILT: calibration and on/off (stored in flash automatically)
// ============================================================================
// Tilt indicator modes (value of LevelState.on; 1 = auto keeps older
// stored settings, where 1 meant "on during sessions", unchanged)
#define LEVEL_OFF  0
#define LEVEL_AUTO 1
#define LEVEL_ON   2

struct LevelState {
  uint32_t magic;
  uint8_t  on;        // LEVEL_OFF / LEVEL_AUTO / LEVEL_ON
  uint8_t  calibrated;
  uint8_t  style;     // 0 = normal (faster when more tilted), 1 = inverted
  uint8_t  pad;
  float    L[3];   // lateral axis (aiming up/down rotates around this axis)
  float    D[3];   // gravity direction with the bow level and horizontal
};

const uint32_t LEVEL_MAGIC = 0x4C564C01;
const char*    LEVEL_FILE  = "/level.bin";

LevelState lvl;

bool  calHaveG0     = false;   // step 1 done
float calG0[3]      = {0, 0, 0};
float tiltF[3]      = {0, 0, 0};
bool  tiltFilterOk  = false;
bool  tilted        = false;
float tiltDeg       = 0;

// Running session (RAM only)
bool     sessionActive     = false;
uint32_t sessionStartMs    = 0;
uint32_t lastShotMs        = 0;   // for the lockout (also set after a mode change)
uint32_t sessionLastShotMs = 0;   // last real shot or manual start
uint16_t shotCount         = 0;   // shots of the whole session
uint16_t endShots          = 0;   // shots of the open end
uint16_t endCount          = 0;   // closed ends (valid + invalid)
uint16_t invalidEnds       = 0;
uint16_t invalidArrows     = 0;
uint16_t scoreCount        = 0;   // validly scored arrows
uint32_t scoreSum          = 0;
uint16_t xCount            = 0;   // inner tens (X) of the session
bool     mismatchAccepted  = false;

// Pending confirmation for a mismatching end
bool     pendingScore      = false;
uint8_t  pendingN          = 0;
uint8_t  pendingVals[MAX_SCORES_LINE];
uint8_t  pendingX          = 0;

// ============================================================================
// LSM6DS3TR-C registers
// ============================================================================
#define REG_CTRL1_XL    0x10
#define REG_CTRL2_G     0x11
#define REG_CTRL6_C     0x15
#define REG_WAKE_UP_SRC 0x1B
#define REG_TAP_SRC     0x1C
#define REG_TAP_CFG     0x58
#define REG_TAP_THS_6D  0x59
#define REG_INT_DUR2    0x5A
#define REG_WAKE_UP_THS 0x5B
#define REG_WAKE_UP_DUR 0x5C
#define REG_MD1_CFG     0x5E

// CTRL1_XL: ODR | full scale +-8 g (0x0C)
#define XL_26HZ_8G      0x2C
#define XL_416HZ_8G     0x6C
// TAP_CFG: interrupts on (0x80), HP filter (0x10), latched (0x01), tap XYZ (0x0E)
#define TAP_CFG_MOTION  0x91
#define TAP_CFG_SHOTS   0x9F

LSM6DS3 imu(I2C_MODE, 0x6A);
BLEUart bleuart;

// ============================================================================
// State
// ============================================================================
enum LedMode  { MODE_AUTO, MODE_ON, MODE_OFF };
enum LedState { LS_OFF, LS_ON, LS_BLINK };

bool     imuOk             = false;
bool     imuFast           = false;
bool     uvPwmActive       = false;
uint16_t curDuty           = 0;
LedState ledState          = LS_OFF;
uint32_t blinkCycleStart   = 0;   // start of the current blink cycle
uint32_t blinkPeriodMs     = 0;   // length of the current blink cycle
bool     isDark            = false;
bool     lowBatLock        = false;
bool     wasInactive       = true;
bool     liveMode          = false;
bool     welcomed          = false;
bool     appMode           = false;   // true: output is JSON only
LedMode  ledMode           = MODE_AUTO;
uint8_t  darkCount         = 0;
uint8_t  brightCount       = 0;
uint16_t lastLdr           = 0;
float    lastVbat          = 0;
float    lastPct           = 0;
uint16_t dutyFull          = 0;
uint16_t dutyTilt          = 0;
float    brightNow         = -1;  // current steady brightness in %, smoothed (-1 = not set)
uint32_t lastMotionMs      = 0;
uint32_t lastSensorMs      = 0;
uint32_t lastReportMs      = 0;
uint32_t ignoreMotionUntil = 0;
volatile bool     tapFlag  = false;   // set by the INT1 interrupt
volatile uint32_t tapMs    = 0;       // time of the first tap since the last check
String   rxBuf;
uint16_t bleConnHandle     = BLE_CONN_HANDLE_INVALID;

enum ChargeState { CHG_NO_USB, CHG_CHARGING, CHG_FULL };
ChargeState lastCharge     = CHG_NO_USB;

// ============================================================================
// Settings table access
// ============================================================================
float getSetting(const Config& c, const SettingDef& d) {
  const uint8_t* p = (const uint8_t*)&c + d.off;
  switch (d.type) {
    case S_U8:  return *p;
    case S_I8:  return (int8_t)*p;
    case S_U16: { uint16_t x; memcpy(&x, p, 2); return x; }
    case S_U32: { uint32_t x; memcpy(&x, p, 4); return x; }
    default:    { float x;    memcpy(&x, p, 4); return x; }
  }
}

void setSetting(Config& c, const SettingDef& d, float v) {
  uint8_t* p = (uint8_t*)&c + d.off;
  switch (d.type) {
    case S_U8:  *p = (uint8_t)lroundf(v); break;
    case S_I8:  { int8_t x = (int8_t)lroundf(v); memcpy(p, &x, 1); break; }
    case S_U16: { uint16_t x = (uint16_t)lroundf(v); memcpy(p, &x, 2); break; }
    case S_U32: { uint32_t x = (uint32_t)lroundf(v); memcpy(p, &x, 4); break; }
    default:    { memcpy(p, &v, 4); break; }
  }
}

String fmtSetting(const SettingDef& d, float v) {
  return (d.type == S_F) ? String(v, (unsigned int)d.dec) : String((long)lroundf(v));
}

const SettingDef* findSetting(const String& key) {
  for (uint8_t i = 0; i < SETTING_COUNT; i++) {
    if (key == SETTINGS[i].key) return &SETTINGS[i];
  }
  return nullptr;
}

// The radio only supports certain power steps: use the nearest one
int8_t snapTxPower(int8_t v) {
  static const int8_t steps[] = { -20, -16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, 8 };
  int8_t best = steps[0];
  for (int8_t s : steps) if (abs(s - v) < abs(best - v)) best = s;
  return best;
}

void applyTxPower() {
  cfg.txPower = snapTxPower(cfg.txPower);
  Bluefruit.setTxPower(cfg.txPower);
  if (Bluefruit.Advertising.isRunning()) {   // refresh the advertised power value
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.start(0);
  }
}

// ============================================================================
// Output (USB + Bluetooth)
// ============================================================================
// Sends text in packets matching the negotiated size. If the send queue is
// full, waits briefly and retries instead of dropping the packet.
void bleSend(const String& s) {
  if (bleConnHandle == BLE_CONN_HANDLE_INVALID) return;
  if (!Bluefruit.connected(bleConnHandle) || !bleuart.notifyEnabled(bleConnHandle)) return;

  BLEConnection* conn = Bluefruit.Connection(bleConnHandle);
  uint16_t chunk = conn ? conn->getMtu() - 3 : 20;
  if (chunk < 20) chunk = 20;

  const uint8_t* p   = (const uint8_t*)s.c_str();
  const size_t   len = s.length();
  size_t off = 0;

  while (off < len) {
    const size_t n = (len - off < chunk) ? (len - off) : chunk;
    bool ok = false;
    for (uint8_t r = 0; r < BLE_SEND_RETRIES && !ok; r++) {
      ok = bleuart.write(bleConnHandle, p + off, n) == n;
      if (!ok) {
        if (!Bluefruit.connected(bleConnHandle)) return;
        delay(BLE_RETRY_MS);
      }
    }
    if (!ok) return;   // connection stuck, drop the rest
    off += n;
  }
}

void sendLine(const String& s) {
  if (Serial) Serial.println(s);
  bleSend(s + "\n");
}

String jsonEsc(const String& s) {
  String r;
  r.reserve(s.length() + 8);
  for (unsigned int i = 0; i < s.length(); i++) {
    const char c = s[i];
    if (c == '"' || c == '\\') { r += '\\'; r += c; }
    else if ((uint8_t)c < 0x20)  r += ' ';
    else                         r += c;
  }
  return r;
}
String jstr(const String& s) { return "\"" + jsonEsc(s) + "\""; }
String jbool(bool b)         { return b ? "true" : "false"; }

// Human-readable message; in app mode wrapped as {"t":"msg"}
void out(const String& s) {
  if (appMode) sendLine("{\"t\":\"msg\",\"text\":" + jstr(s) + "}");
  else         sendLine(s);
}

// Error message; in app mode wrapped as {"t":"err"}
void err(const String& s) {
  if (appMode) sendLine("{\"t\":\"err\",\"text\":" + jstr(s) + "}");
  else         sendLine(s);
}

// Extra structured line, only sent in app mode
void emit(const String& json) {
  if (appMode) sendLine(json);
}

// Human text in text mode, structured JSON in app mode
void say(const String& human, const String& json) {
  sendLine(appMode ? json : human);
}

// ============================================================================
// Flash: settings and log
// ============================================================================
// Perceptual brightness: percent <-> average LED current
float percentToMa(float p) {
  if (p < 0)   p = 0;
  if (p > 100) p = 100;
  return BRIGHT_MAX_MA * powf(p / 100.0f, BRIGHT_GAMMA);
}
float maToPercent(float ma) {
  if (ma <= 0) return 1;
  float p = 100.0f * powf(ma / BRIGHT_MAX_MA, 1.0f / BRIGHT_GAMMA);
  return p < 1 ? 1 : (p > 100 ? 100 : roundf(p));
}

void cfgLoad() {
  cfg = DEFAULTS;
  File f(InternalFS);
  if (f.open(CFG_FILE, FILE_O_READ)) {
    // Start from the defaults: fields missing in an older, shorter file keep them
    Config tmp = DEFAULTS;
    const int n = f.read(&tmp, sizeof(tmp));
    if (tmp.magic == CFG_MAGIC && n >= (int)offsetof(Config, lockoutMs)) {
      // Settings from before firmware 1.8: convert the mA values to percent
      if (n <= (int)offsetof(Config, brightMode)) {
        tmp.brightMax  = maToPercent(tmp.targetMa);
        tmp.tiltOffset = tmp.brightMax - maToPercent(tmp.tiltMa);
        if (tmp.tiltOffset < 0) tmp.tiltOffset = 0;
      }
      cfg = tmp;
    }
    f.close();
  }
}

bool cfgSave() {
  InternalFS.remove(CFG_FILE);
  File f(InternalFS);
  if (!f.open(CFG_FILE, FILE_O_WRITE)) return false;
  f.write((const uint8_t*)&cfg, sizeof(cfg));
  f.close();
  return true;
}

void logLoad() {
  memset(&shotLog, 0, sizeof(shotLog));
  shotLog.magic = LOG_MAGIC;
  File f(InternalFS);
  if (f.open(LOG_FILE, FILE_O_READ)) {
    ShotLog tmp;
    if (f.read(&tmp, sizeof(tmp)) == (int)sizeof(tmp) &&
        tmp.magic == LOG_MAGIC && tmp.next < SLOT_COUNT) {
      shotLog = tmp;
    }
    f.close();
  }
}

bool logSave() {
  InternalFS.remove(LOG_FILE);
  File f(InternalFS);
  if (!f.open(LOG_FILE, FILE_O_WRITE)) return false;
  f.write((const uint8_t*)&shotLog, sizeof(shotLog));
  f.close();
  return true;
}

// Give a fresh log a random epoch, so the app can tell logs apart
void ensureLogEpoch() {
  if (shotLog.epoch != 0) return;
  uint32_t e = 0;
  for (uint8_t tries = 0; tries < 50 && e == 0; tries++) {
    if (sd_rand_application_vector_get((uint8_t*)&e, sizeof(e)) != 0) e = 0;
    if (e == 0) delay(2);
  }
  if (e == 0) e = millis() | 1;   // fallback, should not happen
  shotLog.epoch = e;
}

void levelLoad() {
  memset(&lvl, 0, sizeof(lvl));
  lvl.magic = LEVEL_MAGIC;
  File f(InternalFS);
  if (f.open(LEVEL_FILE, FILE_O_READ)) {
    LevelState tmp;
    if (f.read(&tmp, sizeof(tmp)) == (int)sizeof(tmp) && tmp.magic == LEVEL_MAGIC) {
      lvl = tmp;
    }
    f.close();
  }
}

bool levelSave() {
  InternalFS.remove(LEVEL_FILE);
  File f(InternalFS);
  if (!f.open(LEVEL_FILE, FILE_O_WRITE)) return false;
  f.write((const uint8_t*)&lvl, sizeof(lvl));
  f.close();
  return true;
}

// ============================================================================
// IMU
// ============================================================================
void imuSetThs(uint8_t ths)    { if (imuOk) imu.writeRegister(REG_WAKE_UP_THS, ths & 0x3F); }
void imuSetTapThs(uint8_t ths) { if (imuOk) imu.writeRegister(REG_TAP_THS_6D,  ths & 0x1F); }

bool imuInit() {
#ifdef PIN_LSM6DS3TR_C_POWER
  pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
  digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
  delay(10);
#endif

  imu.settings.gyroEnabled  = 0;
  imu.settings.accelEnabled = 1;
  if (imu.begin() != IMU_SUCCESS) return false;

  imu.writeRegister(REG_CTRL2_G,     0x00);               // gyro off
  imu.writeRegister(REG_CTRL1_XL,    XL_26HZ_8G);         // 26 Hz, +-8 g
  imu.writeRegister(REG_CTRL6_C,     0x10);               // low-power mode
  imu.writeRegister(REG_WAKE_UP_DUR, 0x00);
  imu.writeRegister(REG_WAKE_UP_THS, cfg.wakeThs & 0x3F); // bit7=0: single tap only
  imu.writeRegister(REG_TAP_THS_6D,  cfg.tapThs & 0x1F);
  imu.writeRegister(REG_INT_DUR2,    0x07);               // max shock window, short quiet
  imu.writeRegister(REG_TAP_CFG,     TAP_CFG_MOTION);     // tap off for now
  imu.writeRegister(REG_MD1_CFG,     0x40);               // only single tap on INT1 (motion is polled)

  uint8_t dummy;
  imu.readRegister(&dummy, REG_WAKE_UP_SRC);
  imu.readRegister(&dummy, REG_TAP_SRC);
  imuFast = false;
  return true;
}

// INT1 interrupt: only remember the time, evaluation happens in the loop
void imuInt1Isr() {
  if (!tapFlag) tapMs = millis();
  tapFlag = true;
}

// Fast mode (416 Hz) only for shot detection, otherwise low power (26 Hz)
void imuSetFast(bool fast) {
  if (!imuOk || fast == imuFast) return;

#ifdef PIN_LSM6DS3TR_C_INT1
  if (!fast) detachInterrupt(digitalPinToInterrupt(PIN_LSM6DS3TR_C_INT1));
#endif

  if (fast) {
    imu.writeRegister(REG_CTRL6_C,  0x00);                // high performance
    imu.writeRegister(REG_CTRL1_XL, XL_416HZ_8G);
    imu.writeRegister(REG_TAP_CFG,  TAP_CFG_SHOTS);
  } else {
    imu.writeRegister(REG_TAP_CFG,  TAP_CFG_MOTION);
    imu.writeRegister(REG_CTRL1_XL, XL_26HZ_8G);
    imu.writeRegister(REG_CTRL6_C,  0x10);
  }
  imuFast = fast;

  // Let the filters settle, discard false triggers
  const uint32_t now = millis();
  ignoreMotionUntil = now + IMU_SETTLE_MS;
  lastShotMs = now;
  uint8_t dummy;
  imu.readRegister(&dummy, REG_WAKE_UP_SRC);
  imu.readRegister(&dummy, REG_TAP_SRC);   // clears the latched INT1 line
  tapFlag = false;

#ifdef PIN_LSM6DS3TR_C_INT1
  if (fast) {
    pinMode(PIN_LSM6DS3TR_C_INT1, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_LSM6DS3TR_C_INT1), imuInt1Isr, RISING);
  }
#endif
}

bool imuMotionSinceLastCheck() {
  if (!imuOk) return true;
  uint8_t src = 0;
  if (imu.readRegister(&src, REG_WAKE_UP_SRC) != IMU_SUCCESS) return true;
  if ((int32_t)(millis() - ignoreMotionUntil) < 0) return false;
  return (src & 0x08) != 0;  // WU_IA
}

// Reads (and thereby clears) the latched tap source. Serves as a fallback
// if the interrupt is not available, and re-arms INT1 for the next tap.
bool imuTapSinceLastCheck() {
  if (!imuOk || !imuFast) return false;
  uint8_t src = 0;
  if (imu.readRegister(&src, REG_TAP_SRC) != IMU_SUCCESS) return false;
  return (src & 0x60) != 0;  // TAP_IA or SINGLE_TAP
}

// Raw accelerometer values (scale irrelevant, only the direction matters)
bool readAccel(float v[3]) {
  if (!imuOk) return false;
  uint8_t b[6];
  if (imu.readRegisterRegion(b, 0x28, 6) != IMU_SUCCESS) return false;  // OUTX_L_XL..
  for (uint8_t i = 0; i < 3; i++) {
    v[i] = (float)(int16_t)(b[2 * i] | (b[2 * i + 1] << 8));
  }
  return true;
}

// ============================================================================
// Charge status
// ============================================================================
const uint8_t CHG_PIN = 17;   // P0.17, CHG output of the BQ25101 (charge LED)

void chargeInit() {
  // Input with pull-up: open (not charging) = HIGH, charging = LOW
  NRF_P0->PIN_CNF[CHG_PIN] =
      (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos)   |
      (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) |
      (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos);
}

bool usbPresent() {
  uint32_t reg = 0;
  sd_power_usbregstatus_get(&reg);   // via SoftDevice, since BLE is active
  return (reg & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}

ChargeState readCharge() {
  if (!usbPresent()) return CHG_NO_USB;
  const bool charging = ((NRF_P0->IN >> CHG_PIN) & 1) == 0;
  return charging ? CHG_CHARGING : CHG_FULL;
}

const char* chargeText(ChargeState c) {
  switch (c) {
    case CHG_CHARGING: return "charging";
    case CHG_FULL:     return "full";
    default:           return "no USB";
  }
}

// ============================================================================
// Sensors
// ============================================================================
uint16_t readLdr() {
  digitalWrite(PIN_LDR_PWR, HIGH);
  delay(2);
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 8; i++) sum += analogRead(PIN_LDR);
  digitalWrite(PIN_LDR_PWR, LOW);
  return sum / 8;
}

float readVbat() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 16; i++) sum += analogRead(PIN_VBAT);
  return (sum / 16.0f) * ADC_MV_PER_LSB / 1000.0f * VBAT_DIVIDER;
}

// Rough LiPo discharge curve (voltage -> percent of total capacity)
float rawPercent(float v) {
  static const float tbl[][2] = {
    {4.20, 100}, {4.10, 90}, {4.00, 78}, {3.90, 65}, {3.80, 50},
    {3.75, 40},  {3.70, 30}, {3.65, 20}, {3.55, 10}, {3.40, 3},
    {3.30, 1},   {3.00, 0}
  };
  const uint8_t n = sizeof(tbl) / sizeof(tbl[0]);
  if (v >= tbl[0][0]) return 100;
  if (v <= tbl[n - 1][0]) return 0;
  for (uint8_t i = 0; i < n - 1; i++) {
    if (v >= tbl[i + 1][0]) {
      float f = (v - tbl[i + 1][0]) / (tbl[i][0] - tbl[i + 1][0]);
      return tbl[i + 1][1] + f * (tbl[i][1] - tbl[i + 1][1]);
    }
  }
  return 0;
}

// Displayed percent: 0 % = cutoff voltage, 100 % = 4.2 V
float vbatToPercent(float v) {
  const float lo = rawPercent(cfg.cutoff);
  if (lo >= 99.0f) return 0;
  float p = (rawPercent(v) - lo) / (100.0f - lo) * 100.0f;
  if (p < 0)   p = 0;
  if (p > 100) p = 100;
  return p;
}

// ============================================================================
// UV LED
// ============================================================================
uint16_t dutyForVbat(float vbat, float targetMa) {
  float peakMa = (vbat - cfg.vf - V_CE_SAT) / R_LED_OHM * 1000.0f;
  if (peakMa <= targetMa) return PWM_MAX;
  float duty = targetMa / peakMa * PWM_MAX;
  if (duty < 1) duty = 1;
  return (uint16_t)duty;
}

// Target brightness for the steady light.
// Auto: bright_min at dark_off, rising to "bright" at dark_on and staying there
// below. The sensor responds roughly logarithmically, so the blend is too.
float brightTarget() {
  if (cfg.brightMode == 0) return cfg.brightMax;
  const float full  = (float)(cfg.darkOn  > 0 ? cfg.darkOn  : 1);   // full brightness from here down
  const float start = (float)(cfg.darkOff > 0 ? cfg.darkOff : 1);   // light starts here
  if (start <= full) return cfg.brightMax;
  const float x = (float)(lastLdr > 0 ? lastLdr : 1);
  float t = (logf(start) - logf(x)) / (logf(start) - logf(full));   // 0 at dark_off, 1 at dark_on
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  return cfg.brightMin + t * (cfg.brightMax - cfg.brightMin);
}

// Light-sensor thresholds for switching on / off.
// Fixed: on below dark_on, off above dark_off (the gap prevents flicker).
// Auto: the light already starts at dark_off; a small gap 10 % into the
// range keeps it from flickering there.
uint16_t lightOnThreshold() {
  if (cfg.brightMode == 0 || cfg.darkOff <= cfg.darkOn) return cfg.darkOn;
  return cfg.darkOff - (cfg.darkOff - cfg.darkOn) / 10;
}

// Tilt blinking: fixed offset below the brightest level in darkness
float tiltBrightness() {
  float p = cfg.brightMax - cfg.tiltOffset;
  return p < 1 ? 1 : p;
}

// Set the PWM value (0 = dark, but PWM stays active -> used for blinking)
void uvWrite(uint16_t duty) {
  if (uvPwmActive && duty == curDuty) return;
  analogWrite(PIN_UV, duty);
  uvPwmActive = true;
  curDuty = duty;
}

// Fully off, release the PWM clock
void uvOff() {
  if (!uvPwmActive) return;
  analogWrite(PIN_UV, 0);
  for (uint8_t i = 0; i < HWPWM_MODULE_NUM; i++) {
    if (HwPWMx[i]->removePin(PIN_UV)) break;
  }
  pinMode(PIN_UV, OUTPUT);
  digitalWrite(PIN_UV, LOW);
  uvPwmActive = false;
  curDuty = 0;
}

// ============================================================================
// Tilt
// ============================================================================
float vdot(const float a[3], const float b[3]) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }

bool vnorm(float v[3]) {
  float n = sqrtf(vdot(v, v));
  if (n < 1e-3f) return false;
  for (uint8_t i = 0; i < 3; i++) v[i] /= n;
  return true;
}

void tiltReset() {
  tiltFilterOk = false;
  tilted = false;
  tiltDeg = 0;
}

const char* levelModeText() {
  return lvl.on == LEVEL_ON ? "on" : (lvl.on == LEVEL_AUTO ? "auto" : "off");
}

// Tilt indicator: "on" always, "auto" only during a running session
bool levelActive() {
  if (!lvl.calibrated || !imuOk) return false;
  if (lvl.on == LEVEL_ON)   return true;
  if (lvl.on == LEVEL_AUTO) return sessionActive;
  return false;
}

// Compute the cant: component of gravity along the lateral axis.
// Aiming up/down rotates around the lateral axis and does not change the value.
void updateTilt() {
  float a[3];
  if (!readAccel(a)) return;

  if (!tiltFilterOk) {
    for (uint8_t i = 0; i < 3; i++) tiltF[i] = a[i];
    tiltFilterOk = true;
  } else {
    const float alpha = (float)LEVEL_LOOP_MS / (float)TILT_SMOOTH_MS;
    for (uint8_t i = 0; i < 3; i++) tiltF[i] += alpha * (a[i] - tiltF[i]);
  }

  const float gL = vdot(tiltF, lvl.L);
  const float gD = vdot(tiltF, lvl.D);
  tiltDeg = fabsf(atan2f(gL, gD)) * 57.29578f;

  if (!tilted && tiltDeg > cfg.levelTol)                      tilted = true;
  else if (tilted && tiltDeg < cfg.levelTol - TILT_HYST_DEG)  tilted = false;
}

float blinkHz() {
  float span = TILT_FULL_DEG - cfg.levelTol;
  float f = (span <= 0) ? 1.0f : (tiltDeg - cfg.levelTol) / span;
  if (f < 0) f = 0;
  if (f > 1) f = 1;
  if (lvl.style == 1) f = 1.0f - f;   // inverted: fastest just outside the tolerance
  return TILT_BLINK_MIN_HZ + f * (TILT_BLINK_MAX_HZ - TILT_BLINK_MIN_HZ);
}

// Average the gravity direction over ~0.6 s
bool sampleGravity(float g[3]) {
  float s[3] = {0, 0, 0};
  uint8_t n = 0;
  for (uint8_t k = 0; k < 32; k++) {
    float a[3];
    if (readAccel(a)) { for (uint8_t i = 0; i < 3; i++) s[i] += a[i]; n++; }
    delay(20);
  }
  if (n < 16) return false;
  for (uint8_t i = 0; i < 3; i++) g[i] = s[i] / n;
  return true;
}

String levelJson() {
  return String("{\"t\":\"level\",\"mode\":\"") + levelModeText() + "\",\"on\":" + jbool(lvl.on != LEVEL_OFF) +
         ",\"style\":\"" + String(lvl.style == 1 ? "inverted" : "normal") + "\"" +
         ",\"cal\":" + jbool(lvl.calibrated) +
         ",\"tol\":" + String(cfg.levelTol, 1) + ",\"active\":" + jbool(levelActive()) +
         ",\"tilt\":" + ((levelActive() && tiltFilterOk) ? String(tiltDeg, 1) : String("null")) + "}";
}

String calJson(uint8_t step, const char* state, const String& text) {
  return "{\"t\":\"cal\",\"step\":" + String(step) + ",\"state\":\"" + state +
         "\",\"text\":" + jstr(text) + "}";
}

void calCountdown(uint8_t step) {
  const String txt = "Measuring in " + String(CAL_COUNTDOWN_MS / 1000) + " s - hold the bow still...";
  say(txt, calJson(step, "countdown", txt));
  delay(CAL_COUNTDOWN_MS);
}

void levelCal1() {
  if (!imuOk) { say("IMU not available.", calJson(1, "error", "IMU not available.")); return; }
  calCountdown(1);
  if (!sampleGravity(calG0) || !vnorm(calG0)) {
    say("Measurement failed.", calJson(1, "error", "Measurement failed."));
    return;
  }
  calHaveG0 = true;
  const String txt = "Step 1 ok. Now keep the bow LEVEL but aim clearly up or down "
                     "(at least 20 degrees), then send 'level cal2'.";
  say(txt, calJson(1, "ok", txt));
}

void levelCal2() {
  if (!calHaveG0) { say("Do step 1 first: 'level cal'.", calJson(2, "error", "Do step 1 first.")); return; }
  calCountdown(2);
  float g1[3];
  if (!sampleGravity(g1) || !vnorm(g1)) {
    say("Measurement failed.", calJson(2, "error", "Measurement failed."));
    return;
  }

  const float angle = acosf(constrain(vdot(calG0, g1), -1.0f, 1.0f)) * 57.29578f;
  if (angle < CAL_MIN_ANGLE_DEG) {
    const String txt = "Not tilted enough (" + String(angle, 1) +
                       " degrees). Aim further up/down and repeat 'level cal2'.";
    say(txt, calJson(2, "error", txt));
    return;
  }

  // Lateral axis = rotation axis between the two measurements
  float L[3] = {
    calG0[1] * g1[2] - calG0[2] * g1[1],
    calG0[2] * g1[0] - calG0[0] * g1[2],
    calG0[0] * g1[1] - calG0[1] * g1[0]
  };
  if (!vnorm(L)) {
    say("Calculation failed, please repeat.", calJson(2, "error", "Calculation failed, please repeat."));
    return;
  }

  for (uint8_t i = 0; i < 3; i++) { lvl.L[i] = L[i]; lvl.D[i] = calG0[i]; }
  lvl.calibrated = 1;
  calHaveG0 = false;
  tiltReset();
  const bool ok = levelSave();
  say(ok ? "Calibration saved." : "ERROR while saving!",
      calJson(2, ok ? "ok" : "error", ok ? "Calibration saved." : "ERROR while saving!"));
  emit(levelJson());
}

void setLevelMode(uint8_t mode, const char* text) {
  lvl.on = mode;
  if (mode == LEVEL_OFF) tiltReset();
  if (!levelSave()) { err("ERROR while saving!"); return; }
  say(String(text) + (lvl.calibrated || mode == LEVEL_OFF ? "" : " Note: not calibrated yet ('level cal')."),
      levelJson());
}

void handleLevel(const String& arg) {
  if (arg == "") {
    if (appMode) { sendLine(levelJson()); return; }
    String m = lvl.on == LEVEL_ON   ? "on (always)" :
               lvl.on == LEVEL_AUTO ? "auto (during sessions)" : "off";
    out("Tilt indicator: " + m + (lvl.style ? ", inverted" : ", normal") +
        (lvl.calibrated ? ", calibrated" : ", NOT calibrated") +
        ", tolerance " + String(cfg.levelTol, 1) + " degrees" +
        (levelActive() ? ", active now" : ", inactive now"));
    if (levelActive() && tiltFilterOk) out("Current tilt: " + String(tiltDeg, 1) + " degrees");
  }
  else if (arg == "on")   setLevelMode(LEVEL_ON,   "Tilt indicator on (always).");
  else if (arg == "auto") setLevelMode(LEVEL_AUTO, "Tilt indicator auto (during sessions).");
  else if (arg == "off")  setLevelMode(LEVEL_OFF,  "Tilt indicator off.");
  else if (arg == "style normal" || arg == "style inverted") {
    lvl.style = (arg == "style inverted") ? 1 : 0;
    if (!levelSave()) { err("ERROR while saving!"); return; }
    say(String("Tilt blinking: ") + (lvl.style ? "inverted (faster when closer to level)."
                                               : "normal (faster when more tilted)."), levelJson());
  }
  else if (arg == "cal")  levelCal1();
  else if (arg == "cal2") levelCal2();
  else err("Unknown. Possible: level, level on|auto|off|cal|cal2, level style normal|inverted");
}

// ============================================================================
// LED output (runs on every loop pass)
// ============================================================================
void updateLed(uint32_t now) {
  // After a shot the arrow is gone: pause the tilt indicator for the lockout time
  const bool tiltPause = (int32_t)(now - lastShotMs) < (int32_t)cfg.lockoutMs;

  LedState st;
  if (lowBatLock || ledMode == MODE_OFF)          st = LS_OFF;
  else if (ledMode == MODE_ON)                    st = LS_ON;
  else if (levelActive() && tilted && !tiltPause) st = LS_BLINK;
  else if (isDark)                        st = LS_ON;
  else                                    st = LS_OFF;

  switch (st) {
    case LS_OFF:
      uvOff();
      break;
    case LS_ON:
      uvWrite(dutyFull);
      break;
    case LS_BLINK: {
      // Only pick a new rate at the start of a cycle so the rhythm
      // stays steady (on first, then off).
      if (ledState != LS_BLINK || (now - blinkCycleStart) >= blinkPeriodMs) {
        blinkCycleStart = now;
        blinkPeriodMs   = (uint32_t)(1000.0f / blinkHz());
      }
      const bool phaseOn = (now - blinkCycleStart) < (blinkPeriodMs / 2);
      uvWrite(phaseOn ? dutyTilt : 0);
      break;
    }
  }
  ledState = st;
}

// ============================================================================
// Shot counter
// ============================================================================
String fmtAvg(uint32_t sum, uint16_t count) {
  if (count == 0) return "0.00";
  return String((float)sum / count, 2);
}

String sessionJson() {
  String j = "{\"t\":\"session\",\"counter\":" + jbool(shotLog.shotsOn) +
             ",\"active\":" + jbool(sessionActive);
  if (sessionActive) {
    // epoch + nextId = the key this session will get in the log, so the app
    // can remember its start time even if the board restarts before import
    j += ",\"epoch\":" + String(shotLog.epoch) + ",\"nextId\":" + String(shotLog.seq + 1) +
         ",\"end\":" + String(endCount + 1) + ",\"endShots\":" + String(endShots) +
         ",\"ends\":" + String(endCount) + ",\"invalidEnds\":" + String(invalidEnds) +
         ",\"shots\":" + String(shotCount) + ",\"scored\":" + String(scoreCount) +
         ",\"sum\":" + String(scoreSum) + ",\"x\":" + String(xCount) +
         ",\"avg\":" + fmtAvg(scoreSum, scoreCount) +
         ",\"min\":" + String((millis() - sessionStartMs) / 60000UL) +
         ",\"pending\":" + jbool(pendingScore);
  }
  return j + "}";
}

void startSession(uint32_t now) {
  ensureLogEpoch();                 // the session's future log key must be known now
  sessionActive     = true;
  sessionStartMs    = now;
  sessionLastShotMs = now;
  shotCount = endShots = endCount = 0;
  invalidEnds = invalidArrows = scoreCount = xCount = 0;
  scoreSum = 0;
  mismatchAccepted = false;
  pendingScore = false;
}

void registerShot(uint32_t now) {
  if (!shotLog.shotsOn) return;
  // Signed compare: an interrupt timestamp can be slightly older than lastShotMs
  if ((int32_t)(now - lastShotMs) < (int32_t)cfg.lockoutMs) return;  // ignore vibration
  lastShotMs = now;
  if (!sessionActive) {
    startSession(now);
    say("Session started automatically.", sessionJson());
  }
  shotCount++;
  endShots++;
  sessionLastShotMs = now;
  say("Shot detected (end " + String(endCount + 1) + ": " + String(endShots) + ")",
      "{\"t\":\"shot\",\"end\":" + String(endCount + 1) + ",\"endShots\":" + String(endShots) +
      ",\"total\":" + String(shotCount) + "}");
}

// Close an end with scores
void closeEndValid(const uint8_t* vals, uint8_t n, uint8_t xn) {
  uint16_t sum = 0;
  for (uint8_t k = 0; k < n; k++) sum += vals[k];
  scoreSum   += sum;
  scoreCount += n;
  xCount     += xn;
  endCount++;
  endShots = 0;
  say("End " + String(endCount) + " scored: " + String(n) + " arrows, " + String(sum) +
      " points (avg " + fmtAvg(sum, n) + ", " + String(xn) + " X). Session: " + String(scoreCount) +
      " scored, avg " + fmtAvg(scoreSum, scoreCount) + ", " + String(xCount) + " X",
      "{\"t\":\"end\",\"n\":" + String(endCount) + ",\"valid\":true,\"arrows\":" + String(n) +
      ",\"sum\":" + String(sum) + ",\"x\":" + String(xn) + ",\"avg\":" + fmtAvg(sum, n) + "}");
  emit(sessionJson());
}

// Close an end as invalid: all arrows 0 points, not in the overall average
void closeEndInvalid(uint16_t arrows, const String& reason) {
  endCount++;
  invalidEnds++;
  invalidArrows += arrows;
  endShots = 0;
  say("End " + String(endCount) + " invalid (" + reason + "): " + String(arrows) +
      " arrows stored with 0 points, not included in the overall average.",
      "{\"t\":\"end\",\"n\":" + String(endCount) + ",\"valid\":false,\"arrows\":" + String(arrows) +
      ",\"reason\":" + jstr(reason) + "}");
  emit(sessionJson());
}

String slotJson(const ShotSlot& s, uint8_t i, int16_t idx) {
  String ago = "null";
  if (idx >= 0 && slotSavedKnown[idx]) ago = String((millis() - slotSavedMs[idx]) / 60000UL);
  return "{\"t\":\"slot\",\"i\":" + String(i) + ",\"id\":" + String(s.id) +
         ",\"ago\":" + ago + ",\"ends\":" + String(s.ends) +
         ",\"shots\":" + String(s.shots) + ",\"scored\":" + String(s.scores) +
         ",\"avg\":" + String(s.avgX100 / 100.0f, 2) + ",\"x\":" + String(s.xCount) +
         ",\"min\":" + String(s.minutes) + ",\"invalidEnds\":" + String(s.invalidEnds) +
         ",\"invalidArrows\":" + String(s.invalidArrows) +
         ",\"mismatch\":" + jbool(s.flags & SLOT_MISMATCH) +
         ",\"manual\":" + jbool(s.flags & SLOT_MANUAL) + "}";
}

// Close the session and write it to the next slot
void finishSession(bool manual) {
  const uint32_t now = millis();

  // A pending confirmation expires, the end then counts as not entered
  pendingScore = false;

  // Last end without entry: 1 shot = setting the bow down, ignore it
  if (endShots > 1) {
    closeEndInvalid(endShots, "last end without scores");
  } else if (endShots == 1) {
    out("Single last shot without score ignored (bow set down).");
    shotCount--;
    endShots = 0;
  }

  sessionActive = false;
  tiltReset();   // tilt indicator ends with the session

  if (endCount == 0) {
    say("Empty session discarded (nothing stored).",
        "{\"t\":\"sessionEnd\",\"manual\":" + jbool(manual) + ",\"stored\":false}");
    return;
  }

  ShotSlot s;
  memset(&s, 0, sizeof(s));
  s.shots         = shotCount;
  s.scores        = scoreCount;
  s.avgX100       = scoreCount ? (uint16_t)((scoreSum * 100 + scoreCount / 2) / scoreCount) : 0;
  s.minutes       = (uint16_t)((now - sessionStartMs + 30000UL) / 60000UL);
  s.ends          = endCount;
  s.invalidEnds   = invalidEnds;
  s.invalidArrows = invalidArrows;
  s.xCount        = xCount;
  s.flags         = SLOT_USED | (manual ? SLOT_MANUAL : 0);
  if (invalidEnds > 0 || mismatchAccepted) s.flags |= SLOT_MISMATCH;

  ensureLogEpoch();
  s.id = ++shotLog.seq;
  const uint8_t idx = shotLog.next;
  shotLog.slots[idx]  = s;
  slotSavedMs[idx]    = now;
  slotSavedKnown[idx] = true;
  shotLog.next = (shotLog.next + 1) % SLOT_COUNT;
  const bool ok = logSave();

  say(String(manual ? "Session ended" : "Session ended automatically (1 h without a shot)") +
      ": " + String(s.ends) + " ends (" + String(s.invalidEnds) + " invalid), " +
      String(s.scores) + " arrows scored, avg " + String(s.avgX100 / 100.0f, 2) +
      ", " + String(s.xCount) + " X, " + String(s.minutes) + " min" +
      (ok ? "" : "  ERROR while saving!"),
      "{\"t\":\"sessionEnd\",\"manual\":" + jbool(manual) + ",\"stored\":" + jbool(ok) +
      ",\"epoch\":" + String(shotLog.epoch) + ",\"slot\":" + slotJson(s, 1, idx) + "}");
}

String sessionLine() {
  if (!sessionActive) return "No session active.";
  const uint32_t mins = (millis() - sessionStartMs) / 60000UL;
  return "Session running: end " + String(endCount + 1) + " open (" + String(endShots) +
         " shots), " + String(endCount) + " ends recorded (" + String(invalidEnds) +
         " invalid), " + String(scoreCount) + " arrows scored, avg " +
         fmtAvg(scoreSum, scoreCount) + ", " + String(xCount) + " X, " + String(mins) + " min";
}

void printLog() {
  uint8_t used = 0;
  for (uint8_t i = 0; i < SLOT_COUNT; i++) if (shotLog.slots[i].flags & SLOT_USED) used++;

  if (appMode) sendLine("{\"t\":\"logStart\",\"count\":" + String(used) +
                        ",\"epoch\":" + String(shotLog.epoch) + "}");
  else         out("--- Sessions (1 = newest) ---");

  uint8_t shown = 0;
  for (uint8_t i = 0; i < SLOT_COUNT; i++) {
    const uint8_t   idx = (shotLog.next + SLOT_COUNT - 1 - i) % SLOT_COUNT;
    const ShotSlot& s   = shotLog.slots[idx];
    if (!(s.flags & SLOT_USED)) continue;
    shown++;
    if (appMode) { sendLine(slotJson(s, shown, idx)); continue; }
    String line = String(shown) + ": " + String(s.ends) + " ends, " + String(s.shots) +
                  " shots, " + String(s.scores) + " scored, avg " +
                  String(s.avgX100 / 100.0f, 2) + ", " + String(s.xCount) + " X, " +
                  String(s.minutes) + " min";
    if (s.invalidEnds > 0) {
      line += " | invalid: " + String(s.invalidEnds) + " ends, " +
              String(s.invalidArrows) + " arrows";
    }
    if (s.flags & SLOT_MISMATCH) line += "  [!]";
    out(line);
  }

  if (appMode)         sendLine("{\"t\":\"logEnd\"}");
  else if (shown == 0) out("No sessions stored yet.");
}

void handleShots(const String& arg) {
  if (arg == "") {
    if (appMode) { sendLine(sessionJson()); return; }
    out(String("Shot counter: ") + (shotLog.shotsOn ? "on" : "off"));
    out(sessionLine());
  }
  else if (arg == "on") {
    shotLog.shotsOn = 1;
    if (!logSave()) { err("ERROR while saving!"); return; }
    say("Shot counter on.", sessionJson());
  }
  else if (arg == "off") {
    if (sessionActive) { err("End the running session first ('shots stop')."); return; }
    shotLog.shotsOn = 0;
    if (!logSave()) { err("ERROR while saving!"); return; }
    say("Shot counter off.", sessionJson());
  }
  else if (arg == "start") {
    if (!shotLog.shotsOn) { err("Shot counter is off ('shots on')."); return; }
    if (sessionActive)    { err("Session already started"); return; }
    startSession(millis());
    say("Session started.", sessionJson());
  }
  else if (arg == "stop") {
    if (!sessionActive) { err("No session started"); return; }
    finishSession(true);
  }
  else err("Unknown. Possible: shots, shots on|off|start|stop");
}

void handleScore(String args) {
  if (!sessionActive) { err("No session started"); return; }
  args.trim();

  // Close an end without scores (arrow count from the sensor)
  if (args == "skip") {
    if (endShots == 0) { err("Open end has no shots, nothing to skip."); return; }
    closeEndInvalid(endShots, "skipped");
    return;
  }

  uint8_t vals[MAX_SCORES_LINE];
  uint8_t n  = 0;
  uint8_t xn = 0;
  int i = 0;
  const int len = args.length();

  while (i < len) {
    while (i < len && args[i] == ' ') i++;
    if (i >= len) break;
    int j = i;
    while (j < len && args[j] != ' ') j++;
    String tok = args.substring(i, j);
    i = j;

    int  v    = -1;
    bool isX  = false;
    if (tok == "x") {
      v = 10;
      isX = true;
    } else {
      bool valid = tok.length() >= 1 && tok.length() <= 2;
      for (uint8_t k = 0; valid && k < tok.length(); k++) {
        if (!isDigit(tok[k])) valid = false;
      }
      if (valid) v = tok.toInt();
    }
    if (v < 0 || v > 10) {
      err("Invalid value '" + tok + "' (allowed 0-10 or x). Nothing stored, please re-enter the whole line.");
      return;
    }
    if (n >= MAX_SCORES_LINE) {
      err("Too many values in one line (max. " + String(MAX_SCORES_LINE) + "). Nothing stored.");
      return;
    }
    vals[n++] = (uint8_t)v;
    if (isX) xn++;
  }

  if (n == 0) { err("Format: score 9 7 x 0  or  score skip"); return; }

  // Compare with the shots of this end
  if (n != endShots) {
    pendingScore = true;
    pendingN = n;
    pendingX = xn;
    memcpy(pendingVals, vals, n);
    say("End " + String(endCount + 1) + ": " + String(endShots) + " shots counted, " +
        String(n) + " values entered - save anyway? (yes/no)",
        "{\"t\":\"confirm\",\"end\":" + String(endCount + 1) + ",\"counted\":" + String(endShots) +
        ",\"entered\":" + String(n) + "}");
    return;
  }
  closeEndValid(vals, n, xn);
}

// ============================================================================
// Status and commands
// ============================================================================
String statusLine() {
  String s = "Light=" + String(lastLdr) + (isDark ? " (dark)" : " (bright)");
  s += " | Battery=" + String(lastVbat, 2) + "V " + String((int)(lastPct + 0.5f)) + "%";
  s += " (" + String(chargeText(lastCharge)) + ")";
  s += " | LED=" + String(ledState == LS_ON ? "on" : (ledState == LS_BLINK ? "blinking" : "off"));
  s += " Bright=" + String((int)(brightNow < 0 ? brightTarget() : brightNow)) + "%";
  if (levelActive() && tiltFilterOk) s += " | Tilt=" + String(tiltDeg, 1) + "deg";
  if (ledMode == MODE_ON)  s += " | Mode=on";
  if (ledMode == MODE_OFF) s += " | Mode=off";
  if (lowBatLock)          s += " | BATTERY EMPTY";
  if (sessionActive)       s += " | End " + String(endCount + 1) + ": " + String(endShots) + " shots";
  return s;
}

String statusJson() {
  const char* led  = (ledState == LS_ON) ? "on" : (ledState == LS_BLINK ? "blinking" : "off");
  const char* mode = (ledMode == MODE_ON) ? "on" : (ledMode == MODE_OFF ? "off" : "auto");
  String j = "{\"t\":\"status\",\"light\":" + String(lastLdr) + ",\"dark\":" + jbool(isDark) +
             ",\"vbat\":" + String(lastVbat, 2) + ",\"pct\":" + String((int)(lastPct + 0.5f)) +
             ",\"chg\":\"" + chargeText(lastCharge) + "\",\"led\":\"" + led + "\"" +
             ",\"duty\":" + String(dutyFull) + ",\"bright\":" + String((int)(brightNow < 0 ? brightTarget() : brightNow)) +
             ",\"mode\":\"" + mode + "\"" +
             ",\"lowbat\":" + jbool(lowBatLock) +
             ",\"tilt\":" + ((levelActive() && tiltFilterOk) ? String(tiltDeg, 1) : String("null")) +
             ",\"session\":" + jbool(sessionActive);
  if (sessionActive) j += ",\"end\":" + String(endCount + 1) + ",\"endShots\":" + String(endShots);
  return j + "}";
}

void sendStatus() {
  say(statusLine(), statusJson());
}

// Settings go out as one short line per item: long lines were unreliable over BLE
void sendCfg() {
  sendLine("{\"t\":\"cfgStart\",\"n\":" + String(SETTING_COUNT) + "}");
  for (uint8_t i = 0; i < SETTING_COUNT; i++) {
    const SettingDef& d = SETTINGS[i];
    sendLine("{\"t\":\"cfgItem\",\"k\":\"" + String(d.key) + "\",\"v\":" + fmtSetting(d, getSetting(cfg, d)) +
         ",\"def\":" + fmtSetting(d, getSetting(DEFAULTS, d)) +
         ",\"min\":" + fmtSetting(d, d.mn) + ",\"max\":" + fmtSetting(d, d.mx) +
         ",\"dec\":" + String(d.dec) + ",\"zero\":" + jbool(d.zeroOk) +
         ",\"unit\":" + jstr(d.unit) + ",\"d\":" + jstr(d.desc) + "}");
  }
  sendLine("{\"t\":\"cfgEnd\"}");
}

void printConfig() {
  if (appMode) { sendCfg(); return; }
  out("--- Settings ---");
  for (uint8_t i = 0; i < SETTING_COUNT; i++) {
    const SettingDef& d = SETTINGS[i];
    String k = d.key;
    while (k.length() < 10) k += ' ';
    String line = k + "= " + fmtSetting(d, getSetting(cfg, d));
    if (d.unit[0]) line += " " + String(d.unit);
    line += "   (" + String(d.desc) + ", " + fmtSetting(d, d.mn) + "-" + fmtSetting(d, d.mx) + ")";
    out(line);
  }
}

void printHelp() {
  out("--- Commands ---");
  out("status            current readings");
  out("get               all settings");
  out("set <n> <v>       change a setting");
  out("save              save settings");
  out("defaults          load default values");
  out("live on|off       readings every 2 s");
  out("mode auto|on|off  LED mode (test)");
  out("shots             counter status + running session");
  out("shots on|off      shot counter on/off");
  out("shots start|stop  start/end a session manually");
  out("score 9 x 7 0     score an end (0-10, x = inner ten)");
  out("score skip        end without scores (invalid)");
  out("log               stored sessions");
  out("level             tilt indicator status");
  out("level on|auto|off tilt indicator always / in sessions / off");
  out("level style normal|inverted  blink faster when tilted / when level");
  out("level cal         calibration step 1");
  out("level cal2        calibration step 2");
  out("app on|off        JSON output for the app");
  out("dfu               reboot into update mode (USB needed)");
}

void handleSet(const String& args) {
  int sp = args.indexOf(' ');
  if (sp < 0) { err("Format: set <name> <value>"); return; }
  const String key = args.substring(0, sp);
  String val = args.substring(sp + 1);
  val.trim();

  const SettingDef* d = findSetting(key);
  if (!d) { err("Unknown setting. 'get' shows all names."); return; }

  const float v = val.toFloat();
  const bool inRange = (v >= d->mn && v <= d->mx) || (d->zeroOk && v == 0);
  if (!inRange) {
    err("Value out of the allowed range (" + fmtSetting(*d, d->mn) + "-" + fmtSetting(*d, d->mx) + ").");
    return;
  }

  setSetting(cfg, *d, v);
  imuSetThs(cfg.wakeThs);        // apply IMU thresholds right away
  imuSetTapThs(cfg.tapThs);
  if (key == "tx_power") applyTxPower();

  const String shown = fmtSetting(*d, getSetting(cfg, *d));
  say(key + " = " + shown + "  (active, permanent with 'save')",
      "{\"t\":\"ack\",\"cmd\":\"set\",\"k\":\"" + key + "\",\"v\":" + shown + "}");
  if (cfg.darkOn >= cfg.darkOff) err("Warning: dark_on should be lower than dark_off.");
}

// Reboot into the UF2 bootloader, same as a double click on the reset button.
// Only with USB connected: without a cable the board would sit in the
// bootloader and drain the battery until it is reset by hand.
void enterDfu() {
  if (!usbPresent()) { err("Connect the USB cable first, then send 'dfu' again."); return; }
  say("Rebooting into update mode (UF2 drive). Copy the new firmware onto the drive.",
      "{\"t\":\"ack\",\"cmd\":\"dfu\"}");
  delay(300);                                  // let the message go out
  uvOff();
  sd_power_gpregret_clr(0, 0xFF);
  sd_power_gpregret_set(0, 0x57);              // 0x57 = UF2 mode for the bootloader
  sd_nvic_SystemReset();
}

void appOn() {
  appMode = true;
  sendLine("{\"t\":\"hello\",\"proto\":" + String(PROTO_VERSION) + ",\"fw\":\"" + FW_VERSION +
           "\",\"name\":\"" + BLE_NAME + "\",\"imu\":" + jbool(imuOk) + "}");
  sendCfg();
  sendLine(statusJson());
  sendLine(sessionJson());
  sendLine(levelJson());
}

void handleCommand(String line) {
  line.trim();
  line.toLowerCase();
  if (line.length() == 0) return;

  // Pending confirmation for a mismatching end
  if (pendingScore) {
    pendingScore = false;
    if (line == "yes") {
      // The entered arrow count wins over the sensor count
      shotCount = shotCount - endShots + pendingN;
      mismatchAccepted = true;
      closeEndValid(pendingVals, pendingN, pendingX);
      return;
    }
    say("Input discarded, end " + String(endCount + 1) +
        " stays open. Re-enter the scores or close it with 'score skip'.",
        "{\"t\":\"discarded\",\"end\":" + String(endCount + 1) + "}");
    if (line == "no") return;
  }

  if (line == "help" || line == "h")            printHelp();
  else if (line == "status" || line == "?")     sendStatus();
  else if (line == "get" || line == "config")   printConfig();
  else if (line == "save") {
    const bool ok = cfgSave();
    if (ok) say("Saved.", "{\"t\":\"ack\",\"cmd\":\"save\"}");
    else    err("ERROR while saving!");
  }
  else if (line == "defaults") {
    cfg = DEFAULTS;
    imuSetThs(cfg.wakeThs);
    imuSetTapThs(cfg.tapThs);
    applyTxPower();
    if (appMode) sendCfg();
    else         out("Default values loaded (permanent with 'save').");
  }
  else if (line == "app on")    appOn();
  else if (line == "dfu")       enterDfu();
  else if (line == "app off")   { appMode = false; out("App mode off, human-readable output."); }
  else if (line == "live on")   { liveMode = true;  say("Live output on (every 2 s).", "{\"t\":\"ack\",\"cmd\":\"live\",\"on\":true}"); }
  else if (line == "live off")  { liveMode = false; say("Live output off.", "{\"t\":\"ack\",\"cmd\":\"live\",\"on\":false}"); }
  else if (line == "mode auto") { ledMode = MODE_AUTO; say("LED mode: automatic", "{\"t\":\"ack\",\"cmd\":\"mode\",\"mode\":\"auto\"}"); }
  else if (line == "mode on")   { ledMode = MODE_ON;   say("LED mode: always on (test)", "{\"t\":\"ack\",\"cmd\":\"mode\",\"mode\":\"on\"}"); }
  else if (line == "mode off")  { ledMode = MODE_OFF;  say("LED mode: always off", "{\"t\":\"ack\",\"cmd\":\"mode\",\"mode\":\"off\"}"); }
  else if (line == "shots")               handleShots("");
  else if (line.startsWith("shots "))     { String a = line.substring(6); a.trim(); handleShots(a); }
  else if (line == "score")               handleScore("");
  else if (line.startsWith("score "))     handleScore(line.substring(6));
  else if (line == "log")                 printLog();
  else if (line == "level")               handleLevel("");
  else if (line.startsWith("level "))     { String a = line.substring(6); a.trim(); handleLevel(a); }
  else if (line.startsWith("set "))       handleSet(line.substring(4));
  else err("Unknown command. 'help' lists all commands.");

  lastSensorMs = millis() - SENSOR_INTERVAL_MS;  // apply changes immediately
}

void feedChar(char c) {
  if (c == '\n' || c == '\r') {
    if (rxBuf.length()) { handleCommand(rxBuf); rxBuf = ""; }
  } else if (rxBuf.length() < 160) {
    rxBuf += c;
  }
}

void readCommands() {
  while (bleuart.available()) feedChar((char)bleuart.read());
  while (Serial && Serial.available()) feedChar((char)Serial.read());
}

// ============================================================================
// Bluetooth
// ============================================================================
void onConnect(uint16_t connHandle) {
  bleConnHandle = connHandle;
  BLEConnection* conn = Bluefruit.Connection(connHandle);
  if (conn) {
    conn->requestMtuExchange(BLE_MTU);   // whole lines per packet
    conn->requestDataLengthUpdate();
  }
}

void onDisconnect(uint16_t connHandle, uint8_t reason) {
  (void)reason;
  if (connHandle == bleConnHandle) bleConnHandle = BLE_CONN_HANDLE_INVALID;
}

void bleInit() {
  Bluefruit.autoConnLed(false);                  // blue board LED off (saves power)
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);  // large packets + bigger send queue
  Bluefruit.begin();
  cfg.txPower = snapTxPower(cfg.txPower);
  Bluefruit.setTxPower(cfg.txPower);
  Bluefruit.setName(BLE_NAME);
  Bluefruit.Periph.setConnInterval(CONN_INT_MIN, CONN_INT_MAX);
  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);
  bleuart.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(false);  // handled by the sketch
  Bluefruit.Advertising.setInterval(ADV_FAST_INTERVAL, ADV_SLOW_INTERVAL);
  Bluefruit.Advertising.setFastTimeout(ADV_FAST_TIMEOUT);
}

// Radio only while the board is active
void bleUpdate(bool active) {
  const bool adv       = Bluefruit.Advertising.isRunning();
  const bool connected = Bluefruit.connected();

  if (active) {
    if (!connected && !adv) Bluefruit.Advertising.start(0);
    return;
  }

  // Idle: stop advertising and drop an existing connection
  if (adv) Bluefruit.Advertising.stop();
  if (connected) {
    say("No movement - disconnecting Bluetooth.", "{\"t\":\"event\",\"e\":\"idle\"}");
    delay(100);  // let the message go out
    for (uint16_t h = 0; h < BLE_MAX_CONNECTION; h++) {
      if (Bluefruit.connected(h)) Bluefruit.disconnect(h);
    }
  }
}

// ============================================================================
// Evaluate light, battery and LED
// ============================================================================
void evaluate() {
  const ChargeState c = readCharge();
  if (c != lastCharge) {
    const String json = "{\"t\":\"event\",\"e\":\"charge\",\"state\":\"" + String(chargeText(c)) + "\"}";
    if (c == CHG_CHARGING)      say("Charging started.", json);
    else if (c == CHG_FULL)     say("Battery full, charging finished.", json);
    else                        say("USB disconnected.", json);
    lastCharge = c;
  }

  lastVbat = readVbat();
  lastPct  = vbatToPercent(lastVbat);
  lastLdr  = readLdr();

  // Battery protection
  if (lastVbat < cfg.cutoff) {
    if (!lowBatLock) say("Battery empty - LED locked until charged.", "{\"t\":\"event\",\"e\":\"lowbat\"}");
    lowBatLock = true;
  }
  if (lastVbat > cfg.cutoff + VBAT_RECOVER_OFFSET) lowBatLock = false;

  // Darkness with hysteresis and debouncing
  const bool wasDark = isDark;
  if (wasInactive) {
    brightNow = -1;                  // no gliding right after picking up the bow
    isDark = lastLdr < lightOnThreshold();   // decide immediately after picking up the bow
    darkCount = brightCount = 0;
  } else if (!isDark) {
    if (lastLdr < lightOnThreshold()) {
      if (++darkCount >= cfg.darkConfirm) { isDark = true; darkCount = 0; }
    } else darkCount = 0;
  } else {
    if (lastLdr > cfg.darkOff) {
      if (++brightCount >= cfg.darkConfirm) { isDark = false; brightCount = 0; }
    } else brightCount = 0;
  }
  if (isDark != wasDark && !wasInactive) {
    say(isDark ? "Dark detected" : "Bright detected",
        "{\"t\":\"event\",\"e\":\"light\",\"dark\":" + jbool(isDark) + "}");
  }
  wasInactive = false;

  // Adjust brightness to the battery voltage
  // Steady brightness: jump after wake-up or in fixed mode, glide in auto mode
  const float target = brightTarget();
  if (brightNow < 0 || cfg.brightMode == 0) brightNow = target;
  else brightNow += BRIGHT_SMOOTH * (target - brightNow);
  dutyFull = dutyForVbat(lastVbat, percentToMa(brightNow));
  dutyTilt = dutyForVbat(lastVbat, percentToMa(tiltBrightness()));
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_UV, OUTPUT);
  digitalWrite(PIN_UV, LOW);
  pinMode(PIN_LDR_PWR, OUTPUT);
  digitalWrite(PIN_LDR_PWR, LOW);

  // Enable battery measurement permanently. According to Seeed, P0.14 must
  // NOT be set HIGH while charging, otherwise the measuring pin may be damaged.
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);

  // Charge current fixed at ~50 mA (about 0.6 C at 85 mAh):
  // per Seeed, P0.13 as high-impedance input without pull = 50 mA,
  // as output LOW = 100 mA (too much for this battery).
  NRF_P0->PIN_CNF[13] =
      (GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)   |
      (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
      (GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos);

  analogReadResolution(12);
  analogWriteResolution(PWM_BITS);
  // The internal battery divider (1M/510k) has a very high impedance. With the
  // default 3 us acquisition time the ADC reads too low; Nordic recommends
  // 40 us for source impedances up to 800 kOhm.
  analogSampleTime(40);

  InternalFS.begin();
  cfgLoad();
  logLoad();
  levelLoad();

  imuOk = imuInit();
  bleInit();
  chargeInit();
  lastCharge = readCharge();

  lastMotionMs = millis();                        // active right after start
  lastSensorMs = millis() - SENSOR_INTERVAL_MS;
}

// ============================================================================
// Loop
// ============================================================================
void loop() {
  uint32_t   now       = millis();
  const bool connected = Bluefruit.connected();

  if (!connected) { liveMode = false; welcomed = false; appMode = false; }

  // Only movement keeps the board active (a connection does not)
  if (imuMotionSinceLastCheck()) lastMotionMs = now;

  // Shot: from the INT1 interrupt (exact time) or, as a fallback, from polling
  bool     shot   = false;
  uint32_t shotAt = now;
  if (tapFlag) {
    noInterrupts();
    shotAt  = tapMs;
    tapFlag = false;
    interrupts();
    shot = true;
  }
  if (imuTapSinceLastCheck()) shot = true;   // also re-arms INT1
  if (shot) {
    lastMotionMs = now;
    registerShot(shotAt);
  }

  const bool active = (now - lastMotionMs) < cfg.timeoutS * 1000UL;

  // Shot detection only if the counter is on and the bow is in use
  imuSetFast(active && shotLog.shotsOn);

  // End the session automatically after 1 h without a shot
  if (sessionActive && (now - sessionLastShotMs) >= SESSION_IDLE_MS) {
    finishSession(false);
  }

  bleUpdate(active);
  readCommands();
  now = millis();

  // Greeting as soon as the app is ready
  if (connected && bleuart.notifyEnabled() && !welcomed) {
    welcomed = true;
    out("UV-Sight connected. 'help' lists all commands.");
    if (!imuOk) out("Note: IMU not found, motion detection off.");
    sendStatus();
    if (sessionActive) out(sessionLine());
    if (lvl.on && !lvl.calibrated) out("Note: tilt indicator not calibrated ('level cal').");
    lastReportMs = now;
  }

  // ---- Idle: bow lies still ----
  if (!active) {
    uvOff();
    ledState = LS_OFF;
    tiltReset();
    wasInactive = true;
    darkCount = brightCount = 0;
    delay(IDLE_POLL_MS);
    return;
  }

  // ---- Active ----
  if (wasInactive || (now - lastSensorMs) >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;
    evaluate();
    if (liveMode) { sendStatus(); lastReportMs = now; }
  }

  const bool lvlOn = levelActive();
  if (lvlOn) updateTilt();
  updateLed(millis());

  // Periodic status report, only when connected
  if (connected && !liveMode && cfg.reportS > 0 && (now - lastReportMs) >= cfg.reportS * 1000UL) {
    sendStatus();
    lastReportMs = now;
  }

  delay(lvlOn ? LEVEL_LOOP_MS : ACTIVE_POLL_MS);
}
