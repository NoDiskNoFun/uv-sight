/*
  ============================================================================
  UV sight illumination + shot counter for compound bows
  Board: Seeed XIAO nRF52840 Sense
  ============================================================================

  Illumination
  ------------
  - Motion detection via the built-in IMU (accelerometer only).
  - If the bow lies still (timeout), everything is off: LED, light sensor, radio.
    The accelerometer drops to 12.5 Hz and the board sleeps until INT1 wakes it.
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
    interrupt, so no shot is missed between two loop passes. While the bow
    rests the same line carries the motion wake-up, so the board sleeps
    until the bow is picked up instead of polling the IMU.
    After a shot there is a lockout ("lockout", default 1.5 s): further
    impacts are ignored (vibration) and the tilt indicator pauses.
  - Shot strength: in shot mode the IMU fills its FIFO at 416 Hz (+-16 g).
    After a shot the peak of the total acceleration is read from it and
    reported in g. It is a comparison value for setting tap_ths, not an
    exact physical peak (416 Hz sampling misses the sharpest part).
  - A training session starts with the first shot or with "shots start".
  - A session ends after "session_end" minutes (default 60) without a
    detected shot, or with "shots stop".
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
  - Sessions are stored on the external 2 MB QSPI flash as 64-byte records
    with CRC (about 32,000 sessions; the oldest are overwritten when full):
    date (if the app set the clock), ends, shots, scored arrows, average,
    X, invalid ends/arrows, duration. A log of firmware 1.2 - 2.1 in the
    internal flash is taken over automatically on the first start.

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
  - "log put" writes a session from a backup into the log (skipped if its
    epoch + id are already stored). The app uses it to restore the sight.
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
#include <nrfx_qspi.h>  // external 2 MB flash for the session log
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#if __has_include(<nrf52_erratas.h>)
  #include <nrf52_erratas.h>   // Nordic MDK: which errata apply to the chip revision at hand
  #define HAVE_NRF52_ERRATAS 1
#endif

using namespace Adafruit_LittleFS_Namespace;

#if !defined(PIN_VBAT) || !defined(VBAT_ENABLE)
  #error "Wrong board: select 'Seeed XIAO nRF52840 Sense' from 'Seeed nRF52 Boards' (not mbed)."
#endif

#if !defined(PIN_QSPI_SCK) || !defined(PIN_QSPI_CS) || !defined(PIN_QSPI_IO0) || \
    !defined(PIN_QSPI_IO1) || !defined(PIN_QSPI_IO2) || !defined(PIN_QSPI_IO3)
  #error "Board package without QSPI pin definitions: the session log needs the external flash."
#endif

#ifndef PIN_LSM6DS3TR_C_INT1
  #warning "PIN_LSM6DS3TR_C_INT1 not defined: shot detection falls back to polling only."
#endif

// ============================================================================
// FIXED VALUES
// ============================================================================

// Pins (Arduino pin n = XIAO Dn)
const uint8_t PIN_UV      = 6;    // D6 -> 2.2k -> base of BC547
// UV LED stage present? 0 = no (hunting bow without a sight: no LED, no light sensor), 1 = yes,
// 2 = find out at boot: with the BC547 and its base resistor fitted, a pin charged HIGH and then
// released falls to the base-emitter voltage within microseconds; without them it stays HIGH.
// Without the LED stage all light settings default to off and the app hides them.
#ifndef UV_HAS_LED
#define UV_HAS_LED 2
#endif
bool ledPresent = true;
const uint8_t PIN_LDR_PWR = 2;    // D2 powers the LDR only during a measurement
const uint8_t PIN_LDR     = A0;   // midpoint LDR / 10k

// Timing
const uint32_t IDLE_POLL_MS       = 500;   // loop interval while idle (fallback without INT1)
const uint32_t IDLE_WAKE_MAX_MS   = 5000;  // idle: longest sleep between housekeeping passes
const uint32_t ACTIVE_POLL_MS     = 250;   // loop interval while active
const uint32_t SENSOR_INTERVAL_MS = 2000;  // check light + battery every 2 s
const uint32_t IMU_SETTLE_MS      = 1000;  // ignore motion after a mode change
const uint32_t LEVEL_LOOP_MS      = 30;    // loop interval while the tilt indicator runs
const uint32_t AWAKE_MAX_S        = 900;   // longest hold the app can ask for ("awake <s>")
const uint32_t AWAKE_PUT_MS       = 60000; // every "log put" keeps the sight awake this long

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
const uint8_t  PWM_BITS        = 12;    // fine steps at low brightness
const uint16_t PWM_MAX         = (1u << PWM_BITS) - 1;

// Tilt indicator (adjustable here in the sketch)
const float    TILT_HYST_FRAC    = 0.3;   // hysteresis at the tolerance edge, share of level_tol
const float    CAL_MIN_ANGLE_DEG = 15.0;  // minimum angle between calibration steps
const uint32_t CAL_COUNTDOWN_MS  = 3000;  // wait time before a calibration measurement

// Shot counter
const uint8_t  MAX_SCORES_LINE = 40;

// Firmware / protocol
const char*    FW_VERSION        = "6.3";
const uint8_t  PROTO_VERSION     = 21;

// Bluetooth
const char*    BLE_NAME          = "UV-Sight";
const char*    NAME_FILE         = "/sightname.txt";   // the name the owner gave this sight ("name <text>")
char           sightName[20]     = "";                 // shown in the app and appended to the BLE name
const uint16_t ADV_FAST_INTERVAL = 160;   // 100 ms (unit 0.625 ms), first 10 s
const uint16_t ADV_SLOW_INTERVAL = 1636;  // ~1 s afterwards
const uint16_t ADV_FAST_TIMEOUT  = 10;    // s
const uint16_t CONN_INT_MIN      = 160;   // 200 ms (unit 1.25 ms)
const uint16_t CONN_INT_MAX      = 320;   // 400 ms
const uint16_t BLE_MTU           = 247;   // max. packet size (default would be 23)
const uint8_t  BLE_SEND_RETRIES  = 50;    // retries per packet when the queue is full
const uint16_t BLE_RETRY_MS      = 20;    // wait between retries

// Power
// nRF52840 erratum 89: a TWIM (I2C) that stays enabled while GPIOTE is used
// draws a static 400 uA. Per the Nordic errata this only affects the revision
// Engineering A; every production revision is free of it. The workaround
// (switch the bus off between loop passes and toggle its power domain) is
// therefore only applied when the chip says it needs it.
//   0 = never, 1 = automatic (checked at start with the MDK, off if the MDK
//   header is missing), 2 = always
#define TWIM_ANOMALY_89   1

// ============================================================================
// SETTINGS (changeable via Bluetooth, stored in flash with "save")
// ============================================================================
struct Config {
  uint32_t magic;
  uint16_t darkOn;       // LDR value: below = dark
  uint16_t darkOff;      // LDR value: above = bright again
  uint8_t  darkConfirm;  // consecutive readings needed to switch
  uint8_t  wakeThs;      // motion threshold, 1 LSB = 125 mg (1..63)
  uint8_t  tapThs;       // shot threshold,   1 LSB = 250 mg (1..62)
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
  uint16_t sessionEndMin; // minutes without a shot until the session ends
  uint16_t tiltSmoothMs;  // smoothing of the cant reading against hand tremor
  float    tiltFullDeg;   // cant at which the blink rate reaches its end value
  float    blinkMinHz;    // blink rate just outside the tolerance (normal style)
  float    blinkMaxHz;    // blink rate from tiltFullDeg on (normal style)
  float    fadeS;         // auto brightness: time to follow a change, s (0 = instant)
  uint16_t batMah;        // battery capacity, for the runtime estimate
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
  32.0f,   // tiltOffset (tilt blink at about 1 mA, the former default)
  60,      // sessionEndMin
  400,     // tiltSmoothMs
  10.0f,   // tiltFullDeg
  1.0f,    // blinkMinHz
  8.0f,    // blinkMaxHz
  5.0f,    // fadeS
  85       // batMah
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
  {"fade",      S_F,   offsetof(Config, fadeS),       0,    30,   0, false, "s",      "auto brightness: time to follow a change, 0 = instant"},
  {"tilt_full", S_F,   offsetof(Config, tiltFullDeg), 1,    30,   1, false, "deg",    "cant at which the blink rate reaches its end value"},
  {"blink_min", S_F,   offsetof(Config, blinkMinHz),  0.5,  12,   1, false, "Hz",     "blink rate just outside the tolerance (normal style)"},
  {"blink_max", S_F,   offsetof(Config, blinkMaxHz),  0.5,  12,   1, false, "Hz",     "blink rate at tilt_full and beyond (normal style)"},
  {"tilt_smooth", S_U16, offsetof(Config, tiltSmoothMs), 50, 2000, 0, false, "ms",   "smoothing of the cant reading against hand tremor"},
  {"session_end", S_U16, offsetof(Config, sessionEndMin), 5, 480, 0, false, "min",   "minutes without a shot until the session ends"},
  {"vf",        S_F,   offsetof(Config, vf),          2.5,  3.6,  2, false, "V",      "LED forward voltage"},
  {"cutoff",    S_F,   offsetof(Config, cutoff),      3.0,  3.7,  2, false, "V",      "battery cutoff"},
  {"level_tol", S_F,   offsetof(Config, levelTol),    0.2,  10,   1, false, "deg",    "tilt tolerance"},
  {"tap_ths",   S_U8,  offsetof(Config, tapThs),      1,    62,   0, false, "x250mg", "shot threshold (applied in 0.5 g steps)"},
  {"ths",       S_U8,  offsetof(Config, wakeThs),     1,    63,   0, false, "x125mg", "motion threshold"},
  {"lockout",   S_U32, offsetof(Config, lockoutMs),   200,  5000, 0, false, "ms",     "after a shot: no counting, no tilt"},
  {"timeout",   S_U32, offsetof(Config, timeoutS),    10,   3600, 0, false, "s",      "without movement until idle"},
  {"report",    S_U32, offsetof(Config, reportS),     2,    600,  0, true,  "s",      "between status reports, 0 = off"},
  {"tx_power",  S_I8,  offsetof(Config, txPower),     -20,  8,    0, false, "dBm",    "radio power, higher = more range and current"},
  {"bat_mah",   S_U16, offsetof(Config, batMah),      10,   5000, 0, false, "mAh",    "battery capacity, for the runtime estimate"},
};
const uint8_t SETTING_COUNT = sizeof(SETTINGS) / sizeof(SETTINGS[0]);

// ============================================================================
// SESSION LOG on the external 2 MB QSPI flash
// ============================================================================
// Records of 64 bytes are appended in a ring over the whole chip. Each record
// has a magic, a running write number (seq) and a CRC, so a record torn by a
// power loss is detected and skipped. Before a 4 KB sector is reused, it is
// erased; the oldest sessions are overwritten only when the chip is full
// (about 32,000 sessions).
#define SLOT_MISMATCH 0x02   // at least one end invalid or accepted with "yes" despite mismatch
#define SLOT_MANUAL   0x04

struct __attribute__((packed, aligned(4))) LogRec {
  uint32_t magic;          // REC_MAGIC
  uint32_t seq;            // write order on this flash (1, 2, 3 ...)
  uint32_t epoch;          // log key part 1 (random per log, kept on import)
  uint32_t id;             // log key part 2 (session number)
  uint32_t start;          // session start, unix seconds, 0 = unknown
  uint16_t shots;          // total shots (sensor, corrected by "yes")
  uint16_t scores;         // validly scored arrows
  uint16_t avgX100;        // average of valid arrows x 100
  uint16_t minutes;        // duration
  uint16_t ends;           // total ends (valid + invalid)
  uint16_t invalidEnds;    // invalid ends
  uint16_t invalidArrows;  // arrows with 0 points (invalid)
  uint16_t xCount;         // inner tens (X), included in scores
  uint8_t  flags;          // SLOT_MISMATCH / SLOT_MANUAL
  uint8_t  reserved[23];   // room for later fields, kept 0xFF
  uint32_t crc;            // CRC32 over the first 60 bytes
};
static_assert(sizeof(LogRec) == 64, "LogRec must be 64 bytes");
typedef bool (*LogVisitor)(const LogRec&);   // callback for logForEach()

// One end of a session, written when the end is closed. Same size and ring as
// the session records, told apart by the magic; the session key (epoch + id)
// links it to its session.
const uint32_t END_MAGIC = 0x31444E45;       // "END1"
#define END_VALID       0x01
#define END_SKIPPED     0x02
#define END_DIST_MANUAL 0x04   // distance set by hand (only these teach the range model)
#define END_DIST_AUTO   0x08   // distance recognised from the aiming angle
const int16_t  ANGLE_NONE = -32768;          // no angle measured
struct __attribute__((packed, aligned(4))) EndRec {
  uint32_t magic;          // END_MAGIC
  uint32_t seq;
  uint32_t epoch;          // session key
  uint32_t id;
  uint16_t n;              // end number within the session
  uint8_t  arrows;
  uint8_t  flags;          // END_VALID / END_SKIPPED
  uint16_t sum;
  uint8_t  x;
  uint8_t  distM;          // distance in m, 0 = not given
  int16_t  angleC;         // mean aiming angle up/down, 1/100 degree (ANGLE_NONE = none)
  uint16_t angleSdC;       // spread of the arrows' angles, 1/100 degree
  uint8_t  angleN;         // arrows with an angle
  uint8_t  pad;
  uint8_t  scores[20];     // 4 bits per arrow: 0-10, 11 = X, 15 = none
  int16_t  cantC;          // mean cant at release, 1/100 degree (signed: left/right)
  uint16_t cantMaxC;       // largest cant of an arrow, 1/100 degree
  uint8_t  cantN;          // arrows with a cant reading (0xFF in records before firmware 4.2)
  uint8_t  cantedN;        // arrows released outside the cant tolerance
  uint8_t  calGen;         // calibration number the angles were measured with (0xFF before 5.0)
  uint8_t  setup;          // arrow/bow setup (0 = default; 0xFF before 5.0)
  uint8_t  reserved[2];
  uint32_t crc;
};
static_assert(sizeof(EndRec) == 64, "EndRec must be 64 bytes");

const uint32_t REC_MAGIC       = 0x31565531;              // "1UV1"
const uint32_t QF_SIZE         = 2UL * 1024UL * 1024UL;   // P25Q16H: 2 MB
const uint32_t QF_SECTOR       = 4096;
const uint16_t QF_SECTORS      = QF_SIZE / QF_SECTOR - 1; // 511 for the log ring
const uint32_t QF_TEST_ADDR    = (QF_SIZE / QF_SECTOR - 1) * QF_SECTOR;  // last sector: "log test" only
const uint16_t RECS_PER_SECTOR = QF_SECTOR / sizeof(LogRec); // 64
const uint16_t LOG_PRINT_LAST  = 20;                      // "log" in the terminal

bool     qfOk      = false;          // external flash answered at boot
uint32_t qfJedec   = 0;              // manufacturer / type / capacity id
uint32_t sectorFirstSeq[QF_SECTORS]; // seq of the first record per sector, 0 = empty
uint32_t logHeadAddr = 0;            // next write address
uint32_t logLastSeq  = 0;            // seq of the newest record
uint32_t logCount    = 0;            // valid records on the chip
String   logError;                   // reason of the last failed flash access
uint8_t  qfStatus1 = 0, qfStatus2 = 0; // status registers read at boot
bool     qfUnprotected = false;      // write protection was found and cleared
uint32_t qspiInitErr   = 0;          // last error code of nrfx_qspi_init (0 = none)
uint16_t qspiRecoveries = 0;         // times the chip had to be reset by hand
uint32_t qfBbJedec     = 0;          // JEDEC id read by hand during the last reset

// RAM only: when recent sessions were saved (age for the app, lost on reboot)
const uint8_t RECENT_SAVES = 16;
uint32_t recentSeq[RECENT_SAVES];
uint32_t recentMs[RECENT_SAVES];
uint8_t  recentNext = 0;

// Small state file in internal flash
struct ShotState {
  uint32_t magic;
  uint8_t  shotsOn;   // counter on/off
  uint8_t  pad[3];
  uint32_t epoch;     // log key part 1 for new sessions
  uint32_t lastId;    // last session number handed out
  uint32_t clearedSeq; // "log clear": records up to this seq count as deleted
};
const uint32_t STATE_MAGIC = 0x53545331;   // "1STS"
const char*    STATE_FILE  = "/shotstate.bin";
ShotState shotState;

// Log format of firmware 1.2 - 2.1 (internal flash, 32 slots), for migration
struct LegacySlot {
  uint32_t id;
  uint16_t shots, scores, avgX100, minutes, ends, invalidEnds, invalidArrows, xCount;
  uint8_t  flags;
  uint8_t  pad;
};
const uint8_t LEGACY_SLOTS = 32;
struct LegacyLog {
  uint32_t magic;
  uint8_t  shotsOn;
  uint8_t  next;
  uint8_t  pad[2];
  uint32_t epoch;
  uint32_t seq;
  LegacySlot slots[LEGACY_SLOTS];
};
const uint32_t LEGACY_MAGIC  = 0x53484F04;
const char*    LEGACY_FILE   = "/shots.bin";
const char*    LEGACY_BACKUP = "/shots.old";
uint8_t migratedCount = 0;   // sessions taken over at this boot

// ----------------------------------------------------------------------------
// Battery history (internal flash): unloaded voltage at state changes plus the
// time spent in each state since the previous point. The sight fits its own
// current draw from it (idle, awake, LED factor) and estimates runtimes.
// ----------------------------------------------------------------------------
enum BatPointType : uint8_t {
  BP_BOOT = 1, BP_IDLE, BP_WAKE, BP_LED_ON, BP_LED_OFF, BP_USB_IN, BP_FULL, BP_USB_OUT
};
#define BPF_USB   0x01     // USB was connected at this point
#define BPF_CLOCK 0x02     // t is a real time (clock set by the app)

struct __attribute__((packed)) BatPoint {
  uint32_t t;        // unix seconds (0 = unknown)
  uint16_t mv;       // battery voltage without load
  uint8_t  type;     // BatPointType
  uint8_t  flags;    // BPF_*
  uint32_t idleS;    // seconds asleep since the previous point
  uint32_t activeS;  // seconds awake since the previous point
  uint32_t ledMAs;   // LED charge since the previous point, mA*s (from the set brightness)
  uint32_t ledS;     // seconds the LED was lit (or blinking) since the previous point
};
static_assert(sizeof(BatPoint) == 24, "BatPoint must be 24 bytes");

const uint16_t BH_POINTS       = 256;                     // ring, about 2 months
const uint32_t BH_MAGIC        = 0x31484142;              // "BAH1"
const char*    BH_FILE         = "/bathist.bin";
const uint32_t BH_MIN_GAP_MS   = 10UL * 60UL * 1000UL;    // at most one point per 10 min
const uint32_t BH_IDLE_MIN_MS  = 6UL * 3600UL * 1000UL;   // wake point only after 6 h asleep
const uint32_t BH_RELAX_MS     = 45000;                   // wait after load before measuring
const uint32_t BH_IDLE_READ_MS = 60000;                   // refresh the unloaded voltage while asleep
const float    BH_WINDOW_PCT   = 4.0f;                    // discharge per fit row (above the noise)

struct BatHist {
  uint32_t magic;
  uint16_t next;
  uint16_t count;
  BatPoint p[BH_POINTS];
};
BatHist bh;

// Accumulators since the last saved point
float    bhIdleMs = 0, bhActiveMs = 0, bhLedMAs = 0, bhLedMs = 0;
uint32_t bhTickMs = 0, bhLastPointMs = 0, bhIdlePointMs = 0;
bool     bhPrevAwake = true, bhPrevUsb = false, bhPrevActive = true;
float    ledMaNow  = 0;                 // model LED current right now (for the accumulators)
uint16_t unloadedMv = 0;                // latest battery voltage measured without load
uint32_t unloadedMs = 0, ledOffMs = 0;
uint8_t  bhPending = 0;                 // point to take once the battery has relaxed
bool     bhPendingForce = false;
uint32_t plugMs = 0;                    // USB plugged in (this boot)
float    plugSoc = -1;                  // state of charge when plugged in (%), -1 = unknown

// Fit results
float    fitIdleMa = 0.06f, fitActiveMa = 0.8f, fitLedK = 1.0f;
float    fitChgMinPerPct = 1.0f, fitChgTailMin = 20.0f;
uint16_t fitRows = 0, fitCharges = 0;
float    fitMah = 0;                    // discharge covered by the fit rows

// Clock, set by the app ("time <unix> [tz minutes]"), valid until reboot
bool     clockValid = false;
uint32_t clockUnix  = 0;     // unix seconds at clockMs
uint32_t clockMs    = 0;
int16_t  clockTzMin = 0;     // phone's offset to UTC in minutes

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
  uint8_t  angleMode; // aiming angle: 0 = auto (during sessions), 1 = off, 2 = on
  float    L[3];   // lateral axis (aiming up/down rotates around this axis)
  float    D[3];   // gravity direction with the bow level and horizontal
  // appended in firmware 5.0 (older files load with the defaults below)
  uint8_t  calGen;      // calibration number, +1 with every new calibration
  uint8_t  cantSignal;  // 1 = the LED blinks when canted (measuring is lvl.on)
  uint8_t  rangeSignal; // 1 = the LED double-blinks when the aiming angle doesn't fit the distance
  uint8_t  ledMode;     // LED switch (MODE_AUTO / MODE_ON / MODE_OFF), kept over restarts
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
#define REG_FIFO_CTRL3  0x08
#define REG_FIFO_CTRL5  0x0A
#define REG_FIFO_STAT1  0x3A
#define REG_FIFO_DATA   0x3E

#define XL_12HZ_8G      0x1C    // idle: 12.5 Hz, +-8 g, low power (wake-up within 80 ms)
#define XL_26HZ_8G      0x2C    // active without shot counter: 26 Hz, +-8 g, low power
#define XL_416HZ_16G    0x64    // shot mode: 416 Hz, +-16 g
#define G_416HZ_1000DPS 0x68    // CTRL2_G: gyro 416 Hz, +-1000 dps (shot matching, shot mode only)
#define G_OFF           0x00
#define FIFO_XL_ONLY    0x01    // FIFO_CTRL3: accelerometer, no decimation
#define FIFO_XL_G       0x09    // FIFO_CTRL3: gyro + accelerometer, no decimation (6 words per sample)
#define FIFO_416_CONT   0x36    // FIFO_CTRL5: 416 Hz, continuous mode
#define FIFO_BYPASS     0x00
#define G_PER_LSB_16G   0.000488f
#define DPS_PER_LSB_1000 0.035f
#define FIFO_KEEP_WORDS 720     // restart the FIFO beyond ~0.3 s of data (6 words per sample with the gyro)
#define FIFO_READ_MAX   2040    // never read more words than this after a shot (FIFO holds 2048)
#define SHOT_WIN_SAMPLES 13     // ~30 ms before the impact: the arrow is still on the string
#define SHOT_RING        160    // samples kept from the FIFO around a shot
// TAP_CFG: interrupts on (0x80), HP filter (0x10), latched (0x01), tap XYZ (0x0E)
#define TAP_CFG_MOTION  0x91
#define TAP_CFG_SHOTS   0x9F
// MD1_CFG: what drives the INT1 line
#define INT1_WAKE_UP    0x20    // motion (bow rests: wakes the board)
#define INT1_SINGLE_TAP 0x40    // tap (shot mode: exact time of the impact)

LSM6DS3 imu(I2C_MODE, 0x6A);
BLEUart bleuart;

// ============================================================================
// State
// ============================================================================
enum LedMode  { MODE_AUTO, MODE_ON, MODE_OFF };
enum LedState { LS_OFF, LS_ON, LS_BLINK, LS_WARN, LS_PULSE };

bool     imuOk             = false;
bool     imuFast           = false;
bool     imuIdleOdr        = false;   // accelerometer at 12.5 Hz (bow rests)
bool     imuBusOn          = false;   // I2C peripheral enabled
bool     twimWorkaround    = false;   // erratum 89 workaround active (decided at start)
NRF_TWIM_Type* imuTwim     = nullptr; // the TWIM instance the IMU hangs on (found at start)
TwoWire* imuWire           = &Wire;
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
uint32_t awakeUntil        = 0;   // app hold: stay reachable until then ("awake <s>")

// Aiming angle (up/down) history, sampled in the fast loop
const uint8_t  PITCH_SAMPLES   = 64;
const uint32_t PITCH_FROM_MS   = 1200;   // average from this long before the release ...
const uint32_t PITCH_TO_MS     = 150;    // ... up to this long before it (the bow moves at release)
float    pitchBuf[PITCH_SAMPLES];
float    cantBuf[PITCH_SAMPLES];        // signed cant at the same moments
uint32_t pitchMs[PITCH_SAMPLES];
uint8_t  pitchHead = 0, pitchFill = 0;
// Angles of the arrows in the open end
const float NO_ANGLE = 1000.0f;          // marks a shot without aiming angle
float    endShotAng[MAX_SCORES_LINE];   // aiming angle per shot of the open end (NO_ANGLE = none)
float    endShotCant[MAX_SCORES_LINE];  // cant per shot of the open end (NO_ANGLE = none)
float    endShotYaw[MAX_SCORES_LINE];   // bow rotation about the vertical during the last 30 ms before impact, deg (NO_ANGLE = none)
float    endShotRoll[MAX_SCORES_LINE];  // rotation about the arrow axis in the same window, deg
float    endShotRate[MAX_SCORES_LINE];  // peak rotation rate in the window, deg/s
uint32_t endShotMs[MAX_SCORES_LINE];    // time of each shot (rhythm of the end)
float    endShotHold[MAX_SCORES_LINE];  // spread of the aiming angle while holding, deg (NO_ANGLE = none)
uint16_t endShotHoldMs[MAX_SCORES_LINE];// how long the aim stayed within HOLD_TOL_DEG before the release, ms
float    endShotDrop[MAX_SCORES_LINE];  // aiming angle in the last 150..400 ms minus the hold mean, deg (sinking = negative)
// Aim trace per shot: the angle history of the ~1.9 s before the release, for the app's shot trace view
const uint8_t TRACE_SAMPLES = 64;
const uint16_t TRACE_SPAN_MS = 1900;
const float HOLD_TOL_DEG = 0.6f;
struct ShotTrace { uint8_t n; uint16_t ms[TRACE_SAMPLES]; int16_t pitch[TRACE_SAMPLES]; int16_t cant[TRACE_SAMPLES]; };
ShotTrace endTrace[MAX_SCORES_LINE];
ShotTrace lastEndTrace[MAX_SCORES_LINE];
// The last closed end, kept for "shot teach" (the app matches arrows to shots after the end closed)
uint16_t lastEndNo = 0; uint8_t lastEndN = 0; uint8_t lastEndDist = 0;
float    lastEndAng[MAX_SCORES_LINE], lastEndCant[MAX_SCORES_LINE], lastEndYaw[MAX_SCORES_LINE], lastEndRoll[MAX_SCORES_LINE], lastEndRate[MAX_SCORES_LINE];
uint32_t lastEndMs[MAX_SCORES_LINE]; float lastEndHold[MAX_SCORES_LINE]; uint16_t lastEndHoldMs[MAX_SCORES_LINE]; float lastEndDrop[MAX_SCORES_LINE];

uint8_t  endShotN = 0;                  // shots in that list
float    lastCant = 0, lastCantMax = 0;  // cant statistics of the end just closed (for the messages)
uint8_t  lastCantN = 0, lastCanted = 0;
uint8_t  endDistM = 0;            // distance given with the last score line (sticky)
uint32_t sessionId = 0;           // log id of the running session, reserved at its start
uint8_t  distSrc = 0;             // distance of the session: 0 = unknown, 1 = set by hand, 2 = recognised

// Range model (physics): aiming angle = zero point of the calibration
//   + launch angle from a flat trajectory with drag + height difference arrow/target.
//   Per setup: s = g / v0^2 (arrow speed) and k (drag, 1/m). Shared: zero points, h.
const uint8_t  RANGE_MAX_ROWS   = 240;    // most recent ends used for learning
const uint8_t  RANGE_MAX_GENS   = 6;      // calibrations with their own zero point
const uint8_t  RANGE_MIN_ARROWS = 3;      // an end needs this many angles to count
const uint8_t  SETUP_MAX        = 8;      // arrow/bow setups (ids 0..7, never reused)
const float    K_PRIOR          = 0.0015f;   // typical drag of an arrow, 1/m
const float    K_PRIOR_SD       = 0.0008f;
const float    H_PRIOR_SD       = 0.3f;      // height difference arrow/target, m
const float    ANGLE_SD         = 0.13f;     // typical spread of one arrow's aiming angle, deg
const uint8_t  RANGE_EXTRA_M    = 20;        // recognise/warn up to 20 m beyond what was learned
struct RangeRow { float d; float th; float w; uint8_t gen; uint8_t setup; };
RangeRow rangeRows[RANGE_MAX_ROWS];
uint8_t  rangeRowN = 0, rangeRowNext = 0;
struct SetupModel {
  uint8_t  state;          // 0 = learning, 1 = needs one end (new calibration), 2 = ready
  float    s, k;           // g / v0^2 and drag
  float    speed;          // v0, m/s
  uint16_t ends;
  uint8_t  dists, minD, maxD;
};
SetupModel sm[SETUP_MAX];
uint8_t  genIds[RANGE_MAX_GENS]; float genOffset[RANGE_MAX_GENS]; uint8_t genN = 0;
float    hShared = 0;
// Active setup's view (used by the rest of the sketch)
uint8_t  rngState = 0;
float    rngOffset = 0, rngSpeed = 0;
uint16_t rngEnds = 0; uint8_t rngDists = 0;

// Setups, stored in internal flash
struct SetupEntry { uint8_t used; char name[19]; };     // used: 0 = free, 1 = in use, 2 = deleted (name kept)
struct SetupTable {
  uint32_t   magic;
  uint8_t    active;
  uint8_t    pad[3];
  SetupEntry s[SETUP_MAX];
};
const uint32_t SETUP_MAGIC = 0x31505453;   // "STP1"

// ----------------------------------------------------------------------------
// Shot matching model, one per setup: learns from arrow/shot pairs the app confirmed
// how the aiming angle maps to height and how the bow's rotation at release maps
// to the sideways position. Stored in /shotmodel.bin.
// ----------------------------------------------------------------------------
struct ShotModel {
  uint16_t n;              // confirmed examples
  uint16_t pad;
  float    spp, spa, saa;  // height: sum pred^2, pred*actual, actual^2 (cm), pred = physics
  float    xx[6], xy[3];   // sideways: normal equations over [yaw, roll, dcant] (deg) -> dx (cm); xx upper triangle
  float    sxx;            // sum dx^2
};
struct ShotModelFile {
  uint32_t  magic;
  uint8_t   on;            // shot matching (gyro in shot mode) enabled
  uint8_t   pad[3];
  ShotModel m[SETUP_MAX];
};
const uint32_t SHOTMODEL_MAGIC = 0x314D5348;   // "HSM1"
const char*    SHOTMODEL_FILE  = "/shotmodel.bin";
ShotModelFile  shotModel;
const float SM_PRIOR_N   = 5.0f;    // weight of the prior residual spread
const float SM_PRIOR_SDY = 4.0f;    // cm: height spread left after the angle, before learning
const float SM_PRIOR_SDX = 15.0f;   // cm: sideways spread before learning (no prediction yet)
const float SM_RIDGE_Y   = 50.0f;   // cm^2: pulls the height factor towards 1 (pure physics) while n is small
const float SM_RIDGE_X   = 2.0f;    // deg^2: pulls the sideways factors towards 0 while n is small

// Kinematics of one shot from the FIFO window before the impact
struct ShotKin { bool ok; float yaw, roll, rate; };
const char*    SETUP_FILE  = "/setups.bin";
SetupTable setups;
int8_t   rangeWarnDir = 0;       // -1 = aiming too low (arrow short), +1 = too high (arrow long)
uint8_t  warnFlashes = 2;        // flashes of the running warning cycle (fixed until it ends)
bool     rangeOk = false;        // aiming angle "on": aim fits the distance (LED pulses)
uint32_t rangeOkSince = 0;
float    sessOffset = 0;                  // zero drift of this session (temperature)
float    sessResSum = 0, sessResW = 0;
bool     rangeWarn = false;
uint32_t rangeBadSince = 0, rangeGoodSince = 0;
float    sessD[MAX_SCORES_LINE], sessTh[MAX_SCORES_LINE], sessW[MAX_SCORES_LINE];
uint8_t  sessSetup[MAX_SCORES_LINE];
uint8_t  sessRowN = 0;                    // this session's ends with a distance set by hand
const char* awakeReason    = "";  // "usb", "app" or "" (for the status)
float    lastShotG         = -1;      // peak of the last counted shot in g (-1 = none yet)
bool     lastShotClip      = false;   // the peak hit the +-16 g limit
volatile bool     int1Flag = false;   // INT1 went high (tap in shot mode, motion otherwise)
volatile uint32_t int1Ms   = 0;       // time of the first edge since the last check
SemaphoreHandle_t wakeSem  = nullptr; // the interrupt ends the idle sleep through this
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

// ----------------------------------------------------------------------------
// State file (counter on/off, epoch, session numbers)
// ----------------------------------------------------------------------------
bool stateLoad() {
  memset(&shotState, 0, sizeof(shotState));
  shotState.magic = STATE_MAGIC;
  File f(InternalFS);
  if (!f.open(STATE_FILE, FILE_O_READ)) return false;
  ShotState tmp;
  memset(&tmp, 0, sizeof(tmp));                       // fields missing in older files stay 0
  const int n = f.read(&tmp, sizeof(tmp));
  const bool ok = n >= (int)offsetof(ShotState, clearedSeq) && tmp.magic == STATE_MAGIC;
  f.close();
  if (ok) shotState = tmp;
  return ok;
}

bool stateSave() {
  InternalFS.remove(STATE_FILE);
  File f(InternalFS);
  if (!f.open(STATE_FILE, FILE_O_WRITE)) return false;
  f.write((const uint8_t*)&shotState, sizeof(shotState));
  f.close();
  return true;
}

// Give a fresh log a random epoch, so the app can tell logs apart
void ensureLogEpoch() {
  if (shotState.epoch != 0) return;
  uint32_t e = 0;
  for (uint8_t tries = 0; tries < 50 && e == 0; tries++) {
    if (sd_rand_application_vector_get((uint8_t*)&e, sizeof(e)) != 0) e = 0;
    if (e == 0) delay(2);
  }
  if (e == 0) e = millis() | 1;   // fallback, should not happen
  shotState.epoch = e;
}

// ----------------------------------------------------------------------------
// Clock from the app
// ----------------------------------------------------------------------------
uint32_t unixAt(uint32_t ms) {
  if (!clockValid) return 0;
  return clockUnix + (int32_t)(ms - clockMs) / 1000;
}

// "2026-09-25 18:03" in the phone's local time
String fmtUnix(uint32_t t) {
  if (t == 0) return "date unknown";
  int32_t s = (int32_t)t + clockTzMin * 60;
  int32_t days = s / 86400, rem = s % 86400;
  if (rem < 0) { rem += 86400; days--; }
  // civil date from days since 1970-01-01 (H. Hinnant)
  days += 719468;
  const int32_t era = (days >= 0 ? days : days - 146096) / 146097;
  const uint32_t doe = (uint32_t)(days - era * 146097);
  const uint32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const uint32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const uint32_t mp  = (5 * doy + 2) / 153;
  const uint32_t d   = doy - (153 * mp + 2) / 5 + 1;
  const uint32_t m   = mp < 10 ? mp + 3 : mp - 9;
  const int32_t  y   = (int32_t)yoe + era * 400 + (m <= 2);
  char buf[20];
  snprintf(buf, sizeof(buf), "%04ld-%02lu-%02lu %02ld:%02ld",
           (long)y, (unsigned long)m, (unsigned long)d, (long)(rem / 3600), (long)((rem / 60) % 60));
  return String(buf);
}

// ----------------------------------------------------------------------------
// External flash (QSPI). Only powered up for an access, then deep power-down.
// ----------------------------------------------------------------------------
bool qspiOn = false;

uint8_t qspiPin(uint32_t arduinoPin) { return (uint8_t)g_ADigitalPinMap[arduinoPin]; }

nrf_qspi_cinstr_conf_t qspiInstr(uint8_t opcode, nrf_qspi_cinstr_len_t len) {
  nrf_qspi_cinstr_conf_t c;
  memset(&c, 0, sizeof(c));
  c.opcode    = opcode;
  c.length    = len;
  c.io2_level = true;    // WP# and HOLD# inactive
  c.io3_level = true;
  c.wipwait   = false;
  c.wren      = false;
  return c;
}

uint8_t qspiReadStatus(uint8_t opcode) {   // 0x05 = status 1, 0x35 = status 2
  uint8_t v[2] = {0, 0};
  nrf_qspi_cinstr_conf_t c = qspiInstr(opcode, NRF_QSPI_CINSTR_LEN_2B);
  nrfx_qspi_cinstr_xfer(&c, NULL, v);
  return v[0];
}

// Wait until the flash finished programming / erasing (WIP bit in status 1)
bool qspiWaitReady(uint32_t timeoutMs) {
  const uint32_t t0 = millis();
  while (qspiReadStatus(0x05) & 0x01) {
    if (millis() - t0 > timeoutMs) { logError = "flash busy timeout"; return false; }
    delay(1);
  }
  return true;
}

// ----------------------------------------------------------------------------
// Talking to the flash by hand (plain GPIO, no QSPI peripheral). Works even when
// the QSPI peripheral cannot start because the chip sleeps or hangs:
// wake it from deep power-down (0xAB) and, if needed, reset it (0x66, 0x99),
// which acts like switching the chip off and on.
// ----------------------------------------------------------------------------
void bbPinsOn() {
  nrf_gpio_cfg_output(qspiPin(PIN_QSPI_CS));   nrf_gpio_pin_set(qspiPin(PIN_QSPI_CS));
  nrf_gpio_cfg_output(qspiPin(PIN_QSPI_SCK));  nrf_gpio_pin_clear(qspiPin(PIN_QSPI_SCK));
  nrf_gpio_cfg_output(qspiPin(PIN_QSPI_IO0));  nrf_gpio_pin_clear(qspiPin(PIN_QSPI_IO0));
  nrf_gpio_cfg_input(qspiPin(PIN_QSPI_IO1), NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_output(qspiPin(PIN_QSPI_IO2));  nrf_gpio_pin_set(qspiPin(PIN_QSPI_IO2));   // WP# inactive
  nrf_gpio_cfg_output(qspiPin(PIN_QSPI_IO3));  nrf_gpio_pin_set(qspiPin(PIN_QSPI_IO3));   // HOLD# inactive
}

void bbPinsOff() {                               // hand the pins back, keep the chip deselected
  nrf_gpio_cfg_default(qspiPin(PIN_QSPI_SCK));
  nrf_gpio_cfg_default(qspiPin(PIN_QSPI_IO0));
  nrf_gpio_cfg_default(qspiPin(PIN_QSPI_IO1));
  nrf_gpio_cfg_default(qspiPin(PIN_QSPI_IO2));
  nrf_gpio_cfg_default(qspiPin(PIN_QSPI_IO3));
}

uint8_t bbByte(uint8_t out) {                    // SPI mode 0, MSB first, ~250 kHz
  uint8_t in = 0;
  for (int8_t b = 7; b >= 0; b--) {
    nrf_gpio_pin_write(qspiPin(PIN_QSPI_IO0), (out >> b) & 1);
    delayMicroseconds(2);
    nrf_gpio_pin_set(qspiPin(PIN_QSPI_SCK));
    delayMicroseconds(2);
    in = (in << 1) | (nrf_gpio_pin_read(qspiPin(PIN_QSPI_IO1)) & 1);
    nrf_gpio_pin_clear(qspiPin(PIN_QSPI_SCK));
  }
  return in;
}

void bbCommand(uint8_t op) {
  nrf_gpio_pin_clear(qspiPin(PIN_QSPI_CS));
  bbByte(op);
  nrf_gpio_pin_set(qspiPin(PIN_QSPI_CS));
  delayMicroseconds(5);
}

// Wake the chip; with fullReset also reset it. Returns the JEDEC id read by hand.
uint32_t qspiHandWake(bool fullReset) {
  bbPinsOn();
  bbCommand(0xFF);                 // leave a possible continuous-read mode
  bbCommand(0xAB);                 // release from deep power-down
  delayMicroseconds(50);
  if (fullReset) {
    bbCommand(0x66);               // reset enable
    bbCommand(0x99);               // reset (aborts a hanging program/erase)
    delay(1);
  }
  nrf_gpio_pin_clear(qspiPin(PIN_QSPI_CS));
  bbByte(0x9F);
  const uint32_t id = ((uint32_t)bbByte(0) << 16) | ((uint32_t)bbByte(0) << 8) | bbByte(0);
  nrf_gpio_pin_set(qspiPin(PIN_QSPI_CS));
  bbPinsOff();
  return id;
}

bool qspiOpen() {
  if (qspiOn) return true;
  nrfx_qspi_config_t c;
  memset(&c, 0, sizeof(c));
  c.xip_offset        = 0;
  c.pins.sck_pin      = qspiPin(PIN_QSPI_SCK);
  c.pins.csn_pin      = qspiPin(PIN_QSPI_CS);
  c.pins.io0_pin      = qspiPin(PIN_QSPI_IO0);
  c.pins.io1_pin      = qspiPin(PIN_QSPI_IO1);
  c.pins.io2_pin      = qspiPin(PIN_QSPI_IO2);
  c.pins.io3_pin      = qspiPin(PIN_QSPI_IO3);
  c.prot_if.readoc    = NRF_QSPI_READOC_FASTREAD;   // single line: no quad-enable bit needed
  c.prot_if.writeoc   = NRF_QSPI_WRITEOC_PP;
  c.prot_if.addrmode  = NRF_QSPI_ADDRMODE_24BIT;
  c.prot_if.dpmconfig = false;
  c.phy_if.sck_delay  = 10;
  c.phy_if.dpmen      = false;
  c.phy_if.spi_mode   = NRF_QSPI_MODE_0;
  c.phy_if.sck_freq   = NRF_QSPI_FREQ_32MDIV4;      // 8 MHz, plenty for 64-byte records
  c.irq_priority      = 7;

  // The QSPI start-up already talks to the chip, so it must be awake first
  qspiHandWake(false);
  nrfx_err_t e = nrfx_qspi_init(&c, NULL, NULL);
  if (e != NRFX_SUCCESS) {
    qspiInitErr = (uint32_t)e;
    nrfx_qspi_uninit();                         // reset the driver's state
    qfBbJedec = qspiHandWake(true);             // reset the chip by hand, then try again
    qspiRecoveries++;
    e = nrfx_qspi_init(&c, NULL, NULL);
    if (e != NRFX_SUCCESS) {
      qspiInitErr = (uint32_t)e;
      nrfx_qspi_uninit();
      return false;
    }
  }
  qspiOn = true;
  nrf_qspi_cinstr_conf_t wake = qspiInstr(0xAB, NRF_QSPI_CINSTR_LEN_1B);  // release deep power-down
  nrfx_qspi_cinstr_xfer(&wake, NULL, NULL);
  delayMicroseconds(50);
  return true;
}

void qspiClose() {
  if (!qspiOn) return;
  qspiWaitReady(400);
  nrf_qspi_cinstr_conf_t dpd = qspiInstr(0xB9, NRF_QSPI_CINSTR_LEN_1B);   // deep power-down
  nrfx_qspi_cinstr_xfer(&dpd, NULL, NULL);
  nrfx_qspi_uninit();
  *(volatile uint32_t*)0x40029054UL = 1;   // nRF52840 anomaly 122: no current after disabling QSPI
  const uint8_t cs = qspiPin(PIN_QSPI_CS);  // keep the chip deselected
  nrf_gpio_cfg_output(cs);
  nrf_gpio_pin_set(cs);
  qspiOn = false;
}

uint32_t qspiReadJedec() {
  uint8_t id[4] = {0, 0, 0, 0};
  nrf_qspi_cinstr_conf_t c = qspiInstr(0x9F, NRF_QSPI_CINSTR_LEN_4B);
  if (nrfx_qspi_cinstr_xfer(&c, NULL, id) != NRFX_SUCCESS) return 0;
  return ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
}

bool qfRead(uint32_t addr, LogRec& r) {
  if (!qspiWaitReady(400)) return false;
  if (nrfx_qspi_read(&r, sizeof(r), addr) != NRFX_SUCCESS) { logError = "read call failed"; return false; }
  return true;
}

bool qfWrite(uint32_t addr, const LogRec& r) {
  if (!qspiWaitReady(400)) return false;
  if (nrfx_qspi_write(&r, sizeof(r), addr) != NRFX_SUCCESS) { logError = "write call failed"; return false; }
  return qspiWaitReady(50);
}

bool qfEraseSector(uint32_t addr) {
  if (!qspiWaitReady(400)) return false;
  if (nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, addr) != NRFX_SUCCESS) { logError = "erase call failed"; return false; }
  return qspiWaitReady(500);
}

// Clear block-protection bits (BP0-BP4 in status 1) if the chip came protected
void qspiUnprotect() {
  qfStatus1 = qspiReadStatus(0x05);
  qfStatus2 = qspiReadStatus(0x35);
  if ((qfStatus1 & 0x7C) == 0) return;
  uint8_t sr[2] = { (uint8_t)(qfStatus1 & ~0x7C), qfStatus2 };   // keep SR2 (quad enable etc.)
  nrf_qspi_cinstr_conf_t c = qspiInstr(0x01, NRF_QSPI_CINSTR_LEN_3B);
  c.wren = true;                                                    // write enable first
  nrfx_qspi_cinstr_xfer(&c, sr, NULL);
  qspiWaitReady(100);
  qfUnprotected = (qspiReadStatus(0x05) & 0x7C) == 0;
}

uint32_t crc32(const void* data, size_t len) {
  const uint8_t* p = (const uint8_t*)data;
  uint32_t c = 0xFFFFFFFF;
  while (len--) {
    c ^= *p++;
    for (uint8_t k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320 & (0 - (c & 1)));
  }
  return ~c;
}

bool recKnown(const LogRec& r) { return r.magic == REC_MAGIC || r.magic == END_MAGIC; }
bool recValid(const LogRec& r) {
  return recKnown(r) && r.seq != 0 && r.crc == crc32(&r, 60);
}

// A deleted record: its CRC word was programmed to 0 (flash can clear bits
// without an erase). It keeps its place, so the seq numbers stay gapless.
bool recDeleted(const LogRec& r) {
  return recKnown(r) && r.seq != 0 && r.crc == 0;
}
bool recUsed(const LogRec& r) { return recValid(r) || recDeleted(r); }

bool recBlank(const LogRec& r) {
  const uint32_t* w = (const uint32_t*)&r;
  for (uint8_t i = 0; i < sizeof(LogRec) / 4; i++) if (w[i] != 0xFFFFFFFF) return false;
  return true;
}

// Seq numbers on the chip are gapless from the oldest sector to the newest record
// (seq only advances on a verified write, and whole sectors are erased from the
// oldest end), so the count follows from the smallest first-seq.
void logRecount() {
  uint32_t oldest = 0;
  for (uint16_t s = 0; s < QF_SECTORS; s++) {
    if (sectorFirstSeq[s] && (oldest == 0 || sectorFirstSeq[s] < oldest)) oldest = sectorFirstSeq[s];
  }
  logCount = oldest ? logLastSeq - oldest + 1 : 0;
}

// Find the newest record and the next write position (reads 512 + up to 64 records).
// Invariant: every sector holds a gapless prefix of valid records; anything
// unexpected makes the writer continue in the next sector.
void logScan() {
  logCountChanged();
  memset(sectorFirstSeq, 0, sizeof(sectorFirstSeq));
  logHeadAddr = 0; logLastSeq = 0; logCount = 0;
  int16_t head = -1;
  uint32_t best = 0;
  LogRec r;
  for (uint16_t s = 0; s < QF_SECTORS; s++) {
    if (qfRead((uint32_t)s * QF_SECTOR, r) && recUsed(r)) {
      sectorFirstSeq[s] = r.seq;
      if (r.seq > best) { best = r.seq; head = s; }
    }
  }
  if (head < 0) return;                     // empty log
  uint16_t i = 0;
  for (; i < RECS_PER_SECTOR; i++) {
    if (!qfRead((uint32_t)head * QF_SECTOR + i * sizeof(LogRec), r) || !recUsed(r)) break;
    logLastSeq = r.seq;
  }
  logHeadAddr = (i < RECS_PER_SECTOR) ? (uint32_t)head * QF_SECTOR + i * sizeof(LogRec)
                                      : (uint32_t)((head + 1) % QF_SECTORS) * QF_SECTOR;
  logRecount();
}

// Append one record; fills magic, seq and crc. Returns false on a write error.
// justSaved: a session that ended now (its age is reported to the app); false
// for sessions taken over or imported, whose age is unknown.
bool logAppend(LogRec& r, bool justSaved) {
  if (!qfOk) { logError = "log flash not available"; return false; }
  if (!qspiOpen()) { logError = "QSPI init failed"; return false; }
  logError = "";
  bool ok = false;
  for (uint8_t attempt = 0; attempt < 3 && !ok; attempt++) {
    const uint16_t sec = logHeadAddr / QF_SECTOR;
    LogRec probe;
    if (!qfRead(logHeadAddr, probe)) break;
    if (!recBlank(probe)) {
      // Starting a sector with old data (ring wrapped, or factory content): erase it
      if (logHeadAddr % QF_SECTOR != 0) {        // garbage mid-sector: continue in the next sector
        logHeadAddr = (uint32_t)((sec + 1) % QF_SECTORS) * QF_SECTOR;
        continue;
      }
      sectorFirstSeq[sec] = 0;
      if (!qfEraseSector(logHeadAddr)) break;
      logRecount();
    }
    if (r.magic != END_MAGIC) {                // end records bring their own layout
      memset(r.reserved, 0xFF, sizeof(r.reserved));
      r.magic = REC_MAGIC;
    }
    r.seq   = logLastSeq + 1;
    r.crc   = crc32(&r, 60);
    LogRec check;
    ok = qfWrite(logHeadAddr, r) && qfRead(logHeadAddr, check);
    if (ok && memcmp(&check, &r, sizeof(r)) != 0) { ok = false; logError = "read-back differs (write protected?)"; }
    if (ok) {
      if (logHeadAddr % QF_SECTOR == 0) sectorFirstSeq[sec] = r.seq;
      logLastSeq = r.seq;
      logRecount();
      logHeadAddr += sizeof(LogRec);
      if (logHeadAddr % QF_SECTOR == 0) logHeadAddr = (uint32_t)((sec + 1) % QF_SECTORS) * QF_SECTOR;
    } else {
      // A failed write leaves a broken slot: continue in the next sector so
      // every sector stays a gapless run of valid records
      logHeadAddr = (uint32_t)((sec + 1) % QF_SECTORS) * QF_SECTOR;
    }
  }
  qspiClose();
  if (ok) logCountChanged();
  if (ok && justSaved) {
    recentSeq[recentNext] = r.seq;
    recentMs[recentNext]  = millis();
    recentNext = (recentNext + 1) % RECENT_SAVES;
  }
  return ok;
}

// Call fn for every record with seq > since, oldest first. Stops when fn returns false.
uint32_t logVisitAddr = 0;   // flash address of the record passed to the visitor

#define LOG_SESSIONS 0x01
#define LOG_ENDS     0x02
void logForEachKind(uint32_t since, LogVisitor fn, uint8_t kinds);
void logForEach(uint32_t since, LogVisitor fn) { logForEachKind(since, fn, LOG_SESSIONS); }

void logForEachKind(uint32_t since, LogVisitor fn, uint8_t kinds) {
  if (since < shotState.clearedSeq) since = shotState.clearedSeq;   // "log clear"
  if (!qfOk || logCount == 0 || !qspiOpen()) return;
  const uint16_t headSec = (logHeadAddr / QF_SECTOR + QF_SECTORS - (logHeadAddr % QF_SECTOR == 0 ? 1 : 0)) % QF_SECTORS;
  LogRec r;
  bool go = true;
  for (uint16_t n = 1; n <= QF_SECTORS && go; n++) {
    const uint16_t s = (headSec + n) % QF_SECTORS;          // oldest sector first
    const uint32_t first = sectorFirstSeq[s];
    if (!first || first + RECS_PER_SECTOR - 1 <= since) continue;
    uint16_t i = (since >= first) ? (uint16_t)(since - first + 1) : 0;
    for (; i < RECS_PER_SECTOR && go; i++) {
      const uint32_t addr = (uint32_t)s * QF_SECTOR + i * sizeof(LogRec);
      if (!qfRead(addr, r) || !recUsed(r)) break;
      if (recDeleted(r) || r.seq <= since) continue;
      if (!(kinds & (r.magic == END_MAGIC ? LOG_ENDS : LOG_SESSIONS))) continue;
      logVisitAddr = addr;
      go = fn(r);
    }
  }
  qspiClose();
}

// Sessions the app and the terminal can see (without deleted and cleared ones).
// Counting reads the whole chip, so the result is kept until the log changes.
static uint32_t visibleN;
static bool     visibleValid = false;
void logCountChanged() { visibleValid = false; }
uint32_t logVisibleCount() {
  if (visibleValid) return visibleN;
  visibleN = 0;
  logForEach(0, [](const LogRec&) { visibleN++; return true; });
  visibleValid = true;
  return visibleN;
}

// "log del <epoch> <id>": mark one session as deleted
static uint32_t delEpoch, delId, delAddr;
void logDelete(String args) {
  args.trim();
  const int sp = args.indexOf(' ');
  delEpoch = (uint32_t)strtoul((sp < 0 ? args : args.substring(0, sp)).c_str(), nullptr, 10);
  delId    = sp < 0 ? 0 : (uint32_t)strtoul(args.substring(sp + 1).c_str(), nullptr, 10);
  const String key = "{\"t\":\"ack\",\"cmd\":\"del\",\"epoch\":" + String(delEpoch) +
                     ",\"id\":" + String(delId) + ",\"result\":";
  if (!delEpoch || !delId) { say("Format: log del <epoch> <id>", key + "\"error\"}"); return; }
  if (!qfOk) { say("Log flash not available.", key + "\"error\"}"); return; }
  delAddr = 0;
  logForEach(0, [](const LogRec& r) {
    if (r.epoch == delEpoch && r.id == delId) { delAddr = logVisitAddr; return false; }
    return true;
  });
  if (!delAddr) {
    say("Session " + String(delEpoch) + "-" + String(delId) + " is not on the sight.", key + "\"notfound\"}");
    return;
  }
  bool ok = false;
  if (qspiOpen()) {
    LogRec r;
    r.crc = 0;                                         // only the last word is written
    ok = qspiWaitReady(400) &&
         nrfx_qspi_write(&r.crc, 4, delAddr + offsetof(LogRec, crc)) == NRFX_SUCCESS &&
         qspiWaitReady(50) && qfRead(delAddr, r) && recDeleted(r);
    qspiClose();
  }
  if (ok) {                                           // its ends go as well
    logCountChanged();
    static uint32_t endAddrs[64];
    static uint8_t  endN;
    endN = 0;
    logForEachKind(0, [](const LogRec& r) {
      const EndRec& e = *(const EndRec*)&r;
      if (e.epoch == delEpoch && e.id == delId && endN < 64) endAddrs[endN++] = logVisitAddr;
      return true;
    }, LOG_ENDS);
    if (endN && qspiOpen()) {
      for (uint8_t k = 0; k < endN; k++) {
        LogRec z;
        z.crc = 0;
        qspiWaitReady(400);
        nrfx_qspi_write(&z.crc, 4, endAddrs[k] + offsetof(LogRec, crc));
        qspiWaitReady(50);
      }
      qspiClose();
    }
  }
  say(ok ? "Session " + String(delEpoch) + "-" + String(delId) + " deleted from the sight."
         : String("ERROR: could not delete the session."),
      key + (ok ? "\"deleted\"}" : "\"error\"}"));
}

// "log clear confirm": everything stored so far counts as deleted (instant;
// the records are physically overwritten later as the ring moves on)
void logClear(String args) {
  args.trim();
  if (args != "confirm") {
    out("This deletes ALL sessions on the sight. Send 'log clear confirm' to do it.");
    return;
  }
  const uint32_t n = logVisibleCount();
  shotState.clearedSeq = logLastSeq;
  logCountChanged();
  const bool ok = stateSave();
  say(ok ? "Log on the sight cleared (" + String(n) + (n == 1 ? " session)." : " sessions).") : String("ERROR while saving!"),
      "{\"t\":\"ack\",\"cmd\":\"clear\",\"ok\":" + jbool(ok) + ",\"count\":" + String(n) + "}");
}

// Step-by-step check of the external flash, uses its last sector only
void logTest() {
  auto step = [](const String& name, bool ok, const String& detail) {
    say(name + ": " + (ok ? "ok" : "FAILED") + (detail.length() ? " (" + detail + ")" : ""),
        "{\"t\":\"logtest\",\"step\":" + jstr(name) + ",\"ok\":" + jbool(ok) + ",\"detail\":" + jstr(detail) + "}");
  };
  logError = "";
  const uint32_t t0 = millis();
  char buf[64];
  if (qspiOn) qspiClose();
  const uint32_t hj = qspiHandWake(true);
  snprintf(buf, sizeof(buf), "JEDEC %06lX without QSPI", (unsigned long)hj);
  step("wake + reset by hand", hj == 0x856015, buf);
  const uint16_t r0 = qspiRecoveries;
  const bool opened = qspiOpen();
  snprintf(buf, sizeof(buf), "error 0x%08lX%s", (unsigned long)qspiInitErr,
           qspiRecoveries != r0 ? ", needed a second try" : "");
  step("QSPI init", opened, opened && qspiRecoveries == r0 ? String("") : String(buf));
  if (!opened) return;
  const uint32_t jed = qspiReadJedec();
  snprintf(buf, sizeof(buf), "%06lX, expected 856015", (unsigned long)jed);
  step("JEDEC id", jed != 0 && jed != 0xFFFFFF, buf);
  const uint8_t s1 = qspiReadStatus(0x05), s2 = qspiReadStatus(0x35);
  snprintf(buf, sizeof(buf), "SR1 %02X, SR2 %02X%s", s1, s2, (s1 & 0x7C) ? ", PROTECTED" : "");
  step("status", (s1 & 0x7C) == 0 && !(s1 & 0x01), buf);
  bool ok = qfEraseSector(QF_TEST_ADDR);
  step("erase test sector", ok, logError);
  LogRec w, r;
  memset(&w, 0xA5, sizeof(w));
  w.seq = millis();
  ok = ok && qfWrite(QF_TEST_ADDR, w);
  step("write", ok, logError);
  ok = ok && qfRead(QF_TEST_ADDR, r);
  const bool same = ok && memcmp(&w, &r, sizeof(w)) == 0;
  snprintf(buf, sizeof(buf), "first bytes %02X %02X %02X %02X", ((uint8_t*)&r)[0], ((uint8_t*)&r)[1],
           ((uint8_t*)&r)[2], ((uint8_t*)&r)[3]);
  step("read back", same, buf);
  qspiClose();
  step("done", same, String(millis() - t0) + " ms");
}

// Is a session with this log key already on the chip? (reads all records)
static uint32_t findEpoch, findId;
static bool     findHit;
bool logHasKey(uint32_t epoch, uint32_t id) {
  findEpoch = epoch; findId = id; findHit = false;
  logForEach(0, [](const LogRec& r) {
    if (r.epoch == findEpoch && r.id == findId) { findHit = true; return false; }
    return true;
  });
  return findHit;
}

// Take over the internal-flash log of firmware 1.2 - 2.1 once
void migrateLegacyLog(bool stateFound) {
  File f(InternalFS);
  if (!f.open(LEGACY_FILE, FILE_O_READ)) return;
  LegacyLog* L = (LegacyLog*)malloc(sizeof(LegacyLog));
  if (!L) { f.close(); return; }
  const bool ok = f.read(L, sizeof(LegacyLog)) == (int)sizeof(LegacyLog) && L->magic == LEGACY_MAGIC;
  f.close();
  if (!ok) { free(L); return; }             // unknown format: leave the file alone

  if (!stateFound) {                        // counter state and numbering come along
    shotState.shotsOn = L->shotsOn;
    shotState.epoch   = L->epoch;
    shotState.lastId  = L->seq;
    stateSave();
  }

  if (qfOk && logCount == 0) {
    uint8_t order[LEGACY_SLOTS], used = 0;
    for (uint8_t i = 0; i < LEGACY_SLOTS; i++) if (L->slots[i].flags & 0x01) order[used++] = i;
    for (uint8_t i = 1; i < used; i++) {     // oldest first (by session number)
      const uint8_t k = order[i];
      int8_t j = i - 1;
      while (j >= 0 && L->slots[order[j]].id > L->slots[k].id) { order[j + 1] = order[j]; j--; }
      order[j + 1] = k;
    }
    uint8_t written = 0;
    for (uint8_t i = 0; i < used; i++) {
      const LegacySlot& s = L->slots[order[i]];
      LogRec r;
      memset(&r, 0, sizeof(r));
      r.epoch = L->epoch;  r.id = s.id;  r.start = 0;
      r.shots = s.shots;   r.scores = s.scores;  r.avgX100 = s.avgX100;  r.minutes = s.minutes;
      r.ends = s.ends;     r.invalidEnds = s.invalidEnds;  r.invalidArrows = s.invalidArrows;
      r.xCount = s.xCount; r.flags = s.flags & (SLOT_MISMATCH | SLOT_MANUAL);
      if (logAppend(r, false)) written++;
    }
    if (written == used) {                  // verified: keep the old file as a backup copy
      InternalFS.remove(LEGACY_BACKUP);
      InternalFS.rename(LEGACY_FILE, LEGACY_BACKUP);
      migratedCount = written;
    }
  }
  free(L);
}

void logInit() {
  qfOk = false;
  if (qspiOpen()) {
    qfJedec = qspiReadJedec();
    qfOk = qfJedec != 0 && qfJedec != 0xFFFFFF;
    if (qfOk) { qspiUnprotect(); logScan(); }
    qspiClose();                             // deep power-down right away
  }
  const bool stateFound = stateLoad();
  migrateLegacyLog(stateFound);
}

void levelLoad() {
  memset(&lvl, 0, sizeof(lvl));
  lvl.magic = LEVEL_MAGIC;
  lvl.cantSignal = 1; lvl.rangeSignal = 1;
  File f(InternalFS);
  if (f.open(LEVEL_FILE, FILE_O_READ)) {
    LevelState tmp;
    memset(&tmp, 0, sizeof(tmp));
    const int n = f.read(&tmp, sizeof(tmp));
    if (n >= (int)offsetof(LevelState, calGen) && tmp.magic == LEVEL_MAGIC) {
      if (n < (int)sizeof(tmp)) {             // file from before firmware 5.0
        tmp.calGen = 0; tmp.cantSignal = 1; tmp.rangeSignal = 1; tmp.ledMode = MODE_AUTO;
      }
      lvl = tmp;
    }
    f.close();
  }
  if (!ledPresent) { lvl.cantSignal = 0; lvl.rangeSignal = 0; lvl.ledMode = MODE_OFF; }   // nothing to light up
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
// Write the thresholds for the current range. Settings stay in their units:
// ths in 125 mg, tap_ths in 250 mg. In shot mode (+-16 g) one register step
// is twice as large, so the values are halved (rounded up).
void applyImuThresholds() {
  if (!imuOk) return;
  uint8_t wake = imuFast ? (uint8_t)((cfg.wakeThs + 1) / 2) : cfg.wakeThs;
  uint8_t tap  = (uint8_t)((cfg.tapThs + 1) / 2);   // tap only runs in shot mode
  if (wake < 1) wake = 1;
  if (tap  < 1) tap  = 1;
  if (tap  > 31) tap = 31;
  imu.writeRegister(REG_WAKE_UP_THS, wake & 0x3F);   // bit7=0: single tap only
  imu.writeRegister(REG_TAP_THS_6D,  tap  & 0x1F);
}

void fifoRestart() {
  imu.writeRegister(REG_FIFO_CTRL5, FIFO_BYPASS);    // clears the FIFO
  imu.writeRegister(REG_FIFO_CTRL5, FIFO_416_CONT);
}

uint16_t fifoWords() {
  uint8_t b[2] = {0, 0};
  if (imu.readRegisterRegion(b, REG_FIFO_STAT1, 2) != IMU_SUCCESS) return 0;
  return b[0] | ((b[1] & 0x0F) << 8);          // DIFF_FIFO is 12 bits
}

bool gyroWanted() { return shotModel.on && lvl.calibrated; }
bool fifoHasGyro = false;        // layout the FIFO was started with (follows gyroWanted() in the loop)

// Reads the FIFO after a shot: peak of the total acceleration in g (the impact), and with
// the gyro on, the bow's rotation in the ~30 ms before the impact (yaw about the vertical,
// roll about the arrow axis, peak rate). Restarts the FIFO.
float readShotFifo(bool* clipped, ShotKin* kin) {
  *clipped = false;
  kin->ok = false; kin->yaw = kin->roll = kin->rate = 0;
  uint8_t st[4] = {0, 0, 0, 0};
  if (imu.readRegisterRegion(st, REG_FIFO_STAT1, 4) != IMU_SUCCESS) return -1;
  uint16_t n   = st[0] | ((st[1] & 0x0F) << 8);
  const bool withG = fifoHasGyro;
  const uint8_t per = withG ? 6 : 3;                        // words per sample: Gx Gy Gz Ax Ay Az, or Ax Ay Az
  uint16_t pat = (st[2] | ((st[3] & 0x03) << 8)) % per;     // position of the next word in the pattern
  if (n > FIFO_READ_MAX) n = FIFO_READ_MAX;

  static int16_t ring[SHOT_RING][6];                        // the last samples, oldest first after unrolling
  int16_t cur[6] = {0, 0, 0, 0, 0, 0};
  uint8_t have = 0;
  uint16_t count = 0;                                       // complete samples seen
  float peak2 = 0; int32_t peakAt = -1;
  for (uint16_t i = 0; i < n; i++) {
    uint8_t b[2];
    if (imu.readRegisterRegion(b, REG_FIFO_DATA, 2) != IMU_SUCCESS) break;
    const int16_t w = (int16_t)(b[0] | (b[1] << 8));
    const uint8_t slot = withG ? pat : pat + 3;             // accel always lands in 3..5
    if (slot >= 3 && (w == 32767 || w == -32768)) *clipped = true;
    cur[slot] = w;
    have |= (1 << slot);
    if (slot == 5 && (have & 0x38) == 0x38 && (!withG || (have & 0x07) == 0x07)) {
      memcpy(ring[count % SHOT_RING], cur, sizeof(cur));
      const float x = cur[3], y = cur[4], z = cur[5];
      const float m2 = x * x + y * y + z * z;
      if (m2 > peak2) { peak2 = m2; peakAt = count; }
      count++;
      have = 0;
    }
    pat = (pat + 1) % per;
  }
  fifoRestart();

  // Rotation in the window just before the impact, projected on the bow's axes
  if (withG && peakAt > SHOT_WIN_SAMPLES && count >= (uint16_t)peakAt + 1 && (count < SHOT_RING || (int32_t)count - peakAt <= SHOT_RING - SHOT_WIN_SAMPLES - 1)) {
    float F[3] = { lvl.L[1] * lvl.D[2] - lvl.L[2] * lvl.D[1], lvl.L[2] * lvl.D[0] - lvl.L[0] * lvl.D[2], lvl.L[0] * lvl.D[1] - lvl.L[1] * lvl.D[0] };
    if (vnorm(F)) {
      float rot[3] = {0, 0, 0}; float peakRate = 0;
      const float dt = 1.0f / 416.0f;
      for (int32_t s = peakAt - SHOT_WIN_SAMPLES; s < peakAt; s++) {
        const int16_t* r = ring[s % SHOT_RING];
        const float w[3] = { r[0] * DPS_PER_LSB_1000, r[1] * DPS_PER_LSB_1000, r[2] * DPS_PER_LSB_1000 };
        for (uint8_t k = 0; k < 3; k++) rot[k] += w[k] * dt;
        const float m = sqrtf(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
        if (m > peakRate) peakRate = m;
      }
      kin->yaw  = vdot(rot, lvl.D);    // about the vertical: the sideways swing of the bow
      kin->roll = vdot(rot, F);        // about the arrow: torque / cant change at release
      kin->rate = peakRate;
      kin->ok   = true;
    }
  }
  return sqrtf(peak2) * G_PER_LSB_16G;
}

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
  imu.writeRegister(REG_INT_DUR2,    0x07);               // max shock window, short quiet
  imu.writeRegister(REG_FIFO_CTRL3,  FIFO_XL_ONLY);       // FIFO only used in shot mode
  imu.writeRegister(REG_FIFO_CTRL5,  FIFO_BYPASS);
  imu.writeRegister(REG_TAP_CFG,     TAP_CFG_MOTION);     // tap off for now
  imu.writeRegister(REG_MD1_CFG,     INT1_WAKE_UP);       // motion on INT1 until shot mode routes the tap

  uint8_t dummy;
  imu.readRegister(&dummy, REG_WAKE_UP_SRC);
  imu.readRegister(&dummy, REG_TAP_SRC);
  imuFast = false;
  imuIdleOdr = false;
  imuOk = true;                   // needed by applyImuThresholds()
  applyImuThresholds();
  // Which I2C peripheral the library opened for the IMU (Wire = TWIM0, Wire1 = TWIM1)
  if (NRF_TWIM0->ENABLE == TWIM_ENABLE_ENABLE_Enabled) { imuTwim = NRF_TWIM0; imuWire = &Wire; }
#if WIRE_INTERFACES_COUNT > 1
  else if (NRF_TWIM1->ENABLE == TWIM_ENABLE_ENABLE_Enabled) { imuTwim = NRF_TWIM1; imuWire = &Wire1; }
#endif
  imuBusOn = true;
#if TWIM_ANOMALY_89 == 2
  twimWorkaround = true;
#elif TWIM_ANOMALY_89 == 1 && defined(HAVE_NRF52_ERRATAS)
  twimWorkaround = nrf52_errata_89();       // only the Engineering A revision has it
#endif
  int1Init();
  return true;
}

// I2C bus on/off around the IMU accesses of one loop pass (erratum 89, see above)
void imuBus(bool on) {
#if TWIM_ANOMALY_89
  if (!twimWorkaround || !imuOk || !imuTwim || on == imuBusOn) return;
  if (on) {
    imuWire->begin();
  } else {
    imuWire->end();
    volatile uint32_t* pwr = (volatile uint32_t*)((uintptr_t)imuTwim + 0xFFC);   // POWER register of the peripheral
    *pwr = 0;
    (void)*pwr;
    *pwr = 1;
  }
  imuBusOn = on;
#else
  (void)on;
#endif
}

// Accelerometer rate while not in shot mode: 12.5 Hz when the bow rests, 26 Hz in use
void imuSetIdleOdr(bool idle) {
  if (!imuOk || imuFast || idle == imuIdleOdr) return;
  imu.writeRegister(REG_CTRL1_XL, idle ? XL_12HZ_8G : XL_26HZ_8G);
  imuIdleOdr = idle;
}

// INT1 interrupt. The line is watched with a GPIOTE PORT event (pin sense): unlike
// an IN event it needs no high-frequency clock, so it costs nothing while the
// board sleeps. The handler only notes the time and wakes the loop; what the
// edge meant (tap or motion) is read from the IMU in the loop.
// The sketch owns the GPIOTE interrupt: attachInterrupt() must not be used.
static inline uint32_t millisFromIsr() {
  return (uint32_t)(((uint64_t)xTaskGetTickCountFromISR() * 1000) / configTICK_RATE_HZ);
}

extern "C" void GPIOTE_IRQHandler(void) {
  if (NRF_GPIOTE->EVENTS_PORT) {
    NRF_GPIOTE->EVENTS_PORT = 0;
    (void)NRF_GPIOTE->EVENTS_PORT;                 // make sure the clear reached the peripheral
    if (!int1Flag) int1Ms = millisFromIsr();
    int1Flag = true;
    if (wakeSem) {
      BaseType_t woken = pdFALSE;
      xSemaphoreGiveFromISR(wakeSem, &woken);
      portYIELD_FROM_ISR(woken);
    }
  }
}

void int1Init() {
#ifdef PIN_LSM6DS3TR_C_INT1
  wakeSem = xSemaphoreCreateBinary();
  const uint32_t pin = g_ADigitalPinMap[PIN_LSM6DS3TR_C_INT1];
  nrf_gpio_cfg_sense_input(pin, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);
  NRF_GPIOTE->EVENTS_PORT = 0;
  NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
  NVIC_ClearPendingIRQ(GPIOTE_IRQn);
  NVIC_SetPriority(GPIOTE_IRQn, 6);             // an application level allowed next to the SoftDevice
  NVIC_EnableIRQ(GPIOTE_IRQn);
#endif
}

// Idle: sleep until INT1 reports motion, at most IDLE_WAKE_MAX_MS for housekeeping
void idleSleep() {
  if (wakeSem) {
    xSemaphoreTake(wakeSem, 0);                  // drop a stale wake-up
    if (!int1Flag) xSemaphoreTake(wakeSem, pdMS_TO_TICKS(IDLE_WAKE_MAX_MS));
    int1Flag = false;                            // the loop reads WAKE_UP_SRC next
    return;
  }
  delay(IDLE_POLL_MS);                           // no INT1 pin: poll as before
}

// Fast mode (416 Hz) only for shot detection, otherwise low power (26 Hz)
void imuSetFast(bool fast) {
  if (!imuOk || fast == imuFast) return;

  if (fast) {
    imu.writeRegister(REG_CTRL6_C,  0x00);                // high performance
    imu.writeRegister(REG_CTRL1_XL, XL_416HZ_16G);
    fifoHasGyro = gyroWanted();
    imu.writeRegister(REG_CTRL2_G,  fifoHasGyro ? G_416HZ_1000DPS : G_OFF);
    imu.writeRegister(REG_FIFO_CTRL3, fifoHasGyro ? FIFO_XL_G : FIFO_XL_ONLY);
    imu.writeRegister(REG_TAP_CFG,  TAP_CFG_SHOTS);
  } else {
    imu.writeRegister(REG_FIFO_CTRL5, FIFO_BYPASS);
    imu.writeRegister(REG_CTRL2_G,  G_OFF);               // gyro only in shot mode
    imu.writeRegister(REG_FIFO_CTRL3, FIFO_XL_ONLY);
    fifoHasGyro = false;
    imu.writeRegister(REG_TAP_CFG,  TAP_CFG_MOTION);
    imu.writeRegister(REG_CTRL1_XL, XL_26HZ_8G);
    imu.writeRegister(REG_CTRL6_C,  0x10);
    imuIdleOdr = false;
  }
  imu.writeRegister(REG_MD1_CFG, fast ? INT1_SINGLE_TAP : INT1_WAKE_UP);   // what INT1 reports
  imuFast = fast;
  applyImuThresholds();
  if (fast) fifoRestart();

  // Let the filters settle, discard false triggers
  const uint32_t now = millis();
  ignoreMotionUntil = now + IMU_SETTLE_MS;
  lastShotMs = now;
  uint8_t dummy;
  imu.readRegister(&dummy, REG_WAKE_UP_SRC);
  imu.readRegister(&dummy, REG_TAP_SRC);   // clears the latched INT1 line
  int1Flag = false;
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
// Battery history and runtime estimate
// ============================================================================
void bhLoad() {
  memset(&bh, 0, sizeof(bh));
  bh.magic = BH_MAGIC;
  File f(InternalFS);
  if (!f.open(BH_FILE, FILE_O_READ)) return;
  BatHist* tmp = (BatHist*)malloc(sizeof(BatHist));
  if (tmp) {
    if (f.read(tmp, sizeof(BatHist)) == (int)sizeof(BatHist) && tmp->magic == BH_MAGIC &&
        tmp->next < BH_POINTS && tmp->count <= BH_POINTS) bh = *tmp;
    free(tmp);
  }
  f.close();
}

bool bhSaveFile() {
  InternalFS.remove(BH_FILE);
  File f(InternalFS);
  if (!f.open(BH_FILE, FILE_O_WRITE)) return false;
  f.write((const uint8_t*)&bh, sizeof(bh));
  f.close();
  return true;
}

// i = 0 is the oldest point
const BatPoint& bhAt(uint16_t i) {
  return bh.p[(bh.next + BH_POINTS - bh.count + i) % BH_POINTS];
}

// Add time to the accumulators; called on every loop pass
void bhTick(uint32_t now, bool awake) {
  const uint32_t dt = now - bhTickMs;
  bhTickMs = now;
  if (dt > 60000) return;                    // implausible gap (should not happen)
  if (bhPrevAwake) bhActiveMs += dt; else bhIdleMs += dt;
  if (ledMaNow > 0) { bhLedMAs += ledMaNow * dt / 1000.0f; bhLedMs += dt; }
  bhPrevAwake = awake;
}

void bhFit();   // below

// Store a point; the minimum gap is skipped for anchors (force)
void bhPoint(uint8_t type, uint16_t mv, bool force) {
  const uint32_t now = millis();
  if (!force && bhLastPointMs && (now - bhLastPointMs) < BH_MIN_GAP_MS) return;
  BatPoint& p = bh.p[bh.next];
  p.t       = clockValid ? unixAt(now) : 0;
  p.mv      = mv;
  p.type    = type;
  p.flags   = (usbPresent() ? BPF_USB : 0) | (clockValid ? BPF_CLOCK : 0);
  p.idleS   = (uint32_t)(bhIdleMs / 1000);
  p.activeS = (uint32_t)(bhActiveMs / 1000);
  p.ledMAs  = (uint32_t)bhLedMAs;
  p.ledS    = (uint32_t)(bhLedMs / 1000);
  bh.next = (bh.next + 1) % BH_POINTS;
  if (bh.count < BH_POINTS) bh.count++;
  bhIdleMs = bhActiveMs = bhLedMAs = bhLedMs = 0;
  bhLastPointMs = now;
  bhSaveFile();
  bhFit();
}

// Measure without load: now if the LED has been off long enough, else later
void bhPointUnloaded(uint8_t type, bool force) {
  if (ledMaNow > 0 || (millis() - ledOffMs) < BH_RELAX_MS) {
    bhPending = type;                        // a newer request replaces an older one
    bhPendingForce = bhPendingForce || force;
    return;
  }
  bhPoint(type, (uint16_t)(readVbat() * 1000), force);
}

// Pending measurements and the voltage kept fresh while asleep; every loop pass
void bhService(uint32_t now, bool usb) {
  if (!usb && ledMaNow == 0 && (now - ledOffMs) >= BH_RELAX_MS) {
    if (bhPending) {
      const uint16_t mv = (uint16_t)(readVbat() * 1000);
      unloadedMv = mv; unloadedMs = now;
      bhPoint(bhPending, mv, bhPendingForce);
      bhPending = 0; bhPendingForce = false;
    } else if ((now - unloadedMs) >= BH_IDLE_READ_MS) {
      unloadedMv = (uint16_t)(readVbat() * 1000);
      unloadedMs = now;
    }
  }
}

// Solve a 3x3 system (Gauss); false if singular
bool solve3(float A[3][3], float b[3], float x[3]) {
  for (uint8_t c = 0; c < 3; c++) {
    uint8_t piv = c;
    for (uint8_t r = c + 1; r < 3; r++) if (fabsf(A[r][c]) > fabsf(A[piv][c])) piv = r;
    if (fabsf(A[piv][c]) < 1e-9f) return false;
    for (uint8_t k = 0; k < 3; k++) { float t = A[c][k]; A[c][k] = A[piv][k]; A[piv][k] = t; }
    float t = b[c]; b[c] = b[piv]; b[piv] = t;
    for (uint8_t r = 0; r < 3; r++) {
      if (r == c) continue;
      const float f = A[r][c] / A[c][c];
      for (uint8_t k = 0; k < 3; k++) A[r][k] -= f * A[c][k];
      b[r] -= f * b[c];
    }
  }
  for (uint8_t c = 0; c < 3; c++) x[c] = b[c] / A[c][c];
  return true;
}

// Fit idle current, awake current and an LED factor from discharge windows:
//   used mAh = idleMa * idleH + activeMa * activeH + ledK * ledModelMah
// Each row covers at least BH_WINDOW_PCT of discharge. Priors (the former
// estimates) keep the result sensible while there is little data.
void bhFit() {
  const float prior[3] = { 0.06f, 0.8f, 1.0f };
  const float scale[3] = { 24.0f, 5.0f, 20.0f };      // prior counts like one such observation
  float A[3][3] = {{0}}, b[3] = {0};
  for (uint8_t i = 0; i < 3; i++) { A[i][i] = scale[i] * scale[i]; b[i] = scale[i] * scale[i] * prior[i]; }

  uint16_t rows = 0; float mah = 0;
  bool open = false; float socStart = 0, x0 = 0, x1 = 0, x2 = 0;
  // charge learning: minutes from plug-in to full vs. state of charge at plug-in
  float C[2][2] = {{30.0f * 30.0f, 0}, {0, 3.0f * 3.0f}};
  float cb[2]   = {30.0f * 30.0f * (cfg.batMah / 50.0f * 60.0f / 100.0f), 3.0f * 3.0f * 20.0f};
  uint16_t charges = 0; float chgSoc = -1, chgMin = 0;

  for (uint16_t i = 0; i < bh.count; i++) {
    const BatPoint& p = bhAt(i);
    const float soc = rawPercent(p.mv / 1000.0f);
    // Charging and restarts break a discharge window
    if (p.type == BP_BOOT || p.type == BP_USB_IN || p.type == BP_FULL || (p.flags & BPF_USB)) {
      if (p.type == BP_USB_IN) { chgSoc = soc; chgMin = 0; }
      else if (p.type == BP_FULL && chgSoc >= 0) {
        chgMin += (p.idleS + p.activeS) / 60.0f;
        const float u[2] = { 100.0f - chgSoc, 1.0f };
        for (uint8_t r = 0; r < 2; r++) { for (uint8_t k = 0; k < 2; k++) C[r][k] += u[r] * u[k]; cb[r] += u[r] * chgMin; }
        charges++; chgSoc = -1;
      } else if (chgSoc >= 0) chgMin += (p.idleS + p.activeS) / 60.0f;
      open = false;
      continue;
    }
    if (p.type == BP_USB_OUT) { open = true; socStart = soc; x0 = x1 = x2 = 0; continue; }
    if (!open) { open = true; socStart = soc; x0 = x1 = x2 = 0; continue; }
    x0 += p.idleS / 3600.0f;  x1 += p.activeS / 3600.0f;  x2 += p.ledMAs / 3600.0f;
    const float drop = socStart - soc;
    if (drop >= BH_WINDOW_PCT) {
      const float y = cfg.batMah * drop / 100.0f;
      const float x[3] = { x0, x1, x2 };
      for (uint8_t r = 0; r < 3; r++) { for (uint8_t k = 0; k < 3; k++) A[r][k] += x[r] * x[k]; b[r] += x[r] * y; }
      rows++; mah += y;
      socStart = soc; x0 = x1 = x2 = 0;
    }
  }
  float res[3];
  if (solve3(A, b, res)) {
    fitIdleMa   = constrain(res[0], 0.005f, 2.0f);
    fitActiveMa = constrain(res[1], 0.05f, 10.0f);
    fitLedK     = constrain(res[2], 0.3f, 3.0f);
  }
  fitRows = rows; fitMah = mah;
  const float det = C[0][0] * C[1][1] - C[0][1] * C[1][0];
  if (fabsf(det) > 1e-6f) {
    fitChgMinPerPct = constrain((cb[0] * C[1][1] - C[0][1] * cb[1]) / det, 0.2f, 10.0f);
    fitChgTailMin   = constrain((C[0][0] * cb[1] - C[1][0] * cb[0]) / det, 0.0f, 120.0f);
  }
  fitCharges = charges;
}

bool fitMeasured() { return fitRows >= 3 && fitMah >= 20; }

// Hours left (remaining capacity down to the cutoff divided by the current)
float hoursWithLight() {
  const float ma = fitActiveMa + fitLedK * percentToMa(cfg.brightMax);
  return cfg.batMah * lastPct / 100.0f / ma;
}
float hoursResting() { return cfg.batMah * lastPct / 100.0f / fitIdleMa; }

// Minutes until full while charging, -1 = unknown (e.g. plugged in before a restart)
float minutesToFull() {
  if (lastCharge != CHG_CHARGING || plugSoc < 0) return -1;
  const float total = fitChgMinPerPct * (100.0f - plugSoc) + fitChgTailMin;
  const float left  = total - (millis() - plugMs) / 60000.0f;
  return left < 1 ? 1 : left;
}

String batJson() {
  const float full = minutesToFull();
  return "{\"t\":\"bat\",\"light\":" + String(hoursWithLight(), 1) + ",\"rest\":" + String(hoursResting(), 0) +
         ",\"full\":" + (full < 0 ? String("null") : String((int)(full + 0.5f))) +
         ",\"src\":\"" + (fitMeasured() ? "measured" : "estimate") + "\"" +
         ",\"rows\":" + String(fitRows) + ",\"charges\":" + String(fitCharges) +
         ",\"idleMa\":" + String(fitIdleMa, 3) + ",\"activeMa\":" + String(fitActiveMa, 2) +
         ",\"ledK\":" + String(fitLedK, 2) + ",\"cap\":" + String(cfg.batMah) + "}";
}

void printBat() {
  if (appMode) { sendLine(batJson()); return; }
  out(String("Runtime (") + (fitMeasured() ? "measured" : "estimate, not enough data yet") + "):");
  out("  with light at " + String((int)cfg.brightMax) + " %: about " + String(hoursWithLight(), 1) + " h");
  out("  resting: about " + String(hoursResting() / 24.0f, 0) + " days");
  const float full = minutesToFull();
  if (full >= 0) out("  full in about " + String((int)(full + 0.5f)) + " min");
  out("Current draw: resting " + String(fitIdleMa, 3) + " mA, awake " + String(fitActiveMa, 2) +
      " mA, LED factor " + String(fitLedK, 2));
  out(String("I2C erratum 89 workaround: ") + (twimWorkaround ? "on (chip revision needs it)" : "off (chip revision not affected)"));
  out("Based on " + String(fitRows) + " discharge windows (" + String(fitMah, 0) + " mAh) and " +
      String(fitCharges) + " charges; " + String(bh.count) + " points stored, capacity " + String(cfg.batMah) + " mAh");
}

void bhReset() {
  memset(&bh, 0, sizeof(bh));
  bh.magic = BH_MAGIC;
  bhSaveFile();
  bhFit();
  say("Battery history cleared.", "{\"t\":\"ack\",\"cmd\":\"batreset\"}");
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
void updateTiltFrom(const float a[3]);   // below
void rangeFit();                         // below (range model)
const char* distSrcText();               // below

// Aiming angle up (+) or down (-) in degrees from one accelerometer reading.
// Forward axis = lateral x level-gravity (both from the cant calibration).
float pitchFrom(const float a[3]) {
  float F[3] = {
    lvl.L[1] * lvl.D[2] - lvl.L[2] * lvl.D[1],
    lvl.L[2] * lvl.D[0] - lvl.L[0] * lvl.D[2],
    lvl.L[0] * lvl.D[1] - lvl.L[1] * lvl.D[0]
  };
  if (!vnorm(F)) return 0;
  return atan2f(vdot(a, F), vdot(a, lvl.D)) * 57.29578f;
}

// Signed cant in degrees from one accelerometer reading (lateral axis from the calibration)
float cantFrom(const float a[3]) {
  return atan2f(vdot(a, lvl.L), vdot(a, lvl.D)) * 57.29578f;
}

// Aiming angle measurement: off / auto (during a session) / on
bool angleActive() {
  if (!lvl.calibrated || !imuOk) return false;
  if (lvl.angleMode == 2) return true;
  if (lvl.angleMode == 0) return sessionActive;
  return false;
}
const char* angleModeText() {
  return lvl.angleMode == 2 ? "on" : (lvl.angleMode == 1 ? "off" : "auto");
}

// Mean over the aiming phase before a shot (buf = pitchBuf or cantBuf); false if too few samples
bool aimingMean(const float* buf, uint32_t shotMs, float& mean, float& sd) {
  float s = 0, q = 0; uint8_t n = 0;
  for (uint8_t k = 0; k < pitchFill; k++) {
    const uint8_t i = (pitchHead + PITCH_SAMPLES - 1 - k) % PITCH_SAMPLES;
    const int32_t before = (int32_t)(shotMs - pitchMs[i]);
    if (before < (int32_t)PITCH_TO_MS || before > (int32_t)PITCH_FROM_MS) continue;
    s += buf[i]; q += buf[i] * buf[i]; n++;
  }
  if (n < 10) return false;
  mean = s / n;
  const float v = q / n - mean * mean;
  sd = v > 0 ? sqrtf(v) : 0;
  return true;
}

// Details of the aim before a shot from the angle history: how long the aim stayed within
// HOLD_TOL_DEG of its mean (holdMs), whether it sank in the last 150..400 ms (drop, deg),
// and the trace itself (newest sample last) for the app.
void aimDetail(uint32_t shotMs, bool hasMean, float mean, uint16_t& holdMs, float& drop, bool& hasDrop, ShotTrace& tr) {
  holdMs = 0; drop = 0; hasDrop = false; tr.n = 0;
  float lateS = 0; uint8_t lateN = 0; float holdS = 0; uint8_t holdN = 0;
  uint32_t lastGood = 0; bool broken = false;
  // newest first
  for (uint8_t k = 0; k < pitchFill; k++) {
    const uint8_t i = (pitchHead + PITCH_SAMPLES - 1 - k) % PITCH_SAMPLES;
    const int32_t before = (int32_t)(shotMs - pitchMs[i]);
    if (before < 0) continue;
    if (before <= (int32_t)TRACE_SPAN_MS && tr.n < TRACE_SAMPLES) {
      const uint8_t j = tr.n++;
      tr.ms[j] = (uint16_t)before;
      tr.pitch[j] = (int16_t)lroundf(pitchBuf[i] * 100); tr.cant[j] = (int16_t)lroundf(cantBuf[i] * 100);
    }
    if (before >= 150 && before < 400) { lateS += pitchBuf[i]; lateN++; }
    if (before >= 400 && before <= (int32_t)PITCH_FROM_MS) { holdS += pitchBuf[i]; holdN++; }
    if (hasMean && before >= (int32_t)PITCH_TO_MS && !broken) {
      if (fabsf(pitchBuf[i] - mean) <= HOLD_TOL_DEG) lastGood = (uint32_t)before; else if (lastGood) broken = true;
    }
  }
  if (lastGood > PITCH_TO_MS) holdMs = (uint16_t)(lastGood - PITCH_TO_MS);
  if (lateN >= 3 && holdN >= 5) { drop = lateS / lateN - holdS / holdN; hasDrop = true; }
  // oldest first for the app
  for (uint8_t a = 0, b = tr.n ? tr.n - 1 : 0; a < b; a++, b--) {
    uint16_t tm = tr.ms[a]; tr.ms[a] = tr.ms[b]; tr.ms[b] = tm;
    int16_t tp = tr.pitch[a]; tr.pitch[a] = tr.pitch[b]; tr.pitch[b] = tp;
    int16_t tc = tr.cant[a]; tr.cant[a] = tr.cant[b]; tr.cant[b] = tc;
  }
}

// One accelerometer reading feeds the cant indicator and the angle history
void updateOrientation(bool tiltOn, bool angleOn) {
  float a[3];
  if (!readAccel(a)) return;
  if (angleOn || tiltOn) {                   // history for the aiming angle and the cant at release
    pitchBuf[pitchHead] = pitchFrom(a);
    cantBuf[pitchHead]  = cantFrom(a);
    pitchMs[pitchHead]  = millis();
    pitchHead = (pitchHead + 1) % PITCH_SAMPLES;
    if (pitchFill < PITCH_SAMPLES) pitchFill++;
  }
  if (tiltOn) updateTiltFrom(a);
}

void updateTilt() {
  float a[3];
  if (!readAccel(a)) return;
  updateTiltFrom(a);
}

void updateTiltFrom(const float a[3]) {
  if (!tiltFilterOk) {
    for (uint8_t i = 0; i < 3; i++) tiltF[i] = a[i];
    tiltFilterOk = true;
  } else {
    float alpha = (float)LEVEL_LOOP_MS / (float)(cfg.tiltSmoothMs ? cfg.tiltSmoothMs : 1);
    if (alpha > 1) alpha = 1;
    for (uint8_t i = 0; i < 3; i++) tiltF[i] += alpha * (a[i] - tiltF[i]);
  }

  const float gL = vdot(tiltF, lvl.L);
  const float gD = vdot(tiltF, lvl.D);
  tiltDeg = fabsf(atan2f(gL, gD)) * 57.29578f;

  if (!tilted && tiltDeg > cfg.levelTol)                      tilted = true;
  else if (tilted && tiltDeg < cfg.levelTol * (1.0f - TILT_HYST_FRAC)) tilted = false;
}

float blinkHz() {
  float span = cfg.tiltFullDeg - cfg.levelTol;
  float f = (span <= 0) ? 1.0f : (tiltDeg - cfg.levelTol) / span;
  if (f < 0) f = 0;
  if (f > 1) f = 1;
  if (lvl.style == 1) f = 1.0f - f;   // inverted: fastest just outside the tolerance
  return cfg.blinkMinHz + f * (cfg.blinkMaxHz - cfg.blinkMinHz);
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

// Kept short (one Bluetooth packet even with a small MTU): only what the app uses
String levelJson() {
  return String("{\"t\":\"level\",\"mode\":\"") + levelModeText() + "\"" +
         ",\"style\":\"" + String(lvl.style == 1 ? "inverted" : "normal") + "\"" +
         ",\"cal\":" + jbool(lvl.calibrated) + ",\"angle\":\"" + angleModeText() + "\"" +
         ",\"cantSignal\":" + jbool(lvl.cantSignal) + ",\"rangeSignal\":" + jbool(lvl.rangeSignal) + "}";
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
  const String txt = "Step 1 ok. Now keep the bow LEVEL but aim clearly UP, arrow tip up "
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
  lvl.calGen = lvl.calGen >= 250 ? 1 : lvl.calGen + 1;   // new zero point for the range model
  calHaveG0 = false;
  tiltReset();
  const bool ok = levelSave();
  rangeFit();
  say(ok ? "Calibration saved." : "ERROR while saving!",
      calJson(2, ok ? "ok" : "error", ok ? "Calibration saved." : "ERROR while saving!"));
  emit(levelJson());
}

void setLevelMode(uint8_t mode, const char* text) {
  lvl.on = mode;
  if (mode == LEVEL_ON && lvl.angleMode == 2) lvl.angleMode = 1;   // cant "on" switches the range "on" off
  if (mode == LEVEL_OFF) tiltReset();
  if (!levelSave()) { err("ERROR while saving!"); return; }
  say(String(text) + (lvl.calibrated || mode == LEVEL_OFF ? "" : " Note: not calibrated yet ('level cal')."),
      levelJson());
}

// "angle off|auto|on": measure the aiming angle at every shot
void handleAngle(const String& arg) {
  if (arg == "off" || arg == "auto" || arg == "on") {
    lvl.angleMode = arg == "off" ? 1 : (arg == "on" ? 2 : 0);
    if (lvl.angleMode == 2 && lvl.on == LEVEL_ON) { lvl.on = LEVEL_OFF; tiltReset(); }   // and the other way round
    if (!levelSave()) { err("ERROR while saving!"); return; }
    say("Aiming angle measurement: " + arg + (lvl.calibrated ? "." : ". Note: needs the cant calibration ('level cal')."),
        levelJson());
  } else if (arg == "") {
    say(String("Aiming angle measurement: ") + angleModeText() + (angleActive() ? ", active now" : ", inactive now"),
        levelJson());
  } else err("Unknown. Possible: angle, angle off|auto|on");
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
  else if (arg == "signal on" || arg == "signal off") {
    lvl.cantSignal = arg == "signal on";
    if (!lvl.cantSignal) tilted = false;
    if (!levelSave()) { err("ERROR while saving!"); return; }
    say(String("Cant signal ") + (lvl.cantSignal ? "on." : "off (still measured and stored)."), levelJson());
  }
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
// Length of the running pattern cycle (0 = steady state)
uint32_t ledCycleMs() {
  switch (ledState) {
    case LS_BLINK: return blinkPeriodMs;
    case LS_WARN:  return warnFlashes * 240UL + 600;
    case LS_PULSE: return 1000;
    default:       return 0;
  }
}
bool ledPatternRunning(uint32_t now) {
  const uint32_t c = ledCycleMs();
  return c && (now - blinkCycleStart) < c;
}

void updateLed(uint32_t now) {
  // After a shot the arrow is gone: pause the tilt indicator for the lockout time
  const bool tiltPause = (int32_t)(now - lastShotMs) < (int32_t)cfg.lockoutMs;

  // The cant indicator works in every LED mode; the mode only decides
  // what the LED does while the bow is level (or the indicator is off).
  // Priority: wrong distance (safety: could shoot over the target) before cant
  LedState st;
  if (!ledPresent)                                st = LS_OFF;
  else if (lowBatLock)                            st = LS_OFF;
  else if (rangeWarn && !tiltPause)               st = LS_WARN;
  else if (rangeOk && !tiltPause)                 st = LS_PULSE;   // setting up a sight: aim is right
  else if (levelActive() && lvl.cantSignal && tilted && !tiltPause) st = LS_BLINK;
  else if (ledMode == MODE_ON)                    st = LS_ON;
  else if (ledMode == MODE_OFF)                   st = LS_OFF;
  else if (isDark)                                st = LS_ON;    // auto: light sensor
  else                                            st = LS_OFF;

  // A started pattern (cant blink, double/triple blink, pulse) always plays its
  // cycle to the end before the LED shows anything else - also a change of the
  // warning's direction waits. Only a shot (arrow gone) or an empty battery cut it off.
  if (!lowBatLock && !tiltPause && ledPatternRunning(now) && st != ledState) st = ledState;

  // Model current for the battery history (blinking is lit half the time)
  const float ma = st == LS_ON    ? percentToMa(brightNow < 0 ? brightTarget() : brightNow)
                 : st == LS_BLINK ? percentToMa(tiltBrightness()) * 0.5f
                 : st == LS_WARN  ? percentToMa(tiltBrightness()) * 0.25f
                 : st == LS_PULSE ? percentToMa(tiltBrightness()) * 0.6f : 0.0f;
  if (ledMaNow == 0 && ma > 0) {
    if ((millis() - unloadedMs) < 10000) bhPoint(BP_LED_ON, unloadedMv, false);   // voltage before the load
  } else if (ledMaNow > 0 && ma == 0) {
    ledOffMs = millis();
    bhPointUnloaded(BP_LED_OFF, false);      // measured after the battery relaxed
  }
  ledMaNow = ma;

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
    case LS_PULSE: {
      // Soft swelling, about once per second (15 % to 100 % of the blink brightness)
      if (ledState != LS_PULSE || (now - blinkCycleStart) >= 1000) blinkCycleStart = now;
      const float ph = (now - blinkCycleStart) / 1000.0f;
      const float lvlF = 0.15f + 0.85f * (0.5f - 0.5f * cosf(ph * 6.2831853f));
      uvWrite((uint16_t)(dutyTilt * lvlF));
      break;
    }
    case LS_WARN: {
      // Aiming too low (arrow short): double blink. Too high (arrow long): triple blink.
      // Flashes of 120 ms with 120 ms gaps, then 600 ms pause.
      // The number of flashes is fixed at the start of each cycle
      if (ledState != LS_WARN || (now - blinkCycleStart) >= warnFlashes * 240UL + 600) {
        blinkCycleStart = now;
        warnFlashes = rangeWarnDir < 0 ? 2 : 3;
      }
      const uint32_t ph = now - blinkCycleStart;
      uvWrite((ph < warnFlashes * 240UL && (ph % 240) < 120) ? dutyTilt : 0);
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
  String j = "{\"t\":\"session\",\"counter\":" + jbool(shotState.shotsOn) +
             ",\"active\":" + jbool(sessionActive);
  if (sessionActive) {
    // epoch + nextId = the key this session will get in the log, so the app
    // can remember its start time even if the board restarts before import
    j += ",\"epoch\":" + String(shotState.epoch) + ",\"nextId\":" + String(sessionId) +
         ",\"dist\":" + String(endDistM) + ",\"distSrc\":\"" + distSrcText() + "\"" +
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
  ensureLogEpoch();                 // the session's log key must be known now
  sessionId = ++shotState.lastId;   // reserved: its ends are stored under this key
  stateSave();
  endShotN = 0;
  endDistM = 0;                     // distance unknown until it is set or recognised
  distSrc = 0;
  sessRowN = 0; sessOffset = 0;
  rangeWarn = false; rangeBadSince = rangeGoodSince = 0;
  sessionActive     = true;
  sessionStartMs    = now;
  sessionLastShotMs = now;
  shotCount = endShots = endCount = 0;
  invalidEnds = invalidArrows = scoreCount = xCount = 0;
  scoreSum = 0;
  mismatchAccepted = false;
  pendingScore = false;
}

// True if an impact at this time would be counted (outside the lockout)
bool shotAllowed(uint32_t now) {
  // Signed compare: an interrupt timestamp can be slightly older than lastShotMs
  return shotState.shotsOn && (int32_t)(now - lastShotMs) >= (int32_t)cfg.lockoutMs;
}

String shotGText() {
  if (lastShotG < 0) return "";
  return lastShotClip ? String(">= 16 g") : String(lastShotG, 1) + " g";
}

void registerShot(uint32_t now, float g, bool clipped, const ShotKin& kin) {
  if (!shotAllowed(now)) return;   // ignore vibration
  lastShotMs   = now;
  lastShotG    = g;
  lastShotClip = clipped;
  if (!sessionActive) {
    startSession(now);
    say("Session started automatically.", sessionJson());
  }
  shotCount++;
  endShots++;
  sessionLastShotMs = now;
  float ang = 0, angSd = 0;
  const bool hasAng = angleActive() && aimingMean(pitchBuf, now, ang, angSd);
  float cant = 0, cantSd = 0;
  const bool hasCant = (angleActive() || levelActive()) && aimingMean(cantBuf, now, cant, cantSd);
  if (endShotN < MAX_SCORES_LINE) {
    endShotAng[endShotN]  = hasAng ? ang : NO_ANGLE;
    endShotCant[endShotN] = hasCant ? cant : NO_ANGLE;
    endShotYaw[endShotN]  = kin.ok ? kin.yaw : NO_ANGLE;
    endShotRoll[endShotN] = kin.ok ? kin.roll : NO_ANGLE;
    endShotRate[endShotN] = kin.ok ? kin.rate : NO_ANGLE;
    endShotMs[endShotN]   = now;
    endShotHold[endShotN] = hasAng ? angSd : NO_ANGLE;
    uint16_t holdMs = 0; float drop = 0; bool hasDrop = false;
    if (angleActive()) aimDetail(now, hasAng, ang, holdMs, drop, hasDrop, endTrace[endShotN]); else endTrace[endShotN].n = 0;
    endShotHoldMs[endShotN] = holdMs;
    endShotDrop[endShotN]   = hasDrop ? drop : NO_ANGLE;
    endShotN++;
  }
  say("Shot detected (end " + String(endCount + 1) + ": " + String(endShots) + ")" +
      (g >= 0 ? ", " + shotGText() : String("")) + (hasAng ? ", angle " + String(ang, 2) + " deg" : String("")) +
      (hasCant ? ", cant " + String(cant, 1) + " deg" : String("")),
      "{\"t\":\"shot\",\"end\":" + String(endCount + 1) + ",\"endShots\":" + String(endShots) +
      ",\"total\":" + String(shotCount) +
      ",\"g\":" + (g >= 0 ? String(g, 1) : String("null")) + ",\"clip\":" + jbool(clipped) +
      ",\"ang\":" + (hasAng ? String(ang, 2) : String("null")) +
      ",\"cant\":" + (hasCant ? String(cant, 1) : String("null")) +
      ",\"yaw\":" + (kin.ok ? String(kin.yaw, 2) : String("null")) +
      ",\"roll\":" + (kin.ok ? String(kin.roll, 2) : String("null")) + "}");
}

// ============================================================================
// Range model: learns which aiming angle belongs to which distance
// ============================================================================
// Solve A x = b (n <= 16) by Gauss elimination with pivoting; false if singular
bool solveN(float A[][16], float* b, float* x, uint8_t n) {
  for (uint8_t c = 0; c < n; c++) {
    uint8_t piv = c;
    for (uint8_t r = c + 1; r < n; r++) if (fabsf(A[r][c]) > fabsf(A[piv][c])) piv = r;
    if (fabsf(A[piv][c]) < 1e-12f) return false;
    for (uint8_t k = 0; k < n; k++) { float t = A[c][k]; A[c][k] = A[piv][k]; A[piv][k] = t; }
    float t = b[c]; b[c] = b[piv]; b[piv] = t;
    for (uint8_t r = 0; r < n; r++) {
      if (r == c) continue;
      const float f = A[r][c] / A[c][c];
      for (uint8_t k = 0; k < n; k++) A[r][k] -= f * A[c][k];
      b[r] -= f * b[c];
    }
  }
  for (uint8_t c = 0; c < n; c++) x[c] = b[c] / A[c][c];
  return true;
}

// Flat trajectory with drag (v = v0 * e^(-k x)): launch angle (rad) = s * A(d, k) + h / d
float trajA(float d, float k) {
  const float u = k * d;
  if (u < 1e-3f) return d * 0.5f + k * d * d / 3.0f;              // series for small drag
  return ((expf(2 * u) - 1) / (4 * k * k) - d / (2 * k)) / d;
}
const float RAD2DEG = 57.29578f;

float setupAngle(uint8_t id, float distM, float offset) {
  return offset + RAD2DEG * (sm[id].s * trajA(distM, sm[id].k) + hShared / distM);
}

// Expected aiming angle for a distance, active setup (includes this session's drift)
float rangeAngleFor(float distM) {
  return setupAngle(setups.active, distM, rngOffset + sessOffset);
}

// Distance for an aiming angle, active setup (the angle rises with the distance)
float rangeDistFor(float th) {
  float lo = 5, hi = 200;
  for (uint8_t i = 0; i < 30; i++) {
    const float mid = (lo + hi) / 2;
    if (rangeAngleFor(mid) < th) lo = mid; else hi = mid;
  }
  return (lo + hi) / 2;
}

// Distances in which recognition and warnings are trusted for the active setup
bool rangeCovers(float distM) {
  const SetupModel& m = sm[setups.active];
  return distM >= (int)m.minD - RANGE_EXTRA_M && distM <= (int)m.maxD + RANGE_EXTRA_M;
}

// ---- fitting ----
static uint8_t fitSetupOf[SETUP_MAX];     // setup id -> column of its s (0xFF = none)
static float   fitK[SETUP_MAX];
static float   fitX[16];
static uint8_t fitG, fitJ;

// Linear least squares for fixed drag values; returns the cost (lower = better)
float rangeSolve() {
  static float A[16][16]; float b[16];
  const uint8_t n = fitG + 1 + fitJ;                          // offsets, h, s per setup
  for (uint8_t i = 0; i < n; i++) { b[i] = 0; for (uint8_t k = 0; k < n; k++) A[i][k] = 0; }
  const float wf = 1.0f / (ANGLE_SD * ANGLE_SD);
  for (uint8_t r = 0; r < rangeRowN; r++) {
    const RangeRow& row = rangeRows[r];
    int8_t gi = -1;
    for (uint8_t i = 0; i < fitG; i++) if (genIds[i] == row.gen) gi = i;
    if (gi < 0 || fitSetupOf[row.setup] == 0xFF) continue;
    float v[16] = {0};
    v[gi] = 1;
    v[fitG] = RAD2DEG / row.d;
    v[fitG + 1 + fitSetupOf[row.setup]] = RAD2DEG * trajA(row.d, fitK[row.setup]);
    const float w = row.w * wf;
    for (uint8_t i = 0; i < n; i++) { b[i] += w * v[i] * row.th; for (uint8_t k = 0; k < n; k++) A[i][k] += w * v[i] * v[k]; }
  }
  for (uint8_t i = 0; i < n; i++) A[i][i] += 1e-6f;
  A[fitG][fitG] += 1.0f / (H_PRIOR_SD * H_PRIOR_SD);          // h near 0 unless the data says otherwise
  if (!solveN(A, b, fitX, n)) return 1e30f;
  float cost = (fitX[fitG] / H_PRIOR_SD) * (fitX[fitG] / H_PRIOR_SD);
  for (uint8_t j = 0; j < SETUP_MAX; j++)
    if (fitSetupOf[j] != 0xFF) cost += ((fitK[j] - K_PRIOR) / K_PRIOR_SD) * ((fitK[j] - K_PRIOR) / K_PRIOR_SD);
  for (uint8_t r = 0; r < rangeRowN; r++) {
    const RangeRow& row = rangeRows[r];
    int8_t gi = -1;
    for (uint8_t i = 0; i < fitG; i++) if (genIds[i] == row.gen) gi = i;
    if (gi < 0 || fitSetupOf[row.setup] == 0xFF) continue;
    const float pred = fitX[gi] + RAD2DEG * (fitX[fitG + 1 + fitSetupOf[row.setup]] * trajA(row.d, fitK[row.setup]) + fitX[fitG] / row.d);
    cost += row.w * wf * (row.th - pred) * (row.th - pred);
  }
  return cost;
}

// Learn from all ends with a distance set by hand
static uint8_t rfD[SETUP_MAX][16], rfDE[SETUP_MAX][16], rfDN[SETUP_MAX];
void rangeFit() {
  rangeRowN = 0; rangeRowNext = 0;
  logForEachKind(0, [](const LogRec& r) {
    const EndRec& e = *(const EndRec*)&r;
    if (e.angleC == ANGLE_NONE || e.angleN < RANGE_MIN_ARROWS || !e.distM) return true;
    if (e.flags & END_DIST_AUTO) return true;            // recognised distances would only confirm themselves
    RangeRow& row = rangeRows[rangeRowNext];
    row.d = e.distM;  row.th = e.angleC / 100.0f;  row.w = e.angleN;
    row.gen   = e.calGen == 0xFF ? 0 : e.calGen;          // before 5.0: the calibration of that time (0)
    row.setup = (e.setup == 0xFF || e.setup >= SETUP_MAX) ? 0 : e.setup;
    rangeRowNext = (rangeRowNext + 1) % RANGE_MAX_ROWS;
    if (rangeRowN < RANGE_MAX_ROWS) rangeRowN++;
    return true;
  }, LOG_ENDS);

  // Calibrations, newest first (the current one always gets a slot)
  genN = 0;
  genIds[genN++] = lvl.calGen;
  for (int16_t k = rangeRowN - 1; k >= 0 && genN < RANGE_MAX_GENS; k--) {
    const uint8_t g = rangeRows[(rangeRowNext + RANGE_MAX_ROWS - rangeRowN + k) % RANGE_MAX_ROWS].gen;
    bool known = false;
    for (uint8_t i = 0; i < genN; i++) if (genIds[i] == g) known = true;
    if (!known) genIds[genN++] = g;
  }
  bool curGenData = false;

  // Per setup: ends, distances (with 2+ ends), range
  for (uint8_t j = 0; j < SETUP_MAX; j++) { sm[j].ends = 0; sm[j].minD = 255; sm[j].maxD = 0; rfDN[j] = 0; }
  for (uint8_t r = 0; r < rangeRowN; r++) {
    const RangeRow& row = rangeRows[r];
    SetupModel& m = sm[row.setup];
    m.ends++;
    if (row.gen == lvl.calGen) curGenData = true;
    const uint8_t dm = (uint8_t)row.d;
    if (dm < m.minD) m.minD = dm;
    if (dm > m.maxD) m.maxD = dm;
    uint8_t di = 0;
    while (di < rfDN[row.setup] && rfD[row.setup][di] != dm) di++;
    if (di == rfDN[row.setup] && di < 16) { rfD[row.setup][di] = dm; rfDE[row.setup][di] = 0; rfDN[row.setup]++; }
    if (di < 16) rfDE[row.setup][di]++;
  }
  uint8_t fullSetups = 0;
  fitJ = 0;
  for (uint8_t j = 0; j < SETUP_MAX; j++) {
    uint8_t dists = 0;
    for (uint8_t i = 0; i < rfDN[j]; i++) if (rfDE[j][i] >= 2) dists++;
    sm[j].dists = dists;
    fitSetupOf[j] = dists >= 1 ? fitJ++ : 0xFF;
    fitK[j] = K_PRIOR;
    if (dists >= 2) fullSetups++;
  }
  fitG = genN;

  if (fitJ && fullSetups) {
    // Drag only where 3+ distances show the curvature; coordinate search, 2 passes
    float best = rangeSolve();
    for (uint8_t pass = 0; pass < 2; pass++) {
      for (uint8_t j = 0; j < SETUP_MAX; j++) {
        if (fitSetupOf[j] == 0xFF || sm[j].dists < 3) continue;
        float bestK = fitK[j];
        for (uint8_t i = 0; i <= 20; i++) {
          fitK[j] = i * 0.0002f;
          const float c = rangeSolve();
          if (c < best) { best = c; bestK = fitK[j]; }
        }
        fitK[j] = bestK;
      }
    }
    rangeSolve();                                             // final values
    for (uint8_t i = 0; i < genN; i++) genOffset[i] = fitX[i];
    hShared = fitX[fitG];
  }

  for (uint8_t j = 0; j < SETUP_MAX; j++) {
    SetupModel& m = sm[j];
    m.state = 0;
    if (fitSetupOf[j] == 0xFF || !fullSetups) continue;
    m.s = fitX[fitG + 1 + fitSetupOf[j]];
    m.k = fitK[j];
    // One distance is enough once another setup fixed the zero point
    const bool learned = m.s > 0 && (m.dists >= 2 || (m.dists >= 1 && fullSetups > (m.dists >= 2 ? 1 : 0)));
    if (!learned) continue;
    m.speed = sqrtf(9.81f / m.s);
    if (m.speed < 20 || m.speed > 200) continue;             // nonsense: keep learning
    m.state = curGenData ? 2 : 1;
  }

  const SetupModel& a = sm[setups.active];
  rngState = a.state; rngSpeed = a.speed; rngEnds = a.ends; rngDists = a.dists;
  rngOffset = genOffset[0];                                    // current calibration

  // Zero drift of the running session: mean residual of its ends set by hand
  sessResSum = sessResW = 0;
  for (uint8_t k = 0; k < sessRowN; k++) {
    if (sm[sessSetup[k]].state != 2) continue;
    sessResSum += sessW[k] * (sessTh[k] - setupAngle(sessSetup[k], sessD[k], rngOffset));
    sessResW   += sessW[k];
  }
  sessOffset = sessResW > 0 ? sessResSum / sessResW : 0;
}

const char* rangeStateText() {
  return rngState == 2 ? "ready" : (rngState == 1 ? "anchor" : "learning");
}
const char* distSrcText() {
  return distSrc == 1 ? "manual" : (distSrc == 2 ? "auto" : "none");
}

String setupName(uint8_t id) {
  if (id >= SETUP_MAX || !setups.s[id].used) return "Setup " + String(id + 1);
  return String(setups.s[id].name);
}

String rangeJson() {
  const SetupModel& m = sm[setups.active];
  String j = "{\"t\":\"range\",\"setup\":" + String(setups.active) + ",\"name\":" + jstr(setupName(setups.active)) +
             ",\"state\":\"" + String(rangeStateText()) + "\",\"ends\":" + String(m.ends) + ",\"dists\":" + String(m.dists);
  if (m.state) {
    j += ",\"min\":" + String(m.minD) + ",\"max\":" + String(m.maxD) +
         ",\"speed\":" + String(m.speed, 1) + ",\"kmh\":" + String((int)lroundf(m.speed * 3.6f)) +
         ",\"drag\":" + String(m.k * 1000, 2);
  }
  j += ",\"dist\":" + String(endDistM) + ",\"distSrc\":\"" + distSrcText() + "\"" +
       (m.state && endDistM && !rangeCovers(endDistM) ? String(",\"estimated\":true") : String("")) +
       ",\"signal\":" + jbool(lvl.rangeSignal) +
       ",\"warn\":\"" + (rangeWarn ? (rangeWarnDir < 0 ? "low" : "high") : "") + "\",\"ok\":" + jbool(rangeOk) + "}";
  return j;
}

void printRange() {
  if (appMode) { sendLine(rangeJson()); return; }
  const SetupModel& m = sm[setups.active];
  out("Setup: " + setupName(setups.active));
  if (m.state == 0) {
    out("Range: learning (" + String(m.ends) + " ends with a distance set by hand, " + String(m.dists) +
        " distances with 2+ ends; needs 2 distances, or 1 once another setup is learned).");
  } else {
    out(String("Range: ") + (m.state == 2 ? "ready" : "needs one end at a known distance (new calibration)") +
        ", learned " + String(m.minD) + "-" + String(m.maxD) + " m, used up to " + String(m.maxD + RANGE_EXTRA_M) + " m");
    out("Arrow speed about " + String((int)lroundf(m.speed * 3.6f)) + " km/h (" + String(m.speed, 1) +
        " m/s), drag " + String(m.k * 1000, 2) + "/km, height " + String(hShared, 2) + " m, drift " + String(sessOffset, 2) + " deg");
    String t = "Angles:";
    for (uint8_t d = 30; d <= 90; d += 10) t += " " + String(d) + "m " + String(rangeAngleFor(d), 2);
    out(t);
  }
  out("Distance now: " + String(endDistM) + " m (" + distSrcText() + "), signal " + (lvl.rangeSignal ? "on" : "off"));
}

// Called every loop pass while aiming: double-blink if the steady aiming angle
// belongs to another distance than the one set (more than half a 10 m step off)
// Called every loop pass while aiming. Training ("auto"): warn when the aim fits
// another 10 m step (more than 5 m off), after about 0.8 s of steady aiming.
// Setting up a sight ("on"): quicker (about 0.45 s) and finer: warn beyond 3 m,
// pulse below 2 m, calm in between. Finer than about +/-1.5 m the model can't tell.
void rangeWarnUpdate(uint32_t now) {
  const bool setupMode = lvl.angleMode == 2;
  const uint8_t  nSamples = setupMode ? 10 : 16;
  const uint32_t holdMs   = setupMode ? 150 : 300;
  const float    warnM    = setupMode ? 3.0f : 5.0f;
  const float    backM    = setupMode ? 2.5f : 4.0f;       // hysteresis
  bool bad = false, good = false;
  float dev = 0;
  if (angleActive() && lvl.rangeSignal && rngState == 2 && endDistM > 0 && rangeCovers(endDistM) && pitchFill >= nSamples) {
    const uint8_t last = (pitchHead + PITCH_SAMPLES - 1) % PITCH_SAMPLES;
    if ((now - pitchMs[last]) < 200) {
      float s = 0, q = 0;
      for (uint8_t k = 0; k < nSamples; k++) {
        const float v = pitchBuf[(pitchHead + PITCH_SAMPLES - 1 - k) % PITCH_SAMPLES];
        s += v; q += v * v;
      }
      const float mean = s / nSamples, var = q / nSamples - mean * mean;
      if (var < 0.04f) {                                       // steady: spread below 0.2 deg
        dev  = rangeDistFor(mean) - endDistM;                  // < 0: aiming for a shorter distance
        bad  = fabsf(dev) > (rangeWarn ? backM : warnM);
        good = setupMode && fabsf(dev) < 2.0f;
      }
    }
  }
  if (bad) {
    rangeGoodSince = 0;
    if (!rangeBadSince) rangeBadSince = now;
    if (!rangeWarn && (now - rangeBadSince) >= holdMs) rangeWarn = true;
    if (rangeWarn) rangeWarnDir = dev < 0 ? -1 : 1;
  } else {
    rangeBadSince = 0;
    if (rangeWarn) {
      if (!rangeGoodSince) rangeGoodSince = now;
      if ((now - rangeGoodSince) >= holdMs) rangeWarn = false;
    }
  }
  // "Right" needs the same short hold, so a bow swinging through doesn't pulse
  if (good && !rangeWarn) {
    if (!rangeOkSince) rangeOkSince = now;
    rangeOk = (now - rangeOkSince) >= holdMs;
  } else {
    rangeOkSince = 0;
    rangeOk = false;
  }
  rangeEventIfChanged();
}

// The app mirrors the LED's range signal (e.g. with the phone's vibration motor):
// one event whenever the warning, its direction or the "aim fits" state changes
bool   rangeEvWarn = false, rangeEvOk = false;
int8_t rangeEvDir  = 0;
void rangeEventIfChanged() {
  if (rangeWarn == rangeEvWarn && rangeOk == rangeEvOk && (!rangeWarn || rangeWarnDir == rangeEvDir)) return;
  rangeEvWarn = rangeWarn; rangeEvOk = rangeOk; rangeEvDir = rangeWarnDir;
  emit(String("{\"t\":\"event\",\"e\":\"range\",\"warn\":\"") + (rangeWarn ? (rangeWarnDir < 0 ? "low" : "high") : "") +
       "\",\"ok\":" + jbool(rangeOk) + "}");
}

// "dist <m>": distance of the running session, set by hand (0 = unknown).
// Does not touch an open score question.
void handleDist(String args) {
  args.trim();
  if (args == "") { say("Distance: " + String(endDistM) + " m (" + distSrcText() + ")", rangeJson()); return; }
  const long d = args.toInt();
  if (d < 0 || d > 250) { err("Distance 0-250 m."); return; }
  endDistM = (uint8_t)d;
  distSrc = d ? 1 : 0;
  rangeWarn = false; rangeBadSince = rangeGoodSince = 0;
  rangeOk = false; rangeOkSince = 0;
  rangeEventIfChanged();
  say("Distance " + String(d) + " m.", "{\"t\":\"ack\",\"cmd\":\"dist\",\"dist\":" + String(d) + "}");
}

// Write the closed end to the log flash (vals: 0-10, 11 = X; NULL for invalid ends)
bool storeEnd(bool valid, bool skipped, const uint8_t* vals, uint8_t n, uint16_t sum, uint8_t xn,
              uint8_t take, float& angOut, float& sdOut, uint8_t& angNOut) {
  EndRec e;
  memset(&e, 0, sizeof(e));
  e.magic = END_MAGIC;
  e.epoch = shotState.epoch;  e.id = sessionId;  e.n = endCount;
  e.arrows = n;  e.flags = (valid ? END_VALID : 0) | (skipped ? END_SKIPPED : 0);
  e.sum = sum;  e.x = xn;
  // Angles of the shots that belong to this end: the first `take` ones
  if (take > endShotN) take = endShotN;
  float as = 0, aq = 0; uint8_t an = 0;
  for (uint8_t k = 0; k < take; k++) {
    if (endShotAng[k] >= NO_ANGLE) continue;
    as += endShotAng[k]; aq += endShotAng[k] * endShotAng[k]; an++;
  }
  // Cant of the same shots
  float cs = 0, cmax = 0; uint8_t cn = 0, canted = 0;
  for (uint8_t k = 0; k < take; k++) {
    const float c = endShotCant[k];
    if (c >= NO_ANGLE) continue;
    cs += c; cn++;
    if (fabsf(c) > cmax) cmax = fabsf(c);
    if (fabsf(c) > cfg.levelTol) canted++;
  }
  e.cantN = cn;  e.cantedN = canted;
  // No distance yet: recognise it from the aiming angle (then it holds for the session)
  if (endDistM == 0 && an >= RANGE_MIN_ARROWS && rngState == 2) {
    const long d = lroundf(rangeDistFor(as / an) / 10.0f) * 10;   // mean aiming angle of this end
    if (d >= 10 && d <= 150 && rangeCovers(d)) {                 // only where the model can be trusted
      endDistM = (uint8_t)d;
      distSrc = 2;
    }
  }
  e.distM  = endDistM;
  e.flags |= distSrc == 1 ? END_DIST_MANUAL : (distSrc == 2 ? END_DIST_AUTO : 0);
  e.calGen = lvl.calGen;
  e.setup  = setups.active;
  e.cantC = cn ? (int16_t)lroundf(cs / cn * 100) : 0;
  e.cantMaxC = (uint16_t)lroundf(cmax * 100);
  lastCant = cn ? cs / cn : 0; lastCantMax = cmax; lastCantN = cn; lastCanted = canted;
  // the shots of this end, kept for "shot list"/"shot teach" of the app
  lastEndNo = endCount; lastEndN = take; lastEndDist = endDistM;
  for (uint8_t k = 0; k < take; k++) {
    lastEndAng[k] = endShotAng[k]; lastEndCant[k] = endShotCant[k];
    lastEndYaw[k] = endShotYaw[k]; lastEndRoll[k] = endShotRoll[k]; lastEndRate[k] = endShotRate[k];
    lastEndMs[k] = endShotMs[k]; lastEndHold[k] = endShotHold[k]; lastEndHoldMs[k] = endShotHoldMs[k]; lastEndDrop[k] = endShotDrop[k];
    lastEndTrace[k] = endTrace[k];
  }
  for (uint8_t k = take; k < endShotN; k++) {                                        // keep the rest
    endShotAng[k - take]  = endShotAng[k];
    endShotCant[k - take] = endShotCant[k];
    endShotYaw[k - take]  = endShotYaw[k];
    endShotRoll[k - take] = endShotRoll[k];
    endShotRate[k - take] = endShotRate[k];
    endShotMs[k - take]   = endShotMs[k];
    endShotHold[k - take] = endShotHold[k];
    endShotHoldMs[k - take] = endShotHoldMs[k];
    endShotDrop[k - take] = endShotDrop[k];
    endTrace[k - take]    = endTrace[k];
  }
  endShotN -= take;
  angNOut = an;
  if (an) {
    angOut = as / an;
    const float v = aq / an - angOut * angOut;
    sdOut = v > 0 ? sqrtf(v) : 0;
    e.angleC = (int16_t)lroundf(angOut * 100);  e.angleSdC = (uint16_t)lroundf(sdOut * 100);  e.angleN = an;
  } else {
    e.angleC = ANGLE_NONE;
  }
  memset(e.scores, 0xFF, sizeof(e.scores));
  for (uint8_t k = 0; vals && k < n && k < 40; k++) {
    const uint8_t v = vals[k] & 0x0F;
    e.scores[k / 2] = (k & 1) ? ((e.scores[k / 2] & 0x0F) | (v << 4)) : ((e.scores[k / 2] & 0xF0) | v);
  }
  memset(e.reserved, 0xFF, sizeof(e.reserved));
  LogRec raw;
  memcpy(&raw, &e, sizeof(raw));
  const bool ok = logAppend(raw, false);
  // An end at a distance set by hand teaches the range model
  if (ok && distSrc == 1 && an >= RANGE_MIN_ARROWS && endDistM) {
    if (sessRowN < MAX_SCORES_LINE) {
      sessD[sessRowN] = endDistM; sessTh[sessRowN] = as / an; sessW[sessRowN] = an; sessSetup[sessRowN] = setups.active; sessRowN++;
    }
    rangeFit();
    emit(rangeJson());
  }
  return ok;
}

String endCantJson() {
  return ",\"cant\":" + (lastCantN ? String(lastCant, 1) : String("null")) +
         ",\"cantMax\":" + (lastCantN ? String(lastCantMax, 1) : String("null")) +
         ",\"canted\":" + String(lastCanted) + ",\"cantN\":" + String(lastCantN);
}

String endAngleJson(float ang, float sd, uint8_t angN) {
  return ",\"dist\":" + (endDistM ? String(endDistM) : String("null")) + ",\"distSrc\":\"" + distSrcText() + "\"" +
         ",\"setup\":" + String(setups.active) +
         ",\"ang\":" + (angN ? String(ang, 2) : String("null")) +
         ",\"angSd\":" + (angN ? String(sd, 2) : String("null")) + ",\"angN\":" + String(angN);
}

// Close an end with scores. keepShots: counted shots that stay in the open end
// (split: the scores were for the first n shots only)
void closeEndValid(const uint8_t* vals, uint8_t n, uint8_t xn, uint8_t keepShots) {
  uint16_t sum = 0;
  for (uint8_t k = 0; k < n; k++) sum += vals[k] > 10 ? 10 : vals[k];   // 11 = X counts 10
  scoreSum   += sum;
  scoreCount += n;
  xCount     += xn;
  endCount++;
  const uint8_t take = endShotN > keepShots ? endShotN - keepShots : 0;
  endShots = keepShots;
  float ang = 0, sd = 0; uint8_t angN = 0;
  const bool stored = storeEnd(true, false, vals, n, sum, xn, take, ang, sd, angN);
  say("End " + String(endCount) + " scored: " + String(n) + " arrows, " + String(sum) +
      " points (avg " + fmtAvg(sum, n) + ", " + String(xn) + " X). Session: " + String(scoreCount) +
      " scored, avg " + fmtAvg(scoreSum, scoreCount) + ", " + String(xCount) + " X",
      "{\"t\":\"end\",\"n\":" + String(endCount) + ",\"valid\":true,\"arrows\":" + String(n) +
      ",\"sum\":" + String(sum) + ",\"x\":" + String(xn) + ",\"avg\":" + fmtAvg(sum, n) +
      endAngleJson(ang, sd, angN) + endCantJson() + ",\"stored\":" + jbool(stored) + "}");
  emit(sessionJson());
  if (appMode) sendShots(lastEndNo, lastEndAng, lastEndCant, lastEndYaw, lastEndRoll, lastEndRate, lastEndN, lastEndDist, lastEndMs, lastEndHold, lastEndHoldMs, lastEndDrop);
}

// Close an end as invalid: all arrows 0 points, not in the overall average
void closeEndInvalid(uint16_t arrows, const String& reason) {
  endCount++;
  invalidEnds++;
  invalidArrows += arrows;
  endShots = 0;
  float ang = 0, sd = 0; uint8_t angN = 0;
  const bool stored = storeEnd(false, reason == "skipped", NULL, arrows, 0, 0, endShotN, ang, sd, angN);
  say("End " + String(endCount) + " invalid (" + reason + "): " + String(arrows) +
      " arrows stored with 0 points, not included in the overall average.",
      "{\"t\":\"end\",\"n\":" + String(endCount) + ",\"valid\":false,\"arrows\":" + String(arrows) +
      ",\"reason\":" + jstr(reason) + endAngleJson(ang, sd, angN) + endCantJson() + ",\"stored\":" + jbool(stored) + "}");
  emit(sessionJson());
}

String slotJson(const LogRec& s) {
  String ago = "null";
  for (uint8_t k = 0; k < RECENT_SAVES; k++) {
    if (recentSeq[k] == s.seq && s.seq != 0) ago = String((millis() - recentMs[k]) / 60000UL);
  }
  return "{\"t\":\"slot\",\"seq\":" + String(s.seq) + ",\"epoch\":" + String(s.epoch) +
         ",\"id\":" + String(s.id) + ",\"start\":" + (s.start ? String(s.start) : String("null")) +
         ",\"ago\":" + ago + ",\"ends\":" + String(s.ends) +
         ",\"shots\":" + String(s.shots) + ",\"scored\":" + String(s.scores) +
         ",\"avg\":" + String(s.avgX100 / 100.0f, 2) + ",\"x\":" + String(s.xCount) +
         ",\"min\":" + String(s.minutes) + ",\"invalidEnds\":" + String(s.invalidEnds) +
         ",\"invalidArrows\":" + String(s.invalidArrows) +
         ",\"mismatch\":" + jbool(s.flags & SLOT_MISMATCH) +
         ",\"manual\":" + jbool(s.flags & SLOT_MANUAL) + "}";
}

String slotLine(const LogRec& s) {
  String line = "#" + String(s.seq) + " " + fmtUnix(s.start) + ": " + String(s.ends) + " ends, " +
                String(s.shots) + " shots, " + String(s.scores) + " scored, avg " +
                String(s.avgX100 / 100.0f, 2) + ", " + String(s.xCount) + " X, " + String(s.minutes) + " min";
  if (s.invalidEnds > 0) {
    line += " | invalid: " + String(s.invalidEnds) + " ends, " + String(s.invalidArrows) + " arrows";
  }
  if (s.flags & SLOT_MISMATCH) line += "  [!]";
  return line;
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
  // The distance belongs to the session: outside one (aiming angle "on", setting up
  // a sight) no old value may be used, it has to be set anew with "dist"
  endDistM = 0; distSrc = 0;
  rangeWarn = false; rangeBadSince = rangeGoodSince = 0;
  rangeOk = false; rangeOkSince = 0;
  rangeEventIfChanged();
  emit(rangeJson());

  if (endCount == 0) {
    say("Empty session discarded (nothing stored).",
        "{\"t\":\"sessionEnd\",\"manual\":" + jbool(manual) + ",\"stored\":false}");
    return;
  }

  LogRec s;
  memset(&s, 0, sizeof(s));
  s.shots         = shotCount;
  s.scores        = scoreCount;
  s.avgX100       = scoreCount ? (uint16_t)((scoreSum * 100 + scoreCount / 2) / scoreCount) : 0;
  s.minutes       = (uint16_t)((now - sessionStartMs + 30000UL) / 60000UL);
  s.ends          = endCount;
  s.invalidEnds   = invalidEnds;
  s.invalidArrows = invalidArrows;
  s.xCount        = xCount;
  s.flags         = manual ? SLOT_MANUAL : 0;
  if (invalidEnds > 0 || mismatchAccepted) s.flags |= SLOT_MISMATCH;

  ensureLogEpoch();
  s.epoch = shotState.epoch;
  s.id    = sessionId;
  s.start = unixAt(sessionStartMs);
  stateSave();                         // session number must never repeat
  const bool ok = logAppend(s, true);
  if (!ok) s.seq = 0;                  // the app must not take a number that was never written

  say(String(manual ? "Session ended" : "Session ended automatically (" + String(cfg.sessionEndMin) + " min without a shot)") +
      ": " + String(s.ends) + " ends (" + String(s.invalidEnds) + " invalid), " +
      String(s.scores) + " arrows scored, avg " + String(s.avgX100 / 100.0f, 2) +
      ", " + String(s.xCount) + " X, " + String(s.minutes) + " min" +
      (ok ? "" : "  ERROR: could not write to the log flash (" + logError + ")!"),
      "{\"t\":\"sessionEnd\",\"manual\":" + jbool(manual) + ",\"stored\":" + jbool(ok) +
      ",\"epoch\":" + String(s.epoch) + ",\"id\":" + String(s.id) + ",\"seq\":" + String(ok ? s.seq : 0) +
      (ok ? String("") : ",\"error\":" + jstr(logError)) + "}");
  emit(slotJson(s));                   // details on their own line (keeps lines short)
}

String sessionLine() {
  if (!sessionActive) return "No session active.";
  const uint32_t mins = (millis() - sessionStartMs) / 60000UL;
  return "Session running: end " + String(endCount + 1) + " open (" + String(endShots) +
         " shots), " + String(endCount) + " ends recorded (" + String(invalidEnds) +
         " invalid), " + String(scoreCount) + " arrows scored, avg " +
         fmtAvg(scoreSum, scoreCount) + ", " + String(xCount) + " X, " + String(mins) + " min";
}

String logInfoJson() {
  char jed[8];
  snprintf(jed, sizeof(jed), "%06lX", (unsigned long)qfJedec);
  return "{\"t\":\"loginfo\",\"ok\":" + jbool(qfOk) + ",\"jedec\":\"" + String(jed) + "\"" +
         ",\"count\":" + String(logVisibleCount()) + ",\"capacity\":" + String((uint32_t)(QF_SECTORS - 1) * RECS_PER_SECTOR) +
         ",\"last\":" + String(logLastSeq) + ",\"clock\":" + jbool(clockValid) +
         ",\"migrated\":" + String(migratedCount) + ",\"error\":" + jstr(logError) +
         ",\"initErr\":" + String(qspiInitErr) + ",\"resets\":" + String(qspiRecoveries) + "}";
}

String scoresText(const EndRec& e) {
  String t;
  for (uint8_t k = 0; k < e.arrows && k < 40; k++) {
    const uint8_t v = (k & 1) ? (e.scores[k / 2] >> 4) : (e.scores[k / 2] & 0x0F);
    if (v == 15) break;
    if (t.length()) t += ' ';
    t += v == 11 ? String("X") : String(v);
  }
  return t;
}

// Only the fields that carry information, so long ends still fit one Bluetooth packet
String endJson(const EndRec& e) {
  String j = "{\"t\":\"endrec\",\"seq\":" + String(e.seq) + ",\"epoch\":" + String(e.epoch) + ",\"id\":" + String(e.id) +
             ",\"n\":" + String(e.n) + ",\"valid\":" + jbool(e.flags & END_VALID) +
             ",\"arrows\":" + String(e.arrows);
  if (e.flags & END_SKIPPED) j += ",\"skipped\":true";
  if (e.flags & END_VALID)   j += ",\"sum\":" + String(e.sum) + ",\"x\":" + String(e.x);
  if (e.distM)               j += ",\"dist\":" + String(e.distM) + ((e.flags & END_DIST_AUTO) ? ",\"auto\":true" : "");
  if (e.setup != 0xFF && e.setup != 0) j += ",\"setup\":" + String(e.setup);
  if (e.angleC != ANGLE_NONE)
    j += ",\"ang\":" + String(e.angleC / 100.0f, 2) + ",\"angSd\":" + String(e.angleSdC / 100.0f, 2) + ",\"angN\":" + String(e.angleN);
  if (e.cantN != 0xFF && e.cantN)
    j += ",\"cant\":" + String(e.cantC / 100.0f, 1) + ",\"cantMax\":" + String(e.cantMaxC / 100.0f, 1) +
         ",\"canted\":" + String(e.cantedN) + ",\"cantN\":" + String(e.cantN);
  if (e.flags & END_VALID)   j += ",\"scores\":\"" + scoresText(e) + "\"";
  return j + "}";
}

String endLine(const EndRec& e) {
  String l = "  End " + String(e.n) + ": ";
  l += (e.flags & END_VALID) ? String(e.sum) + " (" + scoresText(e) + ")" : String("invalid, ") + String(e.arrows) + " arrows";
  if (e.distM) l += ", " + String(e.distM) + " m";
  if (e.angleC != ANGLE_NONE) l += ", angle " + String(e.angleC / 100.0f, 2) + " +/- " + String(e.angleSdC / 100.0f, 2) + " deg";
  if (e.cantN != 0xFF && e.cantN) l += ", cant " + String(e.cantC / 100.0f, 1) + " deg (max " + String(e.cantMaxC / 100.0f, 1) +
                                        ", " + String(e.cantedN) + " canted)";
  return l;
}

// Terminal: "log ends <epoch> <id>"
static uint32_t lsEpoch, lsId;
void printEnds(String args) {
  args.trim();
  const int sp = args.indexOf(' ');
  lsEpoch = (uint32_t)strtoul((sp < 0 ? args : args.substring(0, sp)).c_str(), nullptr, 10);
  lsId    = sp < 0 ? 0 : (uint32_t)strtoul(args.substring(sp + 1).c_str(), nullptr, 10);
  if (!lsEpoch || !lsId) { err("Format: log ends <epoch> <id>"); return; }
  out("--- Ends of session " + String(lsEpoch) + "-" + String(lsId) + " ---");
  logForEachKind(0, [](const LogRec& r) {
    const EndRec& e = *(const EndRec*)&r;
    if (e.epoch == lsEpoch && e.id == lsId) out(endLine(e));
    return true;
  }, LOG_ENDS);
}

void printLogInfo() {
  if (appMode) { sendLine(logInfoJson()); return; }
  char jed[8];
  snprintf(jed, sizeof(jed), "%06lX", (unsigned long)qfJedec);
  char sr[24];
  snprintf(sr, sizeof(sr), "SR1 %02X, SR2 %02X", qfStatus1, qfStatus2);
  out(String("Log flash: ") + (qfOk ? "OK" : "NOT AVAILABLE") + " (JEDEC " + jed + ", " + sr +
      (qfUnprotected ? ", write protection cleared" : "") + ")");
  if (logError.length()) out("Last flash error: " + logError);
  if (qspiInitErr || qspiRecoveries) {
    char buf[80];
    snprintf(buf, sizeof(buf), "QSPI start: last error 0x%08lX, chip reset by hand %u times, JEDEC by hand %06lX",
             (unsigned long)qspiInitErr, qspiRecoveries, (unsigned long)qfBbJedec);
    out(buf);
  }
  out("Sessions stored: " + String(logVisibleCount()) + " of about " +
      String((uint32_t)(QF_SECTORS - 1) * RECS_PER_SECTOR) + ", newest #" + String(logLastSeq));
  out(String("Clock: ") + (clockValid ? fmtUnix(unixAt(millis())) : "not set (connect the app)"));
  if (migratedCount) out("Taken over from the old log at this start: " + String(migratedCount) + " sessions");
}

// App: every record newer than "since". Terminal: the newest LOG_PRINT_LAST, or all.
static uint32_t logSentN;   // records in the current answer, so the app can tell a lost line
void printLog(uint32_t since, bool all) {
  if (appMode) {
    sendLine("{\"t\":\"logStart\",\"since\":" + String(since) + ",\"last\":" + String(logLastSeq) +
             ",\"total\":" + String(logVisibleCount()) + ",\"epoch\":" + String(shotState.epoch) + "}");
    logSentN = 0;
    logForEachKind(since, [](const LogRec& r) {
      sendLine(r.magic == END_MAGIC ? endJson(*(const EndRec*)&r) : slotJson(r));
      logSentN++;
      return true;
    }, LOG_SESSIONS | LOG_ENDS);
    sendLine("{\"t\":\"logEnd\",\"last\":" + String(logLastSeq) + ",\"n\":" + String(logSentN) + "}");
    return;
  }
  if (!qfOk) { err("Log flash not available."); return; }
  if (logVisibleCount() == 0) { out("No sessions stored."); return; }
  if (all) {
    out("--- Sessions (oldest first) ---");
    logForEach(since, [](const LogRec& r) { out(slotLine(r)); return true; });
    return;
  }
  // newest LOG_PRINT_LAST, newest first
  static LogRec buf[LOG_PRINT_LAST];
  static uint8_t n;
  n = 0;
  const uint32_t from = logLastSeq > LOG_PRINT_LAST ? logLastSeq - LOG_PRINT_LAST : 0;
  logForEach(from, [](const LogRec& r) { if (n < LOG_PRINT_LAST) buf[n++] = r; return true; });
  out("--- Newest " + String(n) + " of " + String(logVisibleCount()) + " sessions ('log all' for all) ---");
  for (int8_t i = n - 1; i >= 0; i--) out(slotLine(buf[i]));
}

void handleShots(const String& arg) {
  if (arg == "") {
    if (appMode) { sendLine(sessionJson()); return; }
    out(String("Shot counter: ") + (shotState.shotsOn ? "on" : "off"));
    out(sessionLine());
  }
  else if (arg == "on") {
    shotState.shotsOn = 1;
    if (!stateSave()) { err("ERROR while saving!"); return; }
    say("Shot counter on.", sessionJson());
  }
  else if (arg == "off") {
    if (sessionActive) { err("End the running session first ('shots stop')."); return; }
    shotState.shotsOn = 0;
    if (!stateSave()) { err("ERROR while saving!"); return; }
    say("Shot counter off.", sessionJson());
  }
  else if (arg == "start") {
    if (!shotState.shotsOn) { err("Shot counter is off ('shots on')."); return; }
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

  // Distance "@NN" anywhere in the line (sticky for the following ends)
  const int at = args.indexOf('@');
  if (at >= 0) {
    int e = args.indexOf(' ', at);
    const long d = args.substring(at + 1, e < 0 ? args.length() : e).toInt();
    if (d < 0 || d > 250) { err("Invalid distance (0-250 m)."); return; }
    endDistM = (uint8_t)d;
    distSrc = d ? 1 : 0;
    args = args.substring(0, at) + (e < 0 ? String("") : args.substring(e));
    args.trim();
  }

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
    vals[n++] = isX ? 11 : (uint8_t)v;      // 11 = X (10 points, counted separately)
    if (isX) xn++;
  }

  if (n == 0) { err("Format: score 9 7 x 0  or  score skip"); return; }

  // Compare with the shots of this end
  if (n != endShots) {
    pendingScore = true;
    pendingN = n;
    pendingX = xn;
    memcpy(pendingVals, vals, n);
    const bool canSplit = n < endShots;
    say("End " + String(endCount + 1) + ": " + String(endShots) + " shots counted, " +
        String(n) + " values entered - save anyway? (yes/no" + (canSplit ? String("/split") : String("")) + ")" +
        (canSplit ? String(" split = save these as one end, keep the other ") + String(endShots - n) +
                    " shots for the next end" : String("")),
        "{\"t\":\"confirm\",\"end\":" + String(endCount + 1) + ",\"counted\":" + String(endShots) +
        ",\"entered\":" + String(n) + ",\"split\":" + jbool(canSplit) + "}");
    return;
  }
  closeEndValid(vals, n, xn, 0);
}

// ============================================================================
// Status and commands
// ============================================================================
const char* ledStateText() {
  switch (ledState) {
    case LS_ON:    return "on";
    case LS_BLINK: return "blinking";
    case LS_WARN:  return "warning";
    case LS_PULSE: return "pulsing";
    default:       return "off";
  }
}

String statusLine() {
  String s = ledPresent ? "Light=" + String(lastLdr) + (isDark ? " (dark)" : " (bright)") : String("No UV LED fitted");
  s += " | Battery=" + String(lastVbat, 2) + "V " + String((int)(lastPct + 0.5f)) + "%";
  s += " (" + String(chargeText(lastCharge)) + ")";
  s += " | LED=" + String(ledStateText());
  s += " Bright=" + String((int)(brightNow < 0 ? brightTarget() : brightNow)) + "%";
  if (levelActive() && tiltFilterOk) s += " | Tilt=" + String(tiltDeg, 1) + "deg";
  if (ledMode == MODE_ON)  s += " | Mode=on";
  if (ledMode == MODE_OFF) s += " | Mode=off";
  if (lowBatLock)          s += " | BATTERY EMPTY";
  if (sessionActive)       s += " | End " + String(endCount + 1) + ": " + String(endShots) + " shots";
  if (lastShotG >= 0)      s += " | Last shot " + shotGText();
  return s;
}

String statusJson() {
  const char* led  = ledStateText();
  const char* mode = (ledMode == MODE_ON) ? "on" : (ledMode == MODE_OFF ? "off" : "auto");
  String j = "{\"t\":\"status\",\"light\":" + String(lastLdr) + ",\"dark\":" + jbool(isDark) +
             ",\"vbat\":" + String(lastVbat, 2) + ",\"pct\":" + String((int)(lastPct + 0.5f)) +
             ",\"chg\":\"" + chargeText(lastCharge) + "\",\"led\":\"" + led + "\"" +
             ",\"duty\":" + String(dutyFull) + ",\"bright\":" + String((int)(brightNow < 0 ? brightTarget() : brightNow)) +
             ",\"mode\":\"" + mode + "\"" +
             ",\"lowbat\":" + jbool(lowBatLock) +
             ",\"tilt\":" + ((levelActive() && tiltFilterOk) ? String(tiltDeg, 1) : String("null")) +
             ",\"session\":" + jbool(sessionActive) + ",\"awake\":\"" + awakeReason + "\"" +
             ",\"rwarn\":" + jbool(rangeWarn) +
             ",\"lastShotG\":" + (lastShotG >= 0 ? String(lastShotG, 1) : String("null")) +
             ",\"lastShotClip\":" + jbool(lastShotClip);
  if (sessionActive) j += ",\"end\":" + String(endCount + 1) + ",\"endShots\":" + String(endShots);
  return j + "}";
}

// Every status sent restarts the report timer, so an app that asks for the status
// itself does not get the periodic report on top of it
void sendStatus() {
  say(statusLine(), statusJson());
  emit(batJson());
  lastReportMs = millis();
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
  out("mode auto|on|off  LED: light sensor / always on / off (cant blinking works in all)");
  out("shots             counter status + running session");
  out("shots on|off      shot counter on/off");
  out("shots start|stop  start/end a session manually");
  out("score 9 x 7 0     score an end (0-10, x = inner ten); add @50 for the distance");
  out("score skip        end without scores (invalid)");
  out("log               newest sessions (log all: all)");
  out("log since <n>     sessions after #n");
  out("log info          log flash, number of sessions, clock");
  out("log put ...       write a session from a backup (used by the app)");
  out("log test          check the log flash step by step");
  out("log ends <e> <id> ends of one session (scores, distance, angle, cant)");
  out("angle off|auto|on measure the aiming angle at every shot");
  out("dist <m>          distance of the session (0 = unknown); keeps an open question");
  out("range             learned distances, arrow speed; range signal on|off");
  out("setup [use|new|name|del] arrow/bow setups (each learns its own speed)");
  out("name [text|-]      name this sight (shown in the app, added to the BLE name); - removes it");
  out("shot list|trace|model|reset|on|off  shots of the open end with predictions, aim trace of one shot, model state, reset, switch");
  out("level signal on|off  blink when canted (measuring continues)");
  out("log del <e> <id>  delete one session on the sight");
  out("log clear         delete all sessions on the sight (asks first)");
  out("level             tilt indicator status");
  out("level on|auto|off tilt indicator always / in sessions / off");
  out("level style normal|inverted  blink faster when tilted / when level");
  out("level cal         calibration step 1");
  out("level cal2        calibration step 2");
  out("app on|off        JSON output for the app");
  out("awake <s>         stay reachable without movement (max 900 s, 0 = off)");
  out("bat               runtime estimate and measured current draw");
  out("bat reset         clear the battery history");
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
  applyImuThresholds();          // apply IMU thresholds right away
  if (key == "tx_power") applyTxPower();
  if (key == "bat_mah")  bhFit();   // the runtime estimate scales with the capacity

  const String shown = fmtSetting(*d, getSetting(cfg, *d));
  say(key + " = " + shown + "  (active, permanent with 'save')",
      "{\"t\":\"ack\",\"cmd\":\"set\",\"k\":\"" + key + "\",\"v\":" + shown + "}");
  if (cfg.darkOn >= cfg.darkOff) err("Warning: dark_on should be lower than dark_off.");
  if (cfg.tiltFullDeg <= cfg.levelTol) err("Warning: tilt_full should be higher than level_tol.");
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

// "log put <epoch> <id> <start> <ends> <shots> <scored> <avgX100> <x> <min>
//          <invalidEnds> <invalidArrows> <flags>"
// Writes a session from a backup into the log, unless its key is already there.
void logPut(String args) {
  uint32_t v[12];
  uint8_t n = 0;
  args.trim();
  while (args.length() && n < 12) {
    int sp = args.indexOf(' ');
    String tok = sp < 0 ? args : args.substring(0, sp);
    v[n++] = (uint32_t)strtoul(tok.c_str(), nullptr, 10);
    args = sp < 0 ? String("") : args.substring(sp + 1);
    args.trim();
  }
  const String key = "{\"t\":\"ack\",\"cmd\":\"put\",\"epoch\":" + String(n > 0 ? v[0] : 0) +
                     ",\"id\":" + String(n > 1 ? v[1] : 0) + ",\"result\":";
  if (n != 12 || v[0] == 0 || v[1] == 0) {
    say("Format: log put <epoch> <id> <start> <ends> <shots> <scored> <avgX100> <x> <min> "
        "<invalidEnds> <invalidArrows> <flags>", key + "\"error\"}");
    return;
  }
  if (!qfOk) { say("Log flash not available.", key + "\"error\"}"); return; }
  // Copying from the app: stay awake while sessions keep coming
  if ((int32_t)(awakeUntil - (millis() + AWAKE_PUT_MS)) < 0) awakeUntil = millis() + AWAKE_PUT_MS;
  if (logHasKey(v[0], v[1])) {
    say("Session " + String(v[0]) + "-" + String(v[1]) + " is already stored.", key + "\"exists\"}");
    return;
  }
  LogRec r;
  memset(&r, 0, sizeof(r));
  r.epoch = v[0];  r.id = v[1];  r.start = v[2];
  r.ends = v[3];   r.shots = v[4];  r.scores = v[5];  r.avgX100 = v[6];  r.xCount = v[7];
  r.minutes = v[8];  r.invalidEnds = v[9];  r.invalidArrows = v[10];
  r.flags = v[11] & (SLOT_MISMATCH | SLOT_MANUAL);
  const bool ok = logAppend(r, false);
  // Never hand out a session number twice in the current log
  if (ok && r.epoch == shotState.epoch && r.id > shotState.lastId) { shotState.lastId = r.id; stateSave(); }
  say(ok ? "Session " + String(v[0]) + "-" + String(v[1]) + " added as #" + String(r.seq) + "."
         : String("ERROR: could not write to the log flash!"),
      key + (ok ? "\"added\",\"seq\":" + String(r.seq) + "}" : String("\"error\"}")));
}

// Is this end already stored?
static uint32_t feEpoch, feId; static uint16_t feN; static bool feHit;
bool endHasKey(uint32_t epoch, uint32_t id, uint16_t n) {
  feEpoch = epoch; feId = id; feN = n; feHit = false;
  logForEachKind(0, [](const LogRec& r) {
    const EndRec& e = *(const EndRec*)&r;
    if (e.epoch == feEpoch && e.id == feId && e.n == feN) { feHit = true; return false; }
    return true;
  }, LOG_ENDS);
  return feHit;
}

// "log putend <epoch> <id> <n> <arrows> <flags> <sum> <x> <dist> <angleC> <angleSdC> <angleN>
//              <cantC> <cantMaxC> <cantN> <cantedN> <scores>"
// scores: one hex digit per arrow (0-9, A = 10, B = X), "-" for none
void logPutEnd(String args) {
  // 15 numbers [+ setup] + scores (16 or 17 tokens)
  String tok[17]; uint8_t n = 0;
  args.trim();
  while (args.length() && n < 17) {
    const int sp = args.indexOf(' ');
    tok[n++] = sp < 0 ? args : args.substring(0, sp);
    args = sp < 0 ? String("") : args.substring(sp + 1);
    args.trim();
  }
  uint32_t v[16] = {0};
  for (uint8_t i = 0; i + 1 < n && i < 16; i++) v[i] = (uint32_t)strtol(tok[i].c_str(), nullptr, 10);
  const String scores = n ? tok[n - 1] : String("-");
  const uint8_t putSetup = n == 17 ? (uint8_t)v[15] : 0;
  if (n == 17) n = 16;
  const String key = "{\"t\":\"ack\",\"cmd\":\"putend\",\"epoch\":" + String(n > 0 ? v[0] : 0) +
                     ",\"id\":" + String(n > 1 ? v[1] : 0) + ",\"n\":" + String(n > 2 ? v[2] : 0) + ",\"result\":";
  if (n != 16 || !v[0] || !v[1] || !v[2]) { say("Format: log putend <epoch> <id> <n> ... <scores>", key + "\"error\"}"); return; }
  if (!qfOk) { say("Log flash not available.", key + "\"error\"}"); return; }
  if ((int32_t)(awakeUntil - (millis() + AWAKE_PUT_MS)) < 0) awakeUntil = millis() + AWAKE_PUT_MS;
  if (endHasKey(v[0], v[1], v[2])) { say("End already stored.", key + "\"exists\"}"); return; }
  EndRec e;
  memset(&e, 0, sizeof(e));
  e.magic = END_MAGIC;
  e.epoch = v[0]; e.id = v[1]; e.n = v[2]; e.arrows = v[3]; e.flags = v[4] & (END_VALID | END_SKIPPED);
  e.sum = v[5]; e.x = v[6]; e.distM = v[7];
  e.flags = v[4] & (END_VALID | END_SKIPPED | END_DIST_MANUAL | END_DIST_AUTO);
  e.angleC = (int16_t)(int32_t)v[8]; e.angleSdC = v[9]; e.angleN = v[10];
  e.cantC = (int16_t)(int32_t)v[11]; e.cantMaxC = v[12]; e.cantN = v[13]; e.cantedN = v[14];
  e.setup = putSetup < SETUP_MAX ? putSetup : 0;
  e.calGen = 0xFF;                               // measured with an unknown calibration
  memset(e.scores, 0xFF, sizeof(e.scores));
  for (uint8_t k = 0; scores != "-" && k < scores.length() && k < 40; k++) {
    const char c = scores[k];
    const uint8_t d = (c >= '0' && c <= '9') ? c - '0' : ((c == 'a' || c == 'A') ? 10 : ((c == 'b' || c == 'B') ? 11 : 15));
    e.scores[k / 2] = (k & 1) ? ((e.scores[k / 2] & 0x0F) | (d << 4)) : ((e.scores[k / 2] & 0xF0) | d);
  }
  memset(e.reserved, 0xFF, sizeof(e.reserved));
  LogRec raw;
  memcpy(&raw, &e, sizeof(raw));
  const bool ok = logAppend(raw, false);
  say(ok ? String("End added.") : String("ERROR: could not write to the log flash!"),
      key + (ok ? "\"added\"}" : "\"error\"}"));
}

// ============================================================================
// Setups (arrows / bow settings), each with its own learned speed and drag
// ============================================================================
bool setupsSave() {
  InternalFS.remove(SETUP_FILE);
  File f(InternalFS);
  if (!f.open(SETUP_FILE, FILE_O_WRITE)) return false;
  f.write((const uint8_t*)&setups, sizeof(setups));
  f.close();
  return true;
}

void setupsLoad() {
  memset(&setups, 0, sizeof(setups));
  File f(InternalFS);
  if (f.open(SETUP_FILE, FILE_O_READ)) {
    SetupTable tmp;
    if (f.read(&tmp, sizeof(tmp)) == (int)sizeof(tmp) && tmp.magic == SETUP_MAGIC) setups = tmp;
    f.close();
  }
  if (setups.magic != SETUP_MAGIC || setups.active >= SETUP_MAX || setups.s[setups.active].used != 1) {
    memset(&setups, 0, sizeof(setups));        // the first setup is always there
    setups.magic = SETUP_MAGIC;
    setups.active = 0;
    setups.s[0].used = 1;
    strncpy(setups.s[0].name, "Standard", sizeof(setups.s[0].name) - 1);
    setupsSave();
  }
}

// Printable name, at most 18 bytes, never cut inside a UTF-8 character
void setName(uint8_t id, String name) {
  name.trim();
  String clean;
  for (unsigned int i = 0; i < name.length(); i++) {
    const char c = name[i];
    if ((uint8_t)c < 0x20 || c == '"' || c == '\\') continue;
    clean += c;
  }
  uint8_t len = clean.length() > 18 ? 18 : clean.length();
  while (len > 0 && len < clean.length() && ((uint8_t)clean[len] & 0xC0) == 0x80) len--;   // continuation byte
  memset(setups.s[id].name, 0, sizeof(setups.s[id].name));
  memcpy(setups.s[id].name, clean.c_str(), len);
  if (!len) strncpy(setups.s[id].name, ("Setup " + String(id + 1)).c_str(), sizeof(setups.s[id].name) - 1);
}

String setupItemJson(uint8_t id) {
  const SetupModel& m = sm[id];
  String j = "{\"t\":\"setupItem\",\"id\":" + String(id) + ",\"name\":" + jstr(setupName(id)) +
             ",\"deleted\":" + jbool(setups.s[id].used == 2) + ",\"state\":\"" +
             String(m.state == 2 ? "ready" : (m.state == 1 ? "anchor" : "learning")) + "\",\"ends\":" + String(m.ends);
  if (m.state) j += ",\"kmh\":" + String((int)lroundf(m.speed * 3.6f)) + ",\"min\":" + String(m.minD) + ",\"max\":" + String(m.maxD);
  return j + "}";
}

void printSetups() {
  if (appMode) {
    sendLine("{\"t\":\"setupsStart\",\"active\":" + String(setups.active) + "}");
    for (uint8_t i = 0; i < SETUP_MAX; i++) if (setups.s[i].used) sendLine(setupItemJson(i));
    sendLine("{\"t\":\"setupsEnd\"}");
    return;
  }
  out("--- Setups ---");
  for (uint8_t i = 0; i < SETUP_MAX; i++) {
    if (!setups.s[i].used) continue;
    const SetupModel& m = sm[i];
    out(String(i == setups.active ? "* " : "  ") + String(i) + ": " + setupName(i) +
        (setups.s[i].used == 2 ? " (deleted)" : "") +
        (m.state ? ", about " + String((int)lroundf(m.speed * 3.6f)) + " km/h" : String(", learning")) +
        ", " + String(m.ends) + " ends");
  }
}

// setup | setup use <id> | setup new <name> | setup name <id> <name> | setup del <id>
void handleSetup(const String& arg, const String& rawArg) {
  const String ack = "{\"t\":\"ack\",\"cmd\":\"setup\",\"ok\":";
  if (arg == "") { printSetups(); return; }
  if (arg.startsWith("use ")) {
    const int id = arg.substring(4).toInt();
    if (id < 0 || id >= SETUP_MAX || setups.s[id].used != 1) { err("Unknown setup."); return; }
    setups.active = id;
    setupsSave();
    sessOffset = 0;
    rangeFit();
    rangeWarn = false;
    say("Setup: " + setupName(id), ack + "true}");
    printSetups(); emit(rangeJson());
    return;
  }
  if (arg.startsWith("new ")) {
    int id = -1;
    for (uint8_t i = 0; i < SETUP_MAX && id < 0; i++) if (!setups.s[i].used) id = i;
    if (id < 0) { err("All 8 setups are used."); return; }
    setups.s[id].used = 1;
    setName(id, rawArg.substring(4));
    setupsSave();
    say("New setup " + String(id) + ": " + setupName(id), ack + "true,\"id\":" + String(id) + "}");
    printSetups();
    return;
  }
  if (arg.startsWith("name ")) {
    String rest = arg.substring(5); rest.trim();
    const int sp = rest.indexOf(' ');
    const int id = (sp < 0 ? rest : rest.substring(0, sp)).toInt();
    if (sp < 0 || id < 0 || id >= SETUP_MAX || setups.s[id].used != 1) { err("Format: setup name <id> <name>"); return; }
    String rawRest = rawArg.substring(5); rawRest.trim();
    setName(id, rawRest.substring(rawRest.indexOf(' ') + 1));
    setupsSave();
    say("Setup " + String(id) + ": " + setupName(id), ack + "true}");
    printSetups(); emit(rangeJson());
    return;
  }
  if (arg.startsWith("del ")) {
    const int id = arg.substring(4).toInt();
    if (id < 0 || id >= SETUP_MAX || setups.s[id].used != 1) { err("Unknown setup."); return; }
    if (id == setups.active) { err("Choose another setup first."); return; }
    setups.s[id].used = 2;                       // name kept for the stored ends
    setupsSave();
    say("Setup " + setupName(id) + " deleted. Its ends stay in the sessions.", ack + "true}");
    printSetups();
    return;
  }
  err("Possible: setup, setup use <id>, setup new <name>, setup name <id> <name>, setup del <id>");
}

// A session cut off by a restart (empty battery): its ends are on the chip, but
// the session record is missing. Build it from the ends, so nothing is lost.
static uint16_t orEnds, orInvEnds, orInvArrows, orScored, orX, orShots;
static uint32_t orSum;
static uint8_t  orSetupFlags;
void finishOrphanSession() {
  if (!qfOk || !shotState.epoch || !shotState.lastId) return;
  if (logHasKey(shotState.epoch, shotState.lastId)) return;       // it was finished normally
  orEnds = orInvEnds = orInvArrows = orScored = orX = orShots = 0; orSum = 0;
  logForEachKind(0, [](const LogRec& r) {
    const EndRec& e = *(const EndRec*)&r;
    if (e.epoch != shotState.epoch || e.id != shotState.lastId) return true;
    orEnds++;
    orShots += e.arrows;
    if (e.flags & END_VALID) { orScored += e.arrows; orSum += e.sum; orX += e.x; }
    else { orInvEnds++; orInvArrows += e.arrows; }
    return true;
  }, LOG_ENDS);
  if (!orEnds) return;                                            // started, but nothing scored
  LogRec s;
  memset(&s, 0, sizeof(s));
  s.epoch = shotState.epoch;  s.id = shotState.lastId;  s.start = 0;
  s.ends = orEnds;  s.shots = orShots;  s.scores = orScored;  s.xCount = orX;
  s.avgX100 = orScored ? (uint16_t)((orSum * 100 + orScored / 2) / orScored) : 0;
  s.invalidEnds = orInvEnds;  s.invalidArrows = orInvArrows;  s.minutes = 0;
  s.flags = orInvEnds ? SLOT_MISMATCH : 0;                        // ended by the restart, not by hand
  logAppend(s, false);
}

// "awake <seconds>": stay reachable (Bluetooth, readings) without movement, e.g.
// while the app copies sessions. 0 releases the hold. Bow functions still sleep.
void setAwake(String args) {
  args.trim();
  long sec = args.toInt();
  if (sec < 0) sec = 0;
  if ((uint32_t)sec > AWAKE_MAX_S) sec = AWAKE_MAX_S;
  awakeUntil = sec ? millis() + (uint32_t)sec * 1000UL : 0;
  say(sec ? "Staying awake for " + String(sec) + " s." : String("Awake hold released."),
      "{\"t\":\"ack\",\"cmd\":\"awake\",\"s\":" + String(sec) + "}");
}

// "time <unix seconds> [offset to UTC in minutes]" from the app
void setClock(String args) {
  args.trim();
  const int sp = args.indexOf(' ');
  const uint32_t t = (uint32_t)atoll((sp < 0 ? args : args.substring(0, sp)).c_str());
  if (t < 1600000000UL) { err("Format: time <unix seconds> [tz minutes]"); return; }
  clockUnix  = t;
  clockMs    = millis();
  clockTzMin = sp < 0 ? 0 : (int16_t)args.substring(sp + 1).toInt();
  clockValid = true;
  say("Clock set: " + fmtUnix(t), "{\"t\":\"ack\",\"cmd\":\"time\"}");
}

// ============================================================================
// Sight identity: a fixed id from the chip and a name the owner can set, so the
// app can tell two sights apart (sessions, sync, preferences per sight)
// ============================================================================
String deviceIdHex() {
  char b[17];
  snprintf(b, sizeof(b), "%08lX%08lX", (unsigned long)NRF_FICR->DEVICEID[1], (unsigned long)NRF_FICR->DEVICEID[0]);
  return String(b);
}

String bleNameFull() { return sightName[0] ? String(BLE_NAME) + " " + sightName : String(BLE_NAME); }

void sightNameLoad() {
  memset(sightName, 0, sizeof(sightName));
  File f(InternalFS);
  if (f.open(NAME_FILE, FILE_O_READ)) {
    int n = f.read(sightName, sizeof(sightName) - 1);
    if (n < 0) n = 0;
    sightName[n] = 0;
    f.close();
  }
}

bool sightNameSave() {
  InternalFS.remove(NAME_FILE);
  if (!sightName[0]) return true;
  File f(InternalFS);
  if (!f.open(NAME_FILE, FILE_O_WRITE)) return false;
  f.write((const uint8_t*)sightName, strlen(sightName));
  f.close();
  return true;
}

String nameAck() { return "{\"t\":\"ack\",\"cmd\":\"name\",\"name\":" + jstr(String(sightName)) + ",\"id\":\"" + deviceIdHex() + "\"}"; }

// name | name <text> | name -   (raw keeps the case)
void handleName(String raw) {
  raw.trim();
  if (raw.length() == 0) {
    say(sightName[0] ? "Sight name: " + String(sightName) + " (id " + deviceIdHex() + ")" : "No name set (name <text>), id " + deviceIdHex(), nameAck());
    return;
  }
  String clean;
  if (raw != "-") {
    for (unsigned int i = 0; i < raw.length(); i++) {
      const char c = raw[i];
      if ((uint8_t)c < 0x20 || c == '"' || c == '\\') continue;
      clean += c;
    }
  }
  uint8_t len = clean.length() > 16 ? 16 : clean.length();
  while (len > 0 && len < clean.length() && ((uint8_t)clean[len] & 0xC0) == 0x80) len--;   // keep UTF-8 whole
  memset(sightName, 0, sizeof(sightName));
  memcpy(sightName, clean.c_str(), len);
  if (!sightNameSave()) { err("ERROR while saving!"); return; }
  Bluefruit.setName(bleNameFull().c_str());        // advertised from the next wake-up on
  say(sightName[0] ? "Sight name: " + String(sightName) : "Name removed.", nameAck());
}

// ============================================================================
// Shot matching: predictions per shot and learning from confirmed pairs
// ============================================================================
bool shotModelSave() {
  InternalFS.remove(SHOTMODEL_FILE);
  File f(InternalFS);
  if (!f.open(SHOTMODEL_FILE, FILE_O_WRITE)) return false;
  f.write((const uint8_t*)&shotModel, sizeof(shotModel));
  f.close();
  return true;
}

void shotModelLoad() {
  memset(&shotModel, 0, sizeof(shotModel));
  File f(InternalFS);
  if (f.open(SHOTMODEL_FILE, FILE_O_READ)) {
    ShotModelFile tmp;
    if (f.read(&tmp, sizeof(tmp)) == (int)sizeof(tmp) && tmp.magic == SHOTMODEL_MAGIC) shotModel = tmp;
    f.close();
  }
  if (shotModel.magic != SHOTMODEL_MAGIC) { memset(&shotModel, 0, sizeof(shotModel)); shotModel.magic = SHOTMODEL_MAGIC; shotModel.on = 1; }
}

// Solve the 3x3 ridge system (xx + ridge*I) k = xy for the sideways factors
void shotModelKx(const ShotModel& m, float k[3]) {
  float a[3][3] = { { m.xx[0] + SM_RIDGE_X, m.xx[1], m.xx[2] }, { m.xx[1], m.xx[3] + SM_RIDGE_X, m.xx[4] }, { m.xx[2], m.xx[4], m.xx[5] + SM_RIDGE_X } };
  float b[3] = { m.xy[0], m.xy[1], m.xy[2] };
  for (uint8_t c = 0; c < 3; c++) {
    uint8_t piv = c;
    for (uint8_t r = c + 1; r < 3; r++) if (fabsf(a[r][c]) > fabsf(a[piv][c])) piv = r;
    if (fabsf(a[piv][c]) < 1e-9f) { k[0] = k[1] = k[2] = 0; return; }
    if (piv != c) { for (uint8_t j = 0; j < 3; j++) { const float tmp = a[c][j]; a[c][j] = a[piv][j]; a[piv][j] = tmp; } const float tb = b[c]; b[c] = b[piv]; b[piv] = tb; }
    for (uint8_t r = 0; r < 3; r++) {
      if (r == c) continue;
      const float f = a[r][c] / a[c][c];
      for (uint8_t j = c; j < 3; j++) a[r][j] -= f * a[c][j];
      b[r] -= f * b[c];
    }
  }
  for (uint8_t i = 0; i < 3; i++) k[i] = b[i] / a[i][i];
}

float shotModelKy(const ShotModel& m) { return (m.spa + SM_RIDGE_Y) / (m.spp + SM_RIDGE_Y); }

// Residual spreads (cm) with the prior blended in while few examples exist
float shotModelSdy(const ShotModel& m) {
  const float ky = shotModelKy(m);
  float rss = m.saa - 2 * ky * m.spa + ky * ky * m.spp;
  if (rss < 0) rss = 0;
  return sqrtf((SM_PRIOR_N * SM_PRIOR_SDY * SM_PRIOR_SDY + rss) / (SM_PRIOR_N + m.n));
}
float shotModelSdx(const ShotModel& m) {
  float k[3]; shotModelKx(m, k);
  // rss = sum dx^2 - 2 k.xy + k^T xx k
  const float kxxk = k[0] * (m.xx[0] * k[0] + m.xx[1] * k[1] + m.xx[2] * k[2]) + k[1] * (m.xx[1] * k[0] + m.xx[3] * k[1] + m.xx[4] * k[2]) + k[2] * (m.xx[2] * k[0] + m.xx[4] * k[1] + m.xx[5] * k[2]);
  float rss = m.sxx - 2 * (k[0] * m.xy[0] + k[1] * m.xy[1] + k[2] * m.xy[2]) + kxxk;
  if (rss < 0) rss = 0;
  return sqrtf((SM_PRIOR_N * SM_PRIOR_SDX * SM_PRIOR_SDX + rss) / (SM_PRIOR_N + m.n));
}

// Height prediction from physics: how far the arrow lands above the end's mean, in cm
float shotPredY(float dAngDeg, uint8_t distM) {
  const float d = distM ? distM : 18;
  return tanf(dAngDeg * 0.01745329f) * d * 100.0f;
}

// Means of the shots that have a value
static void shotMeans(const float* ang, const float* cant, uint8_t n, float& angMean, float& cantMean) {
  float as = 0, cs = 0; uint8_t an = 0, cn = 0;
  for (uint8_t k = 0; k < n; k++) {
    if (ang[k] < NO_ANGLE) { as += ang[k]; an++; }
    if (cant[k] < NO_ANGLE) { cs += cant[k]; cn++; }
  }
  angMean = an ? as / an : 0; cantMean = cn ? cs / cn : 0;
}

// The shots of one end as a short line each (a single long line would be cut off on the
// way through Bluetooth): shotsStart, one shotItem per shot with the measured values and the
// model's predictions (cm from the end's mean), shotsEnd.
void sendShots(uint16_t endNo, const float* ang, const float* cant, const float* yaw, const float* roll, const float* rate, uint8_t n, uint8_t distM,
               const uint32_t* ms, const float* hold, const uint16_t* holdMs, const float* drop) {
  const ShotModel& m = shotModel.m[setups.active];
  float kx[3]; shotModelKx(m, kx);
  const float ky = shotModelKy(m);
  float angMean, cantMean; shotMeans(ang, cant, n, angMean, cantMean);
  sendLine("{\"t\":\"shotsStart\",\"end\":" + String(endNo) + ",\"n\":" + String(n) + ",\"dist\":" + String(distM) +
           ",\"sdx\":" + String(shotModelSdx(m), 1) + ",\"sdy\":" + String(shotModelSdy(m), 1) + ",\"modelN\":" + String(m.n) +
           ",\"matching\":" + jbool(shotModel.on) + "}");
  for (uint8_t k = 0; k < n; k++) {
    const bool hasAng = ang[k] < NO_ANGLE, hasCant = cant[k] < NO_ANGLE, hasKin = yaw[k] < NO_ANGLE;
    String j = "{\"t\":\"shotItem\",\"end\":" + String(endNo) + ",\"i\":" + String(k) +
         ",\"ang\":"  + (hasAng ? String(ang[k], 2) : String("null")) +
         ",\"cant\":" + (hasCant ? String(cant[k], 1) : String("null")) +
         ",\"yaw\":"  + (hasKin ? String(yaw[k], 2) : String("null")) +
         ",\"roll\":" + (hasKin ? String(roll[k], 2) : String("null")) +
         ",\"rate\":" + (hasKin ? String(rate[k], 0) : String("null"));
    if (ms) j += ",\"t\":" + String((ms[k] - ms[0]) / 1000.0f, 1);                       // seconds since the end's first shot
    if (hold && hold[k] < NO_ANGLE) j += ",\"hold\":" + String(hold[k], 2);
    if (holdMs && holdMs[k]) j += ",\"holdMs\":" + String(holdMs[k]);
    if (drop && drop[k] < NO_ANGLE) j += ",\"drop\":" + String(drop[k], 2);
    if (hasAng) j += ",\"py\":" + String(ky * shotPredY(ang[k] - angMean, distM), 1);
    if (hasKin && m.n > 0) {
      const float dc = hasCant ? cant[k] - cantMean : 0;
      j += ",\"px\":" + String(kx[0] * yaw[k] + kx[1] * roll[k] + kx[2] * dc, 1);
    } else if (hasKin) j += ",\"px\":0";
    sendLine(j + "}");
  }
  sendLine("{\"t\":\"shotsEnd\",\"end\":" + String(endNo) + ",\"n\":" + String(n) + "}");
}

String shotModelJson() {
  const ShotModel& m = shotModel.m[setups.active];
  float kx[3]; shotModelKx(m, kx);
  return "{\"t\":\"shotModel\",\"setup\":" + String(setups.active) + ",\"on\":" + jbool(shotModel.on) + ",\"n\":" + String(m.n) +
         ",\"ky\":" + String(shotModelKy(m), 3) + ",\"sdy\":" + String(shotModelSdy(m), 1) +
         ",\"kx\":[" + String(kx[0], 3) + "," + String(kx[1], 3) + "," + String(kx[2], 3) + "],\"sdx\":" + String(shotModelSdx(m), 1) + "}";
}

// shot teach <end> <i>:<dx mm>:<dy mm> ...   (positions relative to the group's mean, from the photo)
void shotTeach(String args) {
  args.trim();
  const int sp = args.indexOf(' ');
  if (sp < 0) { err("Format: shot teach <end> <i>:<dx>:<dy> ..."); return; }
  const int endNo = args.substring(0, sp).toInt();
  if (endNo != (int)lastEndNo || lastEndN == 0) { err("shot teach: not the last end (" + String(lastEndNo) + ")"); return; }
  ShotModel& m = shotModel.m[setups.active];
  float angMean, cantMean; shotMeans(lastEndAng, lastEndCant, lastEndN, angMean, cantMean);
  uint8_t added = 0;
  String rest = args.substring(sp + 1);
  while (rest.length()) {
    rest.trim();
    int e = rest.indexOf(' '); if (e < 0) e = rest.length();
    const String tok = rest.substring(0, e); rest = rest.substring(e);
    const int c1 = tok.indexOf(':'), c2 = c1 < 0 ? -1 : tok.indexOf(':', c1 + 1);
    if (c1 < 0 || c2 < 0) continue;
    const int i = tok.substring(0, c1).toInt();
    const float dx = tok.substring(c1 + 1, c2).toFloat() / 10.0f, dy = tok.substring(c2 + 1).toFloat() / 10.0f;   // mm -> cm
    if (i < 0 || i >= lastEndN) continue;
    if (lastEndAng[i] < NO_ANGLE) {
      const float pred = shotPredY(lastEndAng[i] - angMean, lastEndDist);
      m.spp += pred * pred; m.spa += pred * dy; m.saa += dy * dy;
    }
    if (lastEndYaw[i] < NO_ANGLE) {
      const float f[3] = { lastEndYaw[i], lastEndRoll[i], lastEndCant[i] < NO_ANGLE ? lastEndCant[i] - cantMean : 0 };
      m.xx[0] += f[0] * f[0]; m.xx[1] += f[0] * f[1]; m.xx[2] += f[0] * f[2];
      m.xx[3] += f[1] * f[1]; m.xx[4] += f[1] * f[2]; m.xx[5] += f[2] * f[2];
      for (uint8_t k = 0; k < 3; k++) m.xy[k] += f[k] * dx;
      m.sxx += dx * dx;
    }
    added++;
  }
  if (added) { m.n += added; if (!shotModelSave()) { err("ERROR while saving!"); return; } }
  say("Shot model: " + String(added) + " examples added, " + String(m.n) + " in total.",
      "{\"t\":\"ack\",\"cmd\":\"shot\",\"what\":\"teach\",\"added\":" + String(added) + ",\"n\":" + String(m.n) + "}");
  emit(shotModelJson());
}

// shot list [dist] | shot teach ... | shot model | shot reset [id] | shot on | shot off
void handleShot(String args) {
  args.trim();
  if (args.startsWith("list")) {
    const int d = args.substring(4).toInt();
    const uint8_t dist = d > 0 && d <= 150 ? (uint8_t)d : endDistM;
    sendShots(endCount + 1, endShotAng, endShotCant, endShotYaw, endShotRoll, endShotRate, endShotN, dist, endShotMs, endShotHold, endShotHoldMs, endShotDrop);
  } else if (args.startsWith("trace")) {
    // shot trace <i> (open end) | shot trace last <i> (last closed end)
    String a = args.substring(5); a.trim();
    const bool last = a.startsWith("last");
    if (last) { a = a.substring(4); a.trim(); }
    const int i = a.toInt();
    const uint8_t n = last ? lastEndN : endShotN;
    if (i < 0 || i >= n) { err("shot trace: shot 0.." + String(n ? n - 1 : 0)); return; }
    const ShotTrace& tr = last ? lastEndTrace[i] : endTrace[i];
    String j = "{\"t\":\"trace\",\"end\":" + String(last ? lastEndNo : endCount + 1) + ",\"i\":" + String(i) + ",\"n\":" + String(tr.n) + ",\"ms\":[";
    for (uint8_t k = 0; k < tr.n; k++) { if (k) j += ","; j += String(tr.ms[k]); }
    j += "],\"pitch\":[";
    for (uint8_t k = 0; k < tr.n; k++) { if (k) j += ","; j += String(tr.pitch[k]); }
    j += "],\"cant\":[";
    for (uint8_t k = 0; k < tr.n; k++) { if (k) j += ","; j += String(tr.cant[k]); }
    sendLine(j + "]}");
  } else if (args.startsWith("teach")) {
    shotTeach(args.substring(5));
  } else if (args == "model") {
    say("Shot model (setup " + setupName(setups.active) + "): " + String(shotModel.m[setups.active].n) + " examples, height +-" +
        String(shotModelSdy(shotModel.m[setups.active]), 1) + " cm, sideways +-" + String(shotModelSdx(shotModel.m[setups.active]), 1) + " cm", shotModelJson());
  } else if (args.startsWith("reset")) {
    int id = args.substring(5).toInt();
    if (args.substring(5).length() == 0 || args.substring(5) == " ") id = setups.active;
    if (id < 0 || id >= SETUP_MAX) { err("shot reset: setup 0.." + String(SETUP_MAX - 1)); return; }
    memset(&shotModel.m[id], 0, sizeof(ShotModel));
    if (!shotModelSave()) { err("ERROR while saving!"); return; }
    say("Shot model of setup " + setupName(id) + " reset.", "{\"t\":\"ack\",\"cmd\":\"shot\",\"what\":\"reset\",\"setup\":" + String(id) + "}");
    emit(shotModelJson());
  } else if (args == "on" || args == "off") {
    shotModel.on = args == "on";
    if (!shotModelSave()) { err("ERROR while saving!"); return; }
    // the loop re-applies the FIFO layout when gyroWanted() changed
    say(String("Shot matching ") + (shotModel.on ? "on." : "off."), shotModelJson());
  } else err("Possible: shot list [m], shot trace [last] <i>, shot teach <end> <i>:<dx>:<dy> ..., shot model, shot reset [id], shot on|off");
}

void appOn() {
  appMode = true;
  sendLine("{\"t\":\"hello\",\"proto\":" + String(PROTO_VERSION) + ",\"fw\":\"" + FW_VERSION +
           "\",\"name\":\"" + BLE_NAME + "\",\"imu\":" + jbool(imuOk) + ",\"log\":" + jbool(qfOk) +
           ",\"twim89\":" + jbool(twimWorkaround) + ",\"id\":\"" + deviceIdHex() + "\",\"sname\":" + jstr(String(sightName)) + ",\"led\":" + jbool(ledPresent) + "}");
  sendCfg();
  sendLine(statusJson());
  sendLine(sessionJson());
  sendLine(levelJson());
  sendLine(logInfoJson());
  sendLine(rangeJson());
  sendLine(shotModelJson());
  printSetups();
  rangeEvWarn = false; rangeEvOk = false; rangeEvDir = 0;   // next change is reported again
  rangeEventIfChanged();
}

// Commands that only read or write elsewhere: they leave an open score question alone
bool keepsPendingScore(const String& l) {
  return l == "status" || l == "?" || l == "get" || l == "config" || l == "help" || l == "h" ||
         l == "shots" || l == "level" || l == "bat" || l == "range" || l == "setup" || l == "angle" || l == "name" || l.startsWith("name ") || l.startsWith("shot ") ||
         l == "log" || l == "log all" || l == "log info" || l.startsWith("log since ") ||
         l.startsWith("log ends ") || l.startsWith("log put ") || l.startsWith("log putend ") ||
         l.startsWith("time ") || l.startsWith("awake ") ||
         l == "app on" || l == "app off" || l == "live on" || l == "live off";
}

void handleCommand(String line) {
  line.trim();
  const String raw = line;                        // original case (setup names)
  line.toLowerCase();
  if (line.length() == 0) return;

  // Setting the distance must not discard an open score question
  if (line == "dist" || line.startsWith("dist ")) { handleDist(line.substring(4)); return; }

  // Pending confirmation for a mismatching end. Only a command that changes the
  // session or its settings withdraws it; the app keeps syncing in the meantime.
  if (pendingScore && !keepsPendingScore(line)) {
    pendingScore = false;
    if (line == "yes") {
      // The entered arrow count wins over the sensor count
      shotCount = shotCount - endShots + pendingN;
      mismatchAccepted = true;
      closeEndValid(pendingVals, pendingN, pendingX, 0);
      return;
    }
    if (line == "split" && pendingN < endShots) {
      // Two ends were shot without saving in between: these scores are the first
      // end, the remaining shots stay counted for the next one. Counts match, so
      // this is no mismatch.
      closeEndValid(pendingVals, pendingN, pendingX, endShots - pendingN);
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
    applyImuThresholds();
    applyTxPower();
    if (appMode) sendCfg();
    else         out("Default values loaded (permanent with 'save').");
  }
  else if (line == "app on")    appOn();
  else if (line == "dfu")       enterDfu();
  else if (line == "app off")   { appMode = false; out("App mode off, human-readable output."); }
  else if (line == "live on")   { liveMode = true;  say("Live output on (every 2 s).", "{\"t\":\"ack\",\"cmd\":\"live\",\"on\":true}"); }
  else if (line == "live off")  { liveMode = false; say("Live output off.", "{\"t\":\"ack\",\"cmd\":\"live\",\"on\":false}"); }
  else if (!ledPresent && (line == "mode auto" || line == "mode on" || line == "level signal on" || line == "range signal on")) err("This sight has no UV LED.");
  else if (line == "mode auto") { ledMode = MODE_AUTO; lvl.ledMode = ledMode; levelSave(); say("LED: auto (light sensor)", "{\"t\":\"ack\",\"cmd\":\"mode\",\"mode\":\"auto\"}"); }
  else if (line == "mode on")   { ledMode = MODE_ON;   lvl.ledMode = ledMode; levelSave(); say("LED: always on (cant blinking still works)", "{\"t\":\"ack\",\"cmd\":\"mode\",\"mode\":\"on\"}"); }
  else if (line == "mode off")  { ledMode = MODE_OFF;  lvl.ledMode = ledMode; levelSave(); say("LED: off (cant blinking still works)", "{\"t\":\"ack\",\"cmd\":\"mode\",\"mode\":\"off\"}"); }
  else if (line == "shots")               handleShots("");
  else if (line.startsWith("shots "))     { String a = line.substring(6); a.trim(); handleShots(a); }
  else if (line == "score")               handleScore("");
  else if (line.startsWith("score "))     handleScore(line.substring(6));
  else if (line == "log")                 printLog(0, appMode);
  else if (line == "log all")             printLog(0, true);
  else if (line == "log info")            printLogInfo();
  else if (line == "log test")            logTest();
  else if (line.startsWith("log del "))   logDelete(line.substring(8));
  else if (line == "log clear" || line.startsWith("log clear ")) logClear(line.substring(9));
  else if (line.startsWith("log since ")) printLog((uint32_t)line.substring(10).toInt(), true);
  else if (line.startsWith("time "))      setClock(line.substring(5));
  else if (line.startsWith("awake "))     setAwake(line.substring(6));
  else if (line == "bat")                 printBat();
  else if (line == "bat reset")           bhReset();
  else if (line.startsWith("log put "))   logPut(line.substring(8));
  else if (line.startsWith("log putend ")) logPutEnd(line.substring(11));
  else if (line.startsWith("log ends "))  printEnds(line.substring(9));
  else if (line == "range")               printRange();
  else if (line.startsWith("shot "))      handleShot(line.substring(5));
  else if (line == "name")                handleName("");
  else if (line.startsWith("name "))      handleName(raw.substring(5));
  else if (line == "setup")               handleSetup("", "");
  else if (line.startsWith("setup "))     handleSetup(line.substring(6), raw.substring(6));
  else if (line == "range signal on" || line == "range signal off") {
    lvl.rangeSignal = line == "range signal on";
    if (!lvl.rangeSignal) { rangeWarn = false; rangeOk = false; rangeEventIfChanged(); }
    if (!levelSave()) { err("ERROR while saving!"); return; }
    say(String("Range signal ") + (lvl.rangeSignal ? "on." : "off."), rangeJson());
  }
  else if (line == "angle")               handleAngle("");
  else if (line.startsWith("angle "))     { String a = line.substring(6); a.trim(); handleAngle(a); }
  else if (line == "level")               handleLevel("");
  else if (line.startsWith("level "))     { String a = line.substring(6); a.trim(); handleLevel(a); }
  else if (line.startsWith("set "))       handleSet(line.substring(4));
  else err("Unknown command. 'help' lists all commands.");

  lastSensorMs = millis() - SENSOR_INTERVAL_MS;  // apply changes immediately
}

void feedChar(char c) {
  if (c == '\n' || c == '\r') {
    if (rxBuf.length()) { handleCommand(rxBuf); rxBuf = ""; }
  } else if (rxBuf.length() < 240) {   // the longest app command is about 150 characters
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
  Bluefruit.setName(bleNameFull().c_str());
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
    if (c == CHG_FULL && lastCharge == CHG_CHARGING) bhPoint(BP_FULL, 4200, true);   // anchor: 100 %
    lastCharge = c;
  }

  lastVbat = readVbat();
  lastPct  = vbatToPercent(lastVbat);
  // A reading counts as unloaded if the LED has been dark long enough and no charger is on
  if (ledMaNow == 0 && (millis() - ledOffMs) >= BH_RELAX_MS && !usbPresent()) {
    unloadedMv = (uint16_t)(lastVbat * 1000);
    unloadedMs = millis();
  }
  lastLdr  = ledPresent ? readLdr() : 0;          // the light sensor only serves the LED

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
  else {
    // share of the difference to follow per reading (exponential fade)
    const float alpha = cfg.fadeS <= 0 ? 1.0f : 1.0f - expf(-(SENSOR_INTERVAL_MS / 1000.0f) / cfg.fadeS);
    brightNow += alpha * (target - brightNow);
  }
  dutyFull = dutyForVbat(lastVbat, percentToMa(brightNow));
  dutyTilt = dutyForVbat(lastVbat, percentToMa(tiltBrightness()));
}

// ============================================================================
// Setup
// ============================================================================
// Is the transistor stage of the UV LED fitted? See UV_HAS_LED.
bool detectLedStage() {
#if UV_HAS_LED == 0
  return false;
#elif UV_HAS_LED == 1
  return true;
#else
  uint8_t low = 0;
  for (uint8_t i = 0; i < 5; i++) {
    pinMode(PIN_UV, OUTPUT); digitalWrite(PIN_UV, HIGH); delay(1);
    pinMode(PIN_UV, INPUT);                       // no pull: the base-emitter junction drains the pin, an open pin keeps its charge
    delayMicroseconds(200);
    if (digitalRead(PIN_UV) == LOW) low++;
  }
  pinMode(PIN_UV, OUTPUT); digitalWrite(PIN_UV, LOW);
  return low >= 3;
#endif
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_UV, OUTPUT);
  digitalWrite(PIN_UV, LOW);
  ledPresent = detectLedStage();
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
  // logInit() runs after bleInit(): the random epoch needs the SoftDevice
  levelLoad();
  sightNameLoad();
  shotModelLoad();

  imuOk = imuInit();
  bleInit();
  logInit();
  chargeInit();
  lastCharge = readCharge();
  bhLoad();
  bhFit();
  setupsLoad();
  finishOrphanSession();
  rangeFit();
  // LED switch exactly as it was set
  ledMode = lvl.ledMode <= MODE_OFF ? (LedMode)lvl.ledMode : MODE_AUTO;
  bhTickMs = millis();
  bhPrevUsb = usbPresent();
  unloadedMv = (uint16_t)(readVbat() * 1000); unloadedMs = millis();
  bhPoint(BP_BOOT, unloadedMv, true);             // the time before this restart is unknown

  lastMotionMs = millis();                        // active right after start
  lastSensorMs = millis() - SENSOR_INTERVAL_MS;
}

// ============================================================================
// Loop
// ============================================================================
void loop() {
  uint32_t   now       = millis();
  const bool connected = Bluefruit.connected();
  imuBus(true);

  if (!connected) { liveMode = false; welcomed = false; appMode = false; }
  // A command cut off by a lost connection must not be glued to the first one
  // of the next connection: start every connection with an empty line buffer
  static bool wasConnected = false;
  if (connected != wasConnected) { rxBuf = ""; wasConnected = connected; }

  // Only movement keeps the board active (a connection does not)
  if (imuMotionSinceLastCheck()) lastMotionMs = now;

  // Shot: the INT1 edge gives the exact time; the latched TAP_SRC confirms it
  // (in shot mode INT1 only carries the tap) and re-arms the line
  bool     shot   = false;
  uint32_t shotAt = now;
  bool     edge   = false;
  if (int1Flag) {
    noInterrupts();
    shotAt   = int1Ms;
    int1Flag = false;
    interrupts();
    edge = true;
  }
  if (imuTapSinceLastCheck()) shot = true;
  if (!edge) shotAt = now;                   // tap found by polling only
  if (imuFast && !shot && fifoHasGyro != gyroWanted()) { imuFast = false; imuSetFast(true); }   // calibration or "shot on|off" changed the layout
  if (shot) {
    lastMotionMs = now;
    if (shotAllowed(shotAt)) {
      bool clip = false; ShotKin kin;
      const float g = readShotFifo(&clip, &kin);   // strength and bow rotation from the FIFO
      registerShot(shotAt, g, clip, kin);
    }
  } else if (imuFast && fifoWords() > FIFO_KEEP_WORDS) {
    fifoRestart();                            // keep only the recent data
  }

  const bool active = (now - lastMotionMs) < cfg.timeoutS * 1000UL;

  // Stay reachable without movement while charging over USB or while the app
  // holds the sight awake (e.g. "Copy to sight"). Only Bluetooth and readings:
  // LED, shot detection and cant indicator still need movement.
  const bool usb  = usbPresent();
  const bool held = (int32_t)(awakeUntil - now) > 0;
  awakeReason = active ? "" : (usb ? "usb" : (held ? "app" : ""));
  const bool reachable = active || usb || held;

  // Battery history: time per state, and points at state changes
  bhTick(now, reachable);
  if (usb != bhPrevUsb) {
    if (usb) {                                   // voltage from just before the charger took over
      plugMs = now;
      plugSoc = (now - unloadedMs) < 120000 ? rawPercent(unloadedMv / 1000.0f) : -1;
      bhPoint(BP_USB_IN, unloadedMv, true);
      bhPending = 0;
    } else {
      plugSoc = -1;
      ledOffMs = now;                             // let the charged battery relax first
      bhPointUnloaded(BP_USB_OUT, true);
    }
    bhPrevUsb = usb;
  }
  if (active != bhPrevActive) {
    if (!active) {                               // falling asleep
      bhIdlePointMs = now;
      if (!usb) bhPointUnloaded(BP_IDLE, false);
    } else if (bhIdlePointMs && (now - bhIdlePointMs) >= BH_IDLE_MIN_MS && !usb) {
      bhPoint(BP_WAKE, (uint16_t)(readVbat() * 1000), true);   // LED is still off here
    }
    bhPrevActive = active;
  }
  bhService(now, usb);

  // Shot detection only if the counter is on and the bow is in use; at rest the
  // accelerometer runs at its slowest rate
  imuSetFast(active && shotState.shotsOn);
  imuSetIdleOdr(!active);

  // End the session automatically after session_end minutes without a shot
  if (sessionActive && (now - sessionLastShotMs) >= cfg.sessionEndMin * 60000UL) {
    finishSession(false);
  }

  bleUpdate(reachable);
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
    if (ledMaNow > 0) { ledMaNow = 0; ledOffMs = now; }
    uvOff();
    ledState = LS_OFF;
    tiltReset();
    wasInactive = true;
    darkCount = brightCount = 0;
    if (reachable) {
      // Kept awake: fresh readings for the app, LED stays off
      if ((now - lastSensorMs) >= SENSOR_INTERVAL_MS) {
        lastSensorMs = now;
        evaluate();
        wasInactive = true;            // evaluate() cleared it; the bow is still at rest
        if (liveMode) { sendStatus(); lastReportMs = now; }
      }
      if (connected && !liveMode && cfg.reportS > 0 && (now - lastReportMs) >= cfg.reportS * 1000UL) {
        sendStatus();
        lastReportMs = now;
      }
      imuBus(false);
      delay(ACTIVE_POLL_MS);
      return;
    }
    imuBus(false);
    idleSleep();
    return;
  }

  // ---- Active ----
  if (wasInactive || (now - lastSensorMs) >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;
    evaluate();
    if (liveMode) { sendStatus(); lastReportMs = now; }
  }

  const bool lvlOn = levelActive();
  const bool angOn = angleActive();
  if (lvlOn || angOn) updateOrientation(lvlOn, angOn);
  if (!angOn && !lvlOn) pitchFill = 0;        // no stale readings into the next session
  rangeWarnUpdate(millis());
  updateLed(millis());

  // Periodic status report, only when connected
  if (connected && !liveMode && cfg.reportS > 0 && (now - lastReportMs) >= cfg.reportS * 1000UL) {
    sendStatus();
    lastReportMs = now;
  }

  imuBus(false);
  delay((lvlOn || angOn) ? LEVEL_LOOP_MS : ACTIVE_POLL_MS);
}
