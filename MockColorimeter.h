/*
 * MockColorimeter.h
 * Compile-time drop-in replacement for Colorimeter used for isolated
 * application-code testing without physical sensors or LittleFS.
 *
 * -----------------------------------------------------------------------
 * HOW TO USE
 * -----------------------------------------------------------------------
 * In colorimeter_tcs34725.ino, define the guard BEFORE the conditional
 * includes (see updated .ino for the exact pattern):
 *
 *   #define USE_MOCK_COLORIMETER       // ← enable mock
 *   // #define USE_MOCK_COLORIMETER    // ← disable (real hardware)
 *
 * When the guard is defined the .ino includes MockColorimeter.h instead
 * of TCS34725_Colorimeter.h and declares:
 *
 *   MockColorimeter colorimeter;
 *
 * instead of:
 *
 *   Colorimeter colorimeter;
 *
 * Every call in loop() (update(), getAbsorbance(), getTCSRaw(), …) is
 * forwarded to the mock identically — your application code does not
 * change at all between mock and real builds.
 *
 * -----------------------------------------------------------------------
 * SELECTING A TEST SCENARIO
 * -----------------------------------------------------------------------
 * Call colorimeter.setScenario(MockScenario::XXXX) in setup(), or send
 * a digit over Serial at runtime (no line ending required):
 *
 *   '0'  STATIC_NORMAL      Fixed mid-range absorbance; both sensors OK.
 *   '1'  STATIC_ZERO        Absorbance = 0 (fully transmissive blank-level).
 *   '2'  STATIC_HIGH        Absorbance near top of range; verify clamping.
 *   '3'  RAMP               Absorbance ramps 0→1.5 over 30 s, then repeats.
 *   '4'  SINE               Absorbance oscillates sinusoidally over 20 s.
 *   '5'  NOISY              Mid-range + Gaussian-like noise; stress display.
 *   '6'  OVERFLOW_TCS       TCS34725 saturated; BH1750 normal.
 *   '7'  OVERFLOW_BH1750    BH1750 saturated; TCS34725 normal.
 *   '8'  IO_ERROR_PRIMARY   Primary sensor I/O fault; secondary OK.
 *   '9'  BOTH_SENSORS_FAIL  Both sensors fault → abort path.
 *   'o'  OUT_OF_RANGE       Absorbance outside calibration range → -1.
 *
 *   'b'  Perform a mock blank (sets is_blanked = true, logs to Serial).
 *   '?'  Print current scenario name to Serial.
 *
 * -----------------------------------------------------------------------
 * PUBLIC API PARITY
 * -----------------------------------------------------------------------
 * MockColorimeter exposes every public method and constant of Colorimeter
 * so application code compiles unchanged under both builds.  Methods that
 * have no meaningful mock equivalent (menu navigation, gain cycling) are
 * stubs that log a message and return immediately.
 */

#ifndef MOCK_COLORIMETER_H
#define MOCK_COLORIMETER_H

#include <Arduino.h>
#include <math.h>
#include "SensorCommon.h"   // SensorResult – no hardware dependencies

// ---------------------------------------------------------------------------
// Enums – mirrors of those defined in TCS34725_Colorimeter.h / Configuration.h
// These are redeclared here so MockColorimeter.h has no dependency on the
// real sensor headers (which pull in I2C / Adafruit / BH1750 libraries).
// ---------------------------------------------------------------------------
enum OperatingMode : uint8_t {
  MODE_MEASURE = 0,
  MODE_MENU    = 1,
  MODE_MESSAGE = 2,
  MODE_ABORT   = 3
};

enum AppState : uint8_t {
  APP_MENU_DRIVEN = 0,
  APP_POLLING     = 1,
  APP_INTERRUPT   = 2   // fully idle; application code calls getters on its own schedule
};

enum PollingState : uint8_t {
  POLLING_IDLE    = 0,
  POLLING_STREAM  = 1,
  POLLING_ONESHOT = 2
};
 
enum SensorType : uint8_t {
  SENSOR_TYPE_TCS34725 = 0,
  SENSOR_TYPE_BH1750   = 1
};

// ---------------------------------------------------------------------------
// MockScenario
// ---------------------------------------------------------------------------
enum class MockScenario : uint8_t {
  STATIC_NORMAL     = 0,
  STATIC_ZERO       = 1,
  STATIC_HIGH       = 2,
  RAMP              = 3,
  SINE              = 4,
  NOISY             = 5,
  OVERFLOW_TCS      = 6,
  OVERFLOW_BH1750   = 7,
  IO_ERROR_PRIMARY  = 8,
  BOTH_SENSORS_FAIL = 9,
  OUT_OF_RANGE      = 10,
  _COUNT            = 11
};

// ---------------------------------------------------------------------------
// Seeded data tables for STATIC_* scenarios
// ---------------------------------------------------------------------------
struct MockSeed {
  float    absorbance;       // drives getAbsorbance() / getTransmittance()
  float    tcs_raw;          // raw TCS34725 count  (float for API parity)
  float    bh1750_raw;       // raw BH1750 lux
  bool     tcs_present;
  bool     bh1750_present;
  SensorResult tcs_result;
  SensorResult bh1750_result;
  SensorType   primary;
  const char*  label;
};

static const MockSeed MOCK_SEEDS[] = {
  // STATIC_NORMAL
  { 0.35f, 22000.0f, 180.5f, true,  true,  SENSOR_OK,       SENSOR_OK,       SENSOR_TYPE_TCS34725, "STATIC_NORMAL"     },
  // STATIC_ZERO
  { 0.00f, 32000.0f, 400.0f, true,  true,  SENSOR_OK,       SENSOR_OK,       SENSOR_TYPE_TCS34725, "STATIC_ZERO"       },
  // STATIC_HIGH
  { 1.45f,  1200.0f,   8.2f, true,  true,  SENSOR_OK,       SENSOR_OK,       SENSOR_TYPE_TCS34725, "STATIC_HIGH"       },
  // RAMP              (dynamic – seed ignored for absorbance)
  { 0.00f, 20000.0f, 150.0f, true,  true,  SENSOR_OK,       SENSOR_OK,       SENSOR_TYPE_TCS34725, "RAMP"              },
  // SINE              (dynamic)
  { 0.00f, 20000.0f, 150.0f, true,  true,  SENSOR_OK,       SENSOR_OK,       SENSOR_TYPE_TCS34725, "SINE"              },
  // NOISY             (dynamic)
  { 0.50f, 18000.0f, 120.0f, true,  true,  SENSOR_OK,       SENSOR_OK,       SENSOR_TYPE_TCS34725, "NOISY"             },
  // OVERFLOW_TCS
  { 0.35f,  -1.0f,   180.5f, true,  true,  SENSOR_OVERFLOW, SENSOR_OK,       SENSOR_TYPE_TCS34725, "OVERFLOW_TCS"      },
  // OVERFLOW_BH1750
  { 0.35f, 22000.0f,  -1.0f, true,  true,  SENSOR_OK,       SENSOR_OVERFLOW, SENSOR_TYPE_TCS34725, "OVERFLOW_BH1750"   },
  // IO_ERROR_PRIMARY
  { -1.0f,  -1.0f,   180.5f, true,  true,  SENSOR_IO_ERROR, SENSOR_OK,       SENSOR_TYPE_TCS34725, "IO_ERROR_PRIMARY"  },
  // BOTH_SENSORS_FAIL
  { -1.0f,  -1.0f,    -1.0f, false, false, SENSOR_IO_ERROR, SENSOR_IO_ERROR, SENSOR_TYPE_TCS34725, "BOTH_SENSORS_FAIL" },
  // OUT_OF_RANGE      (absorbance outside a typical calibration range)
  { 2.50f, 10000.0f,  60.0f, true,  true,  SENSOR_OK,       SENSOR_OK,       SENSOR_TYPE_TCS34725, "OUT_OF_RANGE"      },
};
static_assert(sizeof(MOCK_SEEDS)/sizeof(MOCK_SEEDS[0]) == (uint8_t)MockScenario::_COUNT,
              "MOCK_SEEDS size must match MockScenario::_COUNT");

// ---------------------------------------------------------------------------
// MockColorimeter
// ---------------------------------------------------------------------------
class MockColorimeter {
public:
  // Constants matching Colorimeter exactly so loop() code compiles unchanged
  static const uint8_t  NUM_BLANK_SAMPLES = 50;
  static const uint32_t BLANK_DT_MS       = 50;
  static const uint32_t LOOP_DT_MS        = 100;
  static const uint32_t DEBOUNCE_DT_MS    = 600;

  static const String ABSORBANCE_STR;
  static const String TRANSMITTANCE_STR;
  static const String RAW_TCS_STR;
  static const String RAW_BH1750_STR;
  static const String ABOUT_STR;

  MockColorimeter()
    : _scenario(MockScenario::STATIC_NORMAL),
      _app_state(APP_POLLING),
      _polling_state(POLLING_IDLE),
      _mode(MODE_MEASURE),
      _measurement_name(ABSORBANCE_STR),
      _is_blanked(false),
      _last_cmd_ms(0),
      _scenario_start_ms(0) {}

  // ---- Lifecycle ----------------------------------------------------------

  bool begin() {
    _scenario_start_ms = millis();
    const MockSeed& s = _seed();
    Serial.println("\n[MOCK] MockColorimeter initialised");
    Serial.print  ("[MOCK] Scenario: "); Serial.println(s.label);
    Serial.print  ("[MOCK] TCS34725: "); Serial.println(s.tcs_present  ? "OK (mock)" : "absent");
    Serial.print  ("[MOCK] BH1750:   "); Serial.println(s.bh1750_present ? "OK (mock)" : "absent");

    if (!s.tcs_present && !s.bh1750_present) {
      Serial.println("[MOCK] ABORT: both sensors absent in this scenario");
      _mode = MODE_ABORT;
      return false;
    }

    _print_scenario_help();
    if (_app_state == APP_POLLING) _print_polling_help();
    return true;
  }

  // update() handles Serial scenario-switching and (in menu mode) prints
  // the current measurement – matching the real Colorimeter's behaviour.
  // In APP_INTERRUPT mode it is a near-no-op: only the Serial command handler
  // runs (for blank and mode-switch), and nothing is printed or read.
  void update() {
    _handle_serial();   // always check for scenario / mode commands

    // Display only in menu-driven mode
    if (_app_state == APP_MENU_DRIVEN && _mode == MODE_MEASURE) {
      _display_measure();
    }
    // APP_POLLING and APP_INTERRUPT produce no output from update() itself
  }

  // ---- Scenario control ---------------------------------------------------

  void setScenario(MockScenario sc) {
    _scenario           = sc;
    _scenario_start_ms  = millis();
    Serial.print("[MOCK] Scenario → "); Serial.println(_seed().label);
    if (!_seed().tcs_present && !_seed().bh1750_present) {
      _mode = MODE_ABORT;
    } else {
      _mode = MODE_MEASURE;
    }
  }

  MockScenario getScenario() const { return _scenario; }

  // ---- AppState -----------------------------------------------------------

  void setAppState(AppState s) {
    _app_state = s;
    if (s == APP_POLLING) {
      _polling_state = POLLING_IDLE;
      _print_polling_help();
    } else if (s == APP_INTERRUPT) {
      Serial.println("[MOCK] INT: idle – call getters directly; update() produces no output");
      Serial.println("[MOCK] INT: commands: b=blank  m=menu  ?=help  0-9/o=scenario");
    }
  }
  AppState getAppState() const { return _app_state; }

  // True while POLLING_STREAM – application code should read and print.
  bool isPollingActive() const {
    return _app_state == APP_POLLING && _polling_state == POLLING_STREAM;
  }

  // Returns true exactly once per one-shot request, then resets to IDLE.
  bool consumeOneShot() {
    if (_app_state == APP_POLLING && _polling_state == POLLING_ONESHOT) {
      _polling_state = POLLING_IDLE;
      return true;
    }
    return false;
  }

  // ---- Core measurements --------------------------------------------------

  // Primary sensor raw value (used for Abs / Trans calculations)
  SensorResult getPrimaryRaw(float& out) {
    const MockSeed& s = _seed();
    if (s.tcs_result != SENSOR_OK && s.tcs_result != SENSOR_OVERFLOW) {
      return s.tcs_result; // IO error
    }
    float abs = _compute_absorbance();
    // Convert absorbance back to a plausible raw count via blank_value
    // (blank_value is set during blankSensor(); default 32000)
    float blank = _blank_value > 0.0f ? _blank_value : 32000.0f;
    out = blank * powf(10.0f, -abs);
    return SENSOR_OK;
  }

  // TCS34725-specific raw value
  SensorResult getTCSRaw(float& out) {
    const MockSeed& s = _seed();
    if (s.tcs_result != SENSOR_OK) return s.tcs_result;
    out = _dynamic_raw(s.tcs_raw);
    return SENSOR_OK;
  }

  // BH1750-specific raw value
  SensorResult getBH1750Raw(float& out) {
    const MockSeed& s = _seed();
    if (s.bh1750_result != SENSOR_OK) return s.bh1750_result;
    out = _dynamic_raw(s.bh1750_raw);
    return SENSOR_OK;
  }

  float getTransmittance() {
    float abs = getAbsorbance();
    if (abs < 0.0f) return -1.0f;
    return powf(10.0f, -abs);
  }

  float getAbsorbance() {
    const MockSeed& s = _seed();
    if (s.tcs_result == SENSOR_IO_ERROR) return -1.0f;
    return _compute_absorbance();
  }

  float getMeasurementValue() {
    if (_measurement_name == ABSORBANCE_STR)    return getAbsorbance();
    if (_measurement_name == TRANSMITTANCE_STR) return getTransmittance();
    if (_measurement_name == RAW_TCS_STR) {
      float v; return (getTCSRaw(v) == SENSOR_OK) ? v : -1.0f;
    }
    if (_measurement_name == RAW_BH1750_STR) {
      float v; return (getBH1750Raw(v) == SENSOR_OK) ? v : -1.0f;
    }
    // Calibration: apply a simple linear stand-in (slope=2, intercept=0)
    // so calibrated measurements return a plausible non-zero value.
    float a = getAbsorbance();
    return (a >= 0.0f) ? a * 2.0f : -1.0f;
  }

  String getMeasurementUnits() {
    if (_measurement_name == ABSORBANCE_STR    ||
        _measurement_name == TRANSMITTANCE_STR ||
        _measurement_name == RAW_TCS_STR       ||
        _measurement_name == RAW_BH1750_STR)   return "";
    return "ppm";  // plausible stand-in for calibrated measurements
  }

  // ---- Blanking -----------------------------------------------------------

  void blankSensor(bool set_blanked = true) {
    Serial.println("[MOCK] blankSensor() called");
    _blank_value = 32000.0f; // representative mid-scale value
    _is_blanked  = set_blanked;
    if (set_blanked) Serial.println("[MOCK] Blank accepted (mock value = 32000)");
  }

  // ---- State accessors (mirrors of Colorimeter) ---------------------------

  OperatingMode getMode()            const { return _mode; }
  String        getMeasurementName() const { return _measurement_name; }
  bool          getIsBlanked()       const { return _is_blanked; }
  bool          isTCSPresent()       const { return _seed().tcs_present; }
  bool          isBH1750Present()    const { return _seed().bh1750_present; }
  SensorType    getPrimarySensor()   const { return _seed().primary; }

private:
  MockScenario  _scenario;
  AppState      _app_state;
  PollingState  _polling_state;
  OperatingMode _mode;
  String       _measurement_name;
  bool         _is_blanked;
  uint32_t     _last_cmd_ms;
  uint32_t     _scenario_start_ms;
  float        _blank_value = 32000.0f;

  const MockSeed& _seed() const {
    return MOCK_SEEDS[static_cast<uint8_t>(_scenario)];
  }

  // ---- Dynamic value generation -------------------------------------------

  float _elapsed_s() const {
    return static_cast<float>(millis() - _scenario_start_ms) / 1000.0f;
  }

  float _compute_absorbance() const {
    const MockSeed& s = _seed();
    float t = _elapsed_s();

    switch (_scenario) {
      case MockScenario::RAMP:
        // 0 → 1.5 over 30 s, then resets
        return fmodf(t / 30.0f, 1.0f) * 1.5f;

      case MockScenario::SINE:
        // Oscillates 0.1 → 0.9 over a 20 s period
        return 0.5f + 0.4f * sinf(2.0f * M_PI * t / 20.0f);

      case MockScenario::NOISY: {
        // Mid-range + deterministic "noise" from a fast oscillation
        // (avoids needing a PRNG; good enough for display stress testing)
        float noise = 0.05f * sinf(t * 37.7f) * cosf(t * 19.1f);
        return s.absorbance + noise;
      }

      default:
        return s.absorbance;
    }
  }

  // Apply the same dynamic modifier to raw values so they are consistent
  // with the absorbance (higher absorbance = lower raw counts / lux).
  float _dynamic_raw(float base_raw) const {
    float abs = _compute_absorbance();
    if (abs < 0.0f) return base_raw; // error scenarios – return seed as-is
    // raw ∝ 10^(-absorbance) relative to blank
    float scale = powf(10.0f, -abs) / powf(10.0f, -_seed().absorbance);
    return base_raw * scale;
  }

  // ---- Serial input -------------------------------------------------------

  void _handle_serial() {
    if (!Serial.available()) return;

    char cmd = Serial.read();
    uint32_t now = millis();
    if ((now - _last_cmd_ms) < 200) return;
    _last_cmd_ms = now;

    // ---- Polling-mode commands (highest priority) -------------------------
    if (_app_state == APP_POLLING) {
      if (cmd == 's') {
        _polling_state = POLLING_STREAM;
        Serial.println("[MOCK] POLL: streaming ON  ('x' to stop, Enter for one-shot)");
        return;
      }
      if (cmd == 'x' || cmd == 'q') {
        _polling_state = POLLING_IDLE;
        Serial.println("[MOCK] POLL: idle  ('s' to stream, Enter for one-shot)");
        return;
      }
      if (cmd == '\n' || cmd == '\r') {
        _polling_state = POLLING_ONESHOT;
        return;
      }
      if (cmd == 'm') {
        _app_state     = APP_MENU_DRIVEN;
        _polling_state = POLLING_IDLE;
        Serial.println("[MOCK] Switched to menu-driven mode");
        return;
      }
      if (cmd == '?') {
        _print_polling_help();
        return;
      }
    }

    // ---- Scenario selection (available in both modes) ---------------------
    if (cmd >= '0' && cmd <= '9') {
      uint8_t idx = cmd - '0';
      if (idx < (uint8_t)MockScenario::_COUNT) {
        setScenario(static_cast<MockScenario>(idx));
      }
      return;
    }
    if (cmd == 'o') {
      setScenario(MockScenario::OUT_OF_RANGE);
      return;
    }

    // ---- Shared commands -------------------------------------------------
    if (cmd == 'b') { blankSensor(); return; }

    if (cmd == '?') {
      Serial.print("[MOCK] Scenario:   "); Serial.println(_seed().label);
      Serial.print("[MOCK] Absorbance: "); Serial.println(_compute_absorbance(), 4);
      return;
    }

    // Toggle menu/polling when in menu-driven mode
    if (cmd == 'p' && _app_state == APP_MENU_DRIVEN) {
      setAppState(APP_POLLING);
    }
  }

  // ---- Display (menu-driven mode) -----------------------------------------

  void _display_measure() {
    float value = getMeasurementValue();
    Serial.print("[MOCK] ");
    Serial.print(_measurement_name);
    Serial.print(": ");
    if (value < 0.0f) {
      const MockSeed& s = _seed();
      bool overflow = (s.tcs_result == SENSOR_OVERFLOW ||
                       s.bh1750_result == SENSOR_OVERFLOW);
      Serial.print(overflow ? "OVERFLOW" : "OUT OF RANGE / ERROR");
    } else {
      Serial.print(value, 4);
    }
    Serial.print("  [scenario:");
    Serial.print(_seed().label);
    Serial.print("]");
    Serial.println();
  }

  void _print_polling_help() {
    Serial.println("[MOCK] POLL commands: s=stream  x=stop  Enter=one-shot  b=blank  m=menu  ?=help");
    Serial.print  ("[MOCK] POLL state   : ");
    switch (_polling_state) {
      case POLLING_IDLE:    Serial.println("IDLE");    break;
      case POLLING_STREAM:  Serial.println("STREAM");  break;
      case POLLING_ONESHOT: Serial.println("ONESHOT"); break;
    }
    Serial.println("[MOCK] Scenario commands: 0-9 select, o=OUT_OF_RANGE, ?=scenario info");
  }

  void _print_scenario_help() {
    Serial.println("[MOCK] Runtime commands (Serial, no line ending):");
    Serial.println("[MOCK]   0-9  select scenario by number");
    Serial.println("[MOCK]   o    OUT_OF_RANGE scenario");
    Serial.println("[MOCK]   b    perform blank");
    Serial.println("[MOCK]   ?    print current scenario + absorbance");
    Serial.println("[MOCK]   m    toggle menu / polling display");
    Serial.println("[MOCK] Scenarios:");
    for (uint8_t i = 0; i < (uint8_t)MockScenario::_COUNT; i++) {
      Serial.print("[MOCK]   ");
      if (i < 10) Serial.print(i); else Serial.print('o');
      Serial.print("  ");
      Serial.println(MOCK_SEEDS[i].label);
    }
  }
};

// Static member definitions
const String MockColorimeter::ABSORBANCE_STR    = "Absorbance";
const String MockColorimeter::TRANSMITTANCE_STR = "Transmittance";
const String MockColorimeter::RAW_TCS_STR       = "Raw TCS34725";
const String MockColorimeter::RAW_BH1750_STR    = "Raw BH1750";
const String MockColorimeter::ABOUT_STR         = "About";

#endif // MOCK_COLORIMETER_H