/*
 * Colorimeter.h
 * Main colorimeter class.
 *
 * Changes in this revision:
 *   - Both TCS34725 and BH1750 are always initialised if present on the bus.
 *     _tcs_ok / _bh1750_ok flags track which sensors responded at startup.
 *   - Generic "Raw Sensor" replaced by "Raw TCS34725" and "Raw BH1750" as
 *     distinct menu items, added only if the corresponding sensor is present.
 *   - Absorbance and Transmittance use the "primary_sensor" from config.
 *     If that sensor failed to initialise, the other is used as fallback
 *     rather than aborting entirely.
 *   - 'g' / 'i' commands in raw mode act on the sensor matching the currently
 *     selected raw measurement, not a global active-sensor variable.
 *   - blankSensor() blanks the primary sensor (the one used for Abs/Trans).
 */

#ifndef COLORIMETER_H
#define COLORIMETER_H

#include <Arduino.h>
#include <math.h>
#include <vector>
#include "SensorCommon.h"
#include "LightSensor_TCS34725.h"
#include "LightSensor_BH1750.h"
#include "Calibrations.h"
#include "Configuration.h"

enum OperatingMode : uint8_t {
  MODE_MEASURE = 0,
  MODE_MENU    = 1,
  MODE_MESSAGE = 2,
  MODE_ABORT   = 3
};

// Top-level application state.
//   APP_MENU_DRIVEN – full interactive serial menu (default).
//   APP_POLLING     – no menu output; caller reads values via public getters.
//                     Only 'b' (blank) and 'm' (return to menu) are handled.
enum AppState : uint8_t {
  APP_MENU_DRIVEN = 0,
  APP_POLLING     = 1
};

class Colorimeter {
public:
  static const uint8_t  NUM_BLANK_SAMPLES = 50;
  static const uint32_t BLANK_DT_MS       = 50;
  static const uint32_t LOOP_DT_MS        = 100;
  static const uint32_t DEBOUNCE_DT_MS    = 600;

  static const String ABSORBANCE_STR;
  static const String TRANSMITTANCE_STR;
  static const String RAW_TCS_STR;     // "Raw TCS34725"
  static const String RAW_BH1750_STR;  // "Raw BH1750"
  static const String ABOUT_STR;

  Colorimeter()
    : _mode(MODE_MEASURE),
      _app_state(APP_MENU_DRIVEN),
      _measurement_name(ABSORBANCE_STR),
      _is_blanked(false),
      _blank_value(1.0f),
      _last_button_ms(0),
      _menu_item_pos(0),
      _menu_view_pos(0),
      _primary_sensor(SENSOR_TYPE_TCS34725),
      _tcs_ok(false),
      _bh1750_ok(false),
      _pending_is_abort(false) {}

  // ---- Lifecycle ----------------------------------------------------------

  bool begin() {
    // Load config first (determines primary sensor and initial settings)
    if (!_config.load()) {
      _postMessage("Config load failed", false);
      _mode = MODE_MESSAGE;
    }
    _primary_sensor = _config.getPrimarySensor();

    // ---- Initialise TCS34725 -----------------------------------------------
    _tcs_ok = _tcs.initialize();
    if (_tcs_ok) {
      if (_config.isGainSet())            _tcs.setGain(_config.getGain());
      if (_config.isIntegrationTimeSet()) _tcs.setIntegrationTime(_config.getIntegrationTime());
      Serial.println("TCS34725: OK");
    } else {
      Serial.println("TCS34725: not found");
    }

    // ---- Initialise BH1750 -------------------------------------------------
    _bh1750_ok = _bh1750.initialize();
    if (_bh1750_ok) {
      if (_config.isBH1750MTregSet()) _bh1750.setMTreg(_config.getBH1750MTreg());
      if (_config.isBH1750ModeSet())  _bh1750.setMode(_config.getBH1750Mode());
      Serial.println("BH1750:   OK");
    } else {
      Serial.println("BH1750:   not found");
    }

    // ---- Abort only if the primary sensor is missing -----------------------
    // If the primary is absent but the other sensor is present, fall back
    // gracefully rather than aborting.
    if (!_tcs_ok && !_bh1750_ok) {
      _postMessage("No sensors found", true);
      _mode = MODE_ABORT;
      return false;
    }

    if (_primary_sensor == SENSOR_TYPE_TCS34725 && !_tcs_ok) {
      Serial.println("WARNING: primary sensor TCS34725 absent, falling back to BH1750");
      _primary_sensor = SENSOR_TYPE_BH1750;
    } else if (_primary_sensor == SENSOR_TYPE_BH1750 && !_bh1750_ok) {
      Serial.println("WARNING: primary sensor BH1750 absent, falling back to TCS34725");
      _primary_sensor = SENSOR_TYPE_TCS34725;
    }

    // ---- Calibrations ------------------------------------------------------
    if (!_calibrations.load()) {
      Serial.println("WARNING: Calibrations not loaded");
    }
    if (_calibrations.hasErrors()) {
      _postMessage("Calibration errors found", false);
      _mode = MODE_MESSAGE;
    }

    // ---- Menu and startup measurement --------------------------------------
    _buildMenuItems();

    String startup = _config.getStartup();
    if (!startup.isEmpty() && _isValidMenuItem(startup)) {
      _measurement_name = startup;
    } else {
      if (!startup.isEmpty()) {
        Serial.print("WARNING: startup \"");
        Serial.print(startup);
        Serial.println("\" not found – defaulting to Absorbance");
      }
      _measurement_name = ABSORBANCE_STR;
    }

    // Preliminary blank (set_blanked = false → UI shows "not blanked")
    blankSensor(false);
    return true;
  }

  void update() {
    if (_app_state == APP_POLLING) {
      _handlePollingSerial();
    } else {
      _handleSerial();
      _updateDisplay();
    }
  }

  // ---- App-state control --------------------------------------------------

  void     setAppState(AppState s) { _app_state = s; }
  AppState getAppState()     const { return _app_state; }

  // ---- Measurement API ----------------------------------------------------

  // Read the primary sensor (used for Abs/Trans).
  SensorResult getPrimaryRaw(float& out) {
    return (_primary_sensor == SENSOR_TYPE_BH1750)
             ? _bh1750.getValue(out)
             : _tcs.getValue(out);
  }

  // Read a specific sensor by name – used internally for the two raw items.
  SensorResult getTCSRaw(float& out)    { return _tcs.getValue(out); }
  SensorResult getBH1750Raw(float& out) { return _bh1750.getValue(out); }

  float getTransmittance() {
    float raw;
    if (getPrimaryRaw(raw) != SENSOR_OK) return -1.0f;
    if (_blank_value <= 0.0f)            return -1.0f;
    return raw / _blank_value;
  }

  float getAbsorbance() {
    float t = getTransmittance();
    if (t <= 0.0f) return -1.0f;
    float a = -log10f(t);
    return (a > 0.0f) ? a : 0.0f;
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
    // Custom calibration (uses primary sensor for absorbance)
    float a = getAbsorbance();
    return (a >= 0.0f) ? _calibrations.apply(_measurement_name, a) : -1.0f;
  }

  String getMeasurementUnits() {
    if (_measurement_name == ABSORBANCE_STR    ||
        _measurement_name == TRANSMITTANCE_STR ||
        _measurement_name == RAW_TCS_STR       ||
        _measurement_name == RAW_BH1750_STR)   return "";
    return _calibrations.getUnits(_measurement_name);
  }

  // ---- Blanking -----------------------------------------------------------

  // Blanks the primary sensor only. The raw sensor items are not affected;
  // they display absolute values and don't use the blank reference.
  void blankSensor(bool set_blanked = true) {
    float samples[NUM_BLANK_SAMPLES];

    for (uint8_t i = 0; i < NUM_BLANK_SAMPLES; i++) {
      float raw;
      SensorResult r = getPrimaryRaw(raw);
      if (r == SENSOR_OVERFLOW) {
        raw = (_primary_sensor == SENSOR_TYPE_BH1750)
                ? LightSensorBH1750::MAX_LUX
                : static_cast<float>(_tcs.getMaxCounts());
      } else if (r == SENSOR_IO_ERROR) {
        raw = 0.0f;
      }
      samples[i] = raw;
      delay(BLANK_DT_MS);
    }

    _blank_value = _median(samples, NUM_BLANK_SAMPLES);
    _is_blanked  = set_blanked;

    Serial.print("Blank value (");
    Serial.print(_primary_sensor == SENSOR_TYPE_BH1750 ? "BH1750 lux" : "TCS34725 counts");
    Serial.print("): ");
    Serial.println(_blank_value);
  }

  // ---- State accessors ----------------------------------------------------

  OperatingMode getMode()            const { return _mode; }
  String        getMeasurementName() const { return _measurement_name; }
  bool          getIsBlanked()       const { return _is_blanked; }
  bool          isTCSPresent()       const { return _tcs_ok; }
  bool          isBH1750Present()    const { return _bh1750_ok; }
  SensorType    getPrimarySensor()   const { return _primary_sensor; }

private:
  LightSensor       _tcs;
  LightSensorBH1750 _bh1750;
  Configuration     _config;
  Calibrations      _calibrations;

  OperatingMode _mode;
  AppState      _app_state;
  String        _measurement_name;
  bool          _is_blanked;
  float         _blank_value;
  uint32_t      _last_button_ms;

  std::vector<String> _menu_items;
  uint8_t             _menu_item_pos;
  uint8_t             _menu_view_pos;
  static const uint8_t ITEMS_PER_SCREEN = 5;

  SensorType _primary_sensor;
  bool       _tcs_ok;
  bool       _bh1750_ok;

  String _pending_message;
  bool   _pending_is_abort;

  // ---- Menu ---------------------------------------------------------------

  void _buildMenuItems() {
    _menu_items.clear();
    _menu_items.push_back(ABSORBANCE_STR);
    _menu_items.push_back(TRANSMITTANCE_STR);
    // Add raw items only for sensors that actually responded
    if (_tcs_ok)    _menu_items.push_back(RAW_TCS_STR);
    if (_bh1750_ok) _menu_items.push_back(RAW_BH1750_STR);
    for (const auto& kv : _calibrations.getAllCalibrations()) {
      _menu_items.push_back(kv.first);
    }
    _menu_items.push_back(ABOUT_STR);
  }

  bool _isValidMenuItem(const String& name) const {
    for (const auto& item : _menu_items) {
      if (item == name) return true;
    }
    return false;
  }

  // Returns true if the current measurement is either raw sensor view.
  bool _isRawView() const {
    return _measurement_name == RAW_TCS_STR ||
           _measurement_name == RAW_BH1750_STR;
  }

  // ---- Serial input (polling mode) ----------------------------------------

  void _handlePollingSerial() {
    if (!Serial.available()) return;
    char cmd = Serial.read();
    uint32_t now = millis();
    if ((now - _last_button_ms) < DEBOUNCE_DT_MS) return;
    _last_button_ms = now;

    if (cmd == 'b') {
      blankSensor();
    } else if (cmd == 'm') {
      _app_state = APP_MENU_DRIVEN;
      _mode      = MODE_MEASURE;
      Serial.println("Switched to menu-driven mode");
    }
  }

  // ---- Serial input (menu-driven mode) ------------------------------------

  void _handleSerial() {
    if (!Serial.available()) return;

    char cmd = Serial.read();
    uint32_t now = millis();
    if ((now - _last_button_ms) < DEBOUNCE_DT_MS) return;
    _last_button_ms = now;

    switch (_mode) {

      case MODE_MEASURE:
        if (cmd == 'p') {
          _app_state = APP_POLLING;
          Serial.println("Switched to polling mode");
          return;

        } else if (cmd == 'b' && !_isRawView()) {
          blankSensor();

        } else if (cmd == 'm') {
          _mode = MODE_MENU;
          _menu_item_pos = 0;
          _menu_view_pos = 0;

        } else if (cmd == 'g' && _isRawView()) {
          // Cycle sensitivity on the sensor currently being viewed
          if (_measurement_name == RAW_TCS_STR)    _tcs.cycleGain();
          if (_measurement_name == RAW_BH1750_STR) _bh1750.cycleMTreg();

        } else if (cmd == 'i' && _isRawView()) {
          // Cycle integration/mode on the sensor currently being viewed
          if (_measurement_name == RAW_TCS_STR)    _tcs.cycleIntegrationTime();
          if (_measurement_name == RAW_BH1750_STR) _bh1750.cycleMode();
        }
        break;

      case MODE_MENU:
        if (cmd == 'm') {
          _mode = MODE_MEASURE;
        } else if (cmd == 'u') {
          if (_menu_item_pos > 0) {
            _menu_item_pos--;
            if (_menu_item_pos < _menu_view_pos) _menu_view_pos--;
          }
        } else if (cmd == 'd') {
          if (_menu_item_pos < (uint8_t)(_menu_items.size() - 1)) {
            _menu_item_pos++;
            if (_menu_item_pos >= _menu_view_pos + ITEMS_PER_SCREEN)
              _menu_view_pos++;
          }
        } else if (cmd == 'r') {
          const String& sel = _menu_items[_menu_item_pos];
          if (sel == ABOUT_STR) {
            String about = "Firmware v1.0 | primary: ";
            about += (_primary_sensor == SENSOR_TYPE_BH1750) ? "BH1750" : "TCS34725";
            about += _tcs_ok    ? " | TCS:OK"  : " | TCS:--";
            about += _bh1750_ok ? " | BH:OK"   : " | BH:--";
            _postMessage(about, false);
            _mode = MODE_MESSAGE;
          } else {
            _measurement_name = sel;
            _mode = MODE_MEASURE;
          }
        }
        break;

      case MODE_MESSAGE:
        if (_calibrations.hasErrors()) {
          _postMessage(_calibrations.popError(), false);
        } else {
          _mode = MODE_MEASURE;
        }
        break;

      case MODE_ABORT:
        break;
    }
  }

  // ---- Display ------------------------------------------------------------

  void _updateDisplay() {
    switch (_mode) {
      case MODE_MEASURE:  _displayMeasure(); break;
      case MODE_MENU:     _displayMenu();    break;
      case MODE_MESSAGE:  _displayMessage(); break;
      case MODE_ABORT:    _displayAbort();   break;
    }
  }

  void _displayMeasure() {
    float value = getMeasurementValue();
    String units = getMeasurementUnits();

    Serial.print(_measurement_name);
    Serial.print(": ");

    if (value < 0.0f) {
      // Distinguish overflow from out-of-range for the relevant sensor
      float probe;
      SensorResult r;
      if      (_measurement_name == RAW_TCS_STR)    r = getTCSRaw(probe);
      else if (_measurement_name == RAW_BH1750_STR) r = getBH1750Raw(probe);
      else                                           r = getPrimaryRaw(probe);
      Serial.print(r == SENSOR_OVERFLOW ? "OVERFLOW" : "OUT OF RANGE");
    } else {
      Serial.print(value, _config.getPrecision());
      if (units.length()) { Serial.print(" "); Serial.print(units); }
    }

    // Sensor-specific status suffix
    if (_measurement_name == RAW_TCS_STR) {
      Serial.print("  [gain:");
      Serial.print(LightSensor::gainToString(_tcs.getGain()));
      Serial.print(" itime:");
      Serial.print(LightSensor::integrationTimeToString(_tcs.getIntegrationTime()));
      Serial.print("]");
    } else if (_measurement_name == RAW_BH1750_STR) {
      Serial.print("  [sens:");
      Serial.print(LightSensorBH1750::mtregToString(_bh1750.getMTreg()));
      Serial.print(" mode:");
      Serial.print(LightSensorBH1750::modeToString(_bh1750.getMode()));
      Serial.print("]");
    } else {
      // Abs / Trans / calibrated – show blanking status and primary sensor
      Serial.print(_is_blanked ? "  [blanked" : "  [not blanked");
      Serial.print("|primary:");
      Serial.print(_primary_sensor == SENSOR_TYPE_BH1750 ? "BH1750" : "TCS34725");
      Serial.print("]");
    }

    Serial.println();
  }

  void _displayMenu() {
    Serial.println("\n=== MENU ===");
    uint8_t end = min(
      static_cast<uint8_t>(_menu_view_pos + ITEMS_PER_SCREEN),
      static_cast<uint8_t>(_menu_items.size())
    );
    for (uint8_t i = _menu_view_pos; i < end; i++) {
      Serial.print(i == _menu_item_pos ? "> " : "  ");
      Serial.println(_menu_items[i]);
    }
  }

  void _displayMessage() {
    Serial.print(_pending_is_abort ? "ABORT: " : "MESSAGE: ");
    Serial.println(_pending_message);
    if (!_pending_is_abort) Serial.println("(press any key to continue)");
  }

  void _displayAbort() {
    Serial.println("ABORT – press RESET to restart");
  }

  void _postMessage(const String& msg, bool is_abort) {
    _pending_message  = msg;
    _pending_is_abort = is_abort;
  }

  // Insertion-sort median
  static float _median(float* arr, uint8_t n) {
    for (uint8_t i = 1; i < n; i++) {
      float key = arr[i];
      int8_t j  = static_cast<int8_t>(i) - 1;
      while (j >= 0 && arr[j] > key) { arr[j + 1] = arr[j]; j--; }
      arr[j + 1] = key;
    }
    return (n % 2 == 0)
             ? (arr[n / 2 - 1] + arr[n / 2]) / 2.0f
             : arr[n / 2];
  }
};

const String Colorimeter::ABSORBANCE_STR    = "Absorbance";
const String Colorimeter::TRANSMITTANCE_STR = "Transmittance";
const String Colorimeter::RAW_TCS_STR       = "Raw TCS34725";
const String Colorimeter::RAW_BH1750_STR    = "Raw BH1750";
const String Colorimeter::ABOUT_STR         = "About";

#endif // COLORIMETER_H
