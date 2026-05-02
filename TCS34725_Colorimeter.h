/*
 * TCS34725_Colorimeter.h
 * Main colorimeter class – orchestrates sensor, calibrations, and measurements.
 *
 * ESP32-S3 changes vs. original Arduino Uno port:
 *   - All try/catch blocks replaced with SensorResult return-code checks.
 *   - cycleGain() and cycleIntegrationTime() delegated to LightSensor so the
 *     4-step gain cycle (1x/4x/16x/60x) is driven by the enum defined there.
 *   - SD card references removed; LittleFS init is in the main sketch.
 *   - Serial baud rate comment updated to 115200.
 *   - blank_value stored as float32; no precision change needed on ESP32-S3
 *     (hardware FPU handles float efficiently).
 */

#ifndef TCS34725_COLORIMETER_H
#define TCS34725_COLORIMETER_H

#include <Arduino.h>
#include <math.h>
#include "LightSensor_TCS34725.h"
#include "Calibrations.h"
#include "Configuration.h"

// ---------------------------------------------------------------------------
// Operating modes
// ---------------------------------------------------------------------------
enum OperatingMode : uint8_t {
  MODE_MEASURE = 0,
  MODE_MENU    = 1,
  MODE_MESSAGE = 2,
  MODE_ABORT   = 3
};

// ---------------------------------------------------------------------------
// Colorimeter
// ---------------------------------------------------------------------------
class Colorimeter {
public:
  // Blanking
  static const uint8_t NUM_BLANK_SAMPLES = 50;
  static const uint32_t BLANK_DT_MS      = 50;   // ms between blank samples
  static const uint32_t LOOP_DT_MS       = 100;  // ms per main loop cycle
  static const uint32_t DEBOUNCE_DT_MS   = 600;  // ms button debounce

  // Measurement name constants
  static const String ABSORBANCE_STR;
  static const String TRANSMITTANCE_STR;
  static const String RAW_SENSOR_STR;
  static const String ABOUT_STR;

  Colorimeter()
    : _mode(MODE_MEASURE),
      _measurement_name(ABSORBANCE_STR),
      _is_blanked(false),
      _blank_value(1.0f),
      _last_button_ms(0),
      _menu_item_pos(0),
      _menu_view_pos(0) {}

  // ---- Lifecycle ----------------------------------------------------------

  // Call once in setup() after LittleFS.begin().  Returns false only on a
  // fatal sensor fault; all other errors set MODE_MESSAGE / MODE_ABORT and
  // still return true so the message loop can display them.
  bool begin() {
    // Initialise light sensor ------------------------------------------------
    if (!_sensor.initialize()) {
      Serial.println("ERROR: TCS34725 not found – check I2C wiring");
      _postMessage("Sensor not found", true);
      _mode = MODE_ABORT;
      return false;
    }

    // Load configuration -----------------------------------------------------
    if (!_config.load()) {
      _postMessage("Config load failed", false);
      _mode = MODE_MESSAGE;
    }

    // Apply sensor settings from configuration
    if (_config.isGainSet())            _sensor.setGain(_config.getGain());
    if (_config.isIntegrationTimeSet()) _sensor.setIntegrationTime(_config.getIntegrationTime());

    // Load calibrations ------------------------------------------------------
    if (!_calibrations.load()) {
      // File absent is non-fatal – device still works for Abs/Trans/Raw
      Serial.println("WARNING: Calibrations not loaded");
    }

    if (_calibrations.hasErrors()) {
      _postMessage("Calibration errors found", false);
      _mode = MODE_MESSAGE;
    }

    // Build menu -------------------------------------------------------------
    _buildMenuItems();

    // Set startup measurement ------------------------------------------------
    String startup = _config.getStartup();
    if (!startup.isEmpty() && _isValidMenuItem(startup)) {
      _measurement_name = startup;
    } else {
      if (!startup.isEmpty()) {
        Serial.print("WARNING: startup measurement \"");
        Serial.print(startup);
        Serial.println("\" not found – defaulting to Absorbance");
        _postMessage("Startup not found", false);
        _mode = MODE_MESSAGE;
      }
      _measurement_name = ABSORBANCE_STR;
    }

    // Preliminary blank (set_blanked = false so the UI shows "not blanked") --
    blankSensor(false);

    return true;
  }

  // Call repeatedly from loop()
  void update() {
    _handleSerial();
    _updateDisplay();
  }

  // ---- Public measurement API ---------------------------------------------

  // Read raw clear-channel value.
  // Writes result to `out`.  Returns SENSOR_OK, SENSOR_OVERFLOW, or SENSOR_IO_ERROR.
  SensorResult getRawValue(uint16_t& out) {
    return _sensor.getValue(out);
  }

  // Returns transmittance [0,1], or -1 on sensor error / overflow.
  float getTransmittance() {
    uint16_t raw;
    if (_sensor.getValue(raw) != SENSOR_OK) return -1.0f;
    if (_blank_value <= 0.0f) return -1.0f;
    return static_cast<float>(raw) / _blank_value;
  }

  // Returns absorbance (optical density), clamped to >= 0.  Returns -1 on error.
  float getAbsorbance() {
    float t = getTransmittance();
    if (t <= 0.0f) return -1.0f;
    float a = -log10f(t);
    return (a > 0.0f) ? a : 0.0f;
  }

  // Returns the value appropriate for the current measurement selection.
  // -1.0 signals "invalid / out of range / overflow" to the caller.
  float getMeasurementValue() {
    if (_measurement_name == ABSORBANCE_STR)    return getAbsorbance();
    if (_measurement_name == TRANSMITTANCE_STR) return getTransmittance();
    if (_measurement_name == RAW_SENSOR_STR) {
      uint16_t raw;
      SensorResult r = _sensor.getValue(raw);
      return (r == SENSOR_OK) ? static_cast<float>(raw) : -1.0f;
    }

    // Custom calibration
    float a = getAbsorbance();
    if (a < 0.0f) return -1.0f;
    return _calibrations.apply(_measurement_name, a);
  }

  String getMeasurementUnits() {
    if (_measurement_name == ABSORBANCE_STR ||
        _measurement_name == TRANSMITTANCE_STR ||
        _measurement_name == RAW_SENSOR_STR) {
      return "";
    }
    return _calibrations.getUnits(_measurement_name);
  }

  // ---- Blanking -----------------------------------------------------------

  // Collect NUM_BLANK_SAMPLES readings, compute the median, store as reference.
  // set_blanked = false performs the reading without marking the device as
  // blanked (used at startup).
  void blankSensor(bool set_blanked = true) {
    float samples[NUM_BLANK_SAMPLES];

    for (uint8_t i = 0; i < NUM_BLANK_SAMPLES; i++) {
      uint16_t raw;
      SensorResult r = _sensor.getValue(raw);
      // On overflow use max counts so the median calculation is still valid
      samples[i] = (r == SENSOR_OVERFLOW)
                     ? static_cast<float>(_sensor.getMaxCounts())
                     : static_cast<float>(raw);
      delay(BLANK_DT_MS);
    }

    _blank_value = _median(samples, NUM_BLANK_SAMPLES);
    _is_blanked  = set_blanked;

    Serial.print("Blank value: ");
    Serial.println(_blank_value);
  }

  // ---- State accessors ----------------------------------------------------

  OperatingMode  getMode()            const { return _mode; }
  String         getMeasurementName() const { return _measurement_name; }
  bool           getIsBlanked()       const { return _is_blanked; }
  Gain_t         getSensorGain()      const { return _sensor.getGain(); }
  IntegrationTime_t getSensorItime()  const { return _sensor.getIntegrationTime(); }

private:
  LightSensor   _sensor;
  Configuration _config;
  Calibrations  _calibrations;

  OperatingMode _mode;
  String        _measurement_name;
  bool          _is_blanked;
  float         _blank_value;
  uint32_t      _last_button_ms;

  std::vector<String> _menu_items;
  uint8_t             _menu_item_pos;
  uint8_t             _menu_view_pos;
  static const uint8_t ITEMS_PER_SCREEN = 5;

  // Pending message for MODE_MESSAGE display
  String _pending_message;
  bool   _pending_is_abort = false;

  // ---- Menu ---------------------------------------------------------------

  void _buildMenuItems() {
    _menu_items.clear();
    _menu_items.push_back(ABSORBANCE_STR);
    _menu_items.push_back(TRANSMITTANCE_STR);
    _menu_items.push_back(RAW_SENSOR_STR);
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

  // ---- Button / serial input ----------------------------------------------

  void _handleSerial() {
    if (!Serial.available()) return;

    char cmd = Serial.read();
    uint32_t now = millis();
    if ((now - _last_button_ms) < DEBOUNCE_DT_MS) return;
    _last_button_ms = now;

    switch (_mode) {

      case MODE_MEASURE:
        if (cmd == 'b' && _measurement_name != RAW_SENSOR_STR) {
          blankSensor();
        } else if (cmd == 'm') {
          _mode = MODE_MENU;
          _menu_item_pos = 0;
          _menu_view_pos = 0;
        } else if (cmd == 'g' && _measurement_name == RAW_SENSOR_STR) {
          _sensor.cycleGain();
          _is_blanked = false;
        } else if (cmd == 'i' && _measurement_name == RAW_SENSOR_STR) {
          _sensor.cycleIntegrationTime();
          _is_blanked = false;
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
            if (_menu_item_pos >= _menu_view_pos + ITEMS_PER_SCREEN) {
              _menu_view_pos++;
            }
          }
        } else if (cmd == 'r') {
          const String& sel = _menu_items[_menu_item_pos];
          if (sel == ABOUT_STR) {
            _postMessage("Firmware v1.0 / ESP32-S3", false);
            _mode = MODE_MESSAGE;
          } else {
            _measurement_name = sel;
            _mode = MODE_MEASURE;
          }
        }
        break;

      case MODE_MESSAGE:
        // Pop the next queued calibration error, or return to measure
        if (_calibrations.hasErrors()) {
          _postMessage(_calibrations.popError(), false);
        } else {
          _mode = MODE_MEASURE;
        }
        break;

      case MODE_ABORT:
        // No recovery without hardware reset
        break;
    }
  }

  // ---- Display (Serial) ---------------------------------------------------

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
      // Distinguish overflow from out-of-range
      uint16_t probe;
      SensorResult r = _sensor.getValue(probe);
      Serial.print(r == SENSOR_OVERFLOW ? "OVERFLOW" : "OUT OF RANGE");
    } else {
      Serial.print(value, _config.getPrecision());
      if (units.length()) { Serial.print(" "); Serial.print(units); }
    }

    if (_measurement_name == RAW_SENSOR_STR) {
      Serial.print("  [gain: ");
      Serial.print(LightSensor::gainToString(_sensor.getGain()));
      Serial.print("  itime: ");
      Serial.print(LightSensor::integrationTimeToString(_sensor.getIntegrationTime()));
      Serial.print("]");
    } else {
      Serial.print(_is_blanked ? "  [blanked]" : "  [not blanked]");
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

  // ---- Utilities ----------------------------------------------------------

  // Simple insertion-sort median – adequate for 50 samples on ESP32-S3.
  static float _median(float* arr, uint8_t n) {
    for (uint8_t i = 1; i < n; i++) {
      float key = arr[i];
      int8_t j  = static_cast<int8_t>(i) - 1;
      while (j >= 0 && arr[j] > key) {
        arr[j + 1] = arr[j];
        j--;
      }
      arr[j + 1] = key;
    }
    return (n % 2 == 0)
             ? (arr[n / 2 - 1] + arr[n / 2]) / 2.0f
             : arr[n / 2];
  }
};

// Static member definitions
const String Colorimeter::ABSORBANCE_STR    = "Absorbance";
const String Colorimeter::TRANSMITTANCE_STR = "Transmittance";
const String Colorimeter::RAW_SENSOR_STR    = "Raw Sensor";
const String Colorimeter::ABOUT_STR         = "About";

#endif // TCS34725_COLORIMETER_H
