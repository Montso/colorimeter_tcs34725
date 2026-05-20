/*
 * Colorimeter.h
 * Main colorimeter class.
 *
 * Operating modes (AppState):
 *   APP_MENU_DRIVEN – full interactive serial menu; sensors read every cycle.
 *   APP_POLLING     – no automatic display; application code gates reads via
 *                     isPollingActive() / consumeOneShot().  PollingState
 *                     sub-state (IDLE / STREAM / ONESHOT) is driven by
 *                     Serial commands (s / Enter / x).
 *   APP_INTERRUPT   – fully idle.  update() only services blank + mode-switch
 *                     Serial commands.  No sensor reads occur in update().
 *                     Application code calls getters on its own schedule
 *                     (timer ISR, hardware trigger, test harness, etc.).
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
#include "LEDController.h"

enum OperatingMode : uint8_t {
  MODE_MEASURE     = 0,
  MODE_MENU        = 1,
  MODE_MESSAGE     = 2,
  MODE_ABORT       = 3,
  MODE_LED_CONTROL = 4   // LED brightness / on-off adjustment
};

// AppState controls what update() does each loop cycle.
//
//   APP_MENU_DRIVEN  – full interactive serial menu; sensors read and printed
//                      every cycle inside update().
//
//   APP_POLLING      – no automatic display.  update() processes streaming /
//                      one-shot commands (s / Enter / x); application code
//                      gates reads via isPollingActive() / consumeOneShot().
//                      PollingState sub-state (IDLE / STREAM / ONESHOT) tracks
//                      whether a read should happen on this cycle.
//
//   APP_INTERRUPT    – fully idle.  update() does nothing except service a
//                      minimal set of Serial commands (blank, mode switch).
//                      No sensor reads occur anywhere in update().
//                      Application code calls getters directly and entirely
//                      on its own schedule – the Colorimeter never initiates
//                      a read or produces Serial output between those calls.
//                      This is the right choice when an external trigger
//                      (timer ISR, hardware pin, RTOS task, test harness)
//                      controls measurement timing rather than loop().
enum AppState : uint8_t {
  APP_MENU_DRIVEN  = 0,  // full interactive serial menu
  APP_POLLING      = 1,  // application code gates reads via isPollingActive() / consumeOneShot()
  APP_INTERRUPT    = 2   // fully idle; application code calls getters on its own schedule
};

enum PollingState : uint8_t {
  POLLING_IDLE    = 0,
  POLLING_STREAM  = 1,
  POLLING_ONESHOT = 2
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
  static const String LED_CONTROL_STR; // "LED Control"
  static const String ABOUT_STR;

  Colorimeter()
    : _mode(MODE_MEASURE),
      _app_state(APP_MENU_DRIVEN),
      _polling_state(POLLING_IDLE),
      _measurement_name(ABSORBANCE_STR),
      _is_blanked(false),
      _blank_value(1.0f),
      _last_button_ms(0),
      _menu_item_pos(0),
      _menu_view_pos(0),
      _primary_sensor(SENSOR_TYPE_TCS34725),
      _tcs_ok(false),
      _bh1750_ok(false),
      _pending_is_abort(false),
      _led_selected(0) {}

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

    // LEDs – initialise independent of sensor / calibration state
    _leds.begin();

    // Preliminary blank (set_blanked = false → UI shows "not blanked")
    blankSensor(false);
    return true;
  }

  void update() {
    switch (_app_state) {
      case APP_INTERRUPT:
        _handleInterruptSerial();   // sensors never touched here
        break;
      case APP_POLLING:
        _handlePollingSerial();     // gates reads via isPollingActive() / consumeOneShot()
        break;
      default:                      // APP_MENU_DRIVEN
        _handleSerial();
        _updateDisplay();
        break;
    }
  }

  // ---- AppState / polling control -----------------------------------------

  void setAppState(AppState s) {
    _app_state = s;
    switch (s) {
      case APP_POLLING:
        _polling_state = POLLING_IDLE;
        _printPollingHelp();
        break;
      case APP_INTERRUPT:
        Serial.println("INT: idle – call getters directly to read sensors");
        Serial.println("INT: commands: b=blank  m=menu  p=polling  ?=help");
        break;
      default:
        break;
    }
  }
  AppState getAppState() const { return _app_state; }

  // True while POLLING_STREAM – application code should read and print.
  bool isPollingActive() const {
    return _app_state == APP_POLLING && _polling_state == POLLING_STREAM;
  }

  // Returns true exactly once per one-shot request, then resets to IDLE.
  // Call at the top of the polling block in loop():
  //   if (colorimeter.isPollingActive() || colorimeter.consumeOneShot()) { … }
  bool consumeOneShot() {
    if (_app_state == APP_POLLING && _polling_state == POLLING_ONESHOT) {
      _polling_state = POLLING_IDLE;
      return true;
    }
    return false;
  }

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

  // Direct access to the LED controller.  Fully independent of operating mode.
  LEDController&       getLEDs()       { return _leds; }
  const LEDController& getLEDs() const { return _leds; }

private:
  LightSensor       _tcs;
  LightSensorBH1750 _bh1750;
  Configuration     _config;
  Calibrations      _calibrations;

  OperatingMode _mode;
  AppState      _app_state;
  PollingState  _polling_state;
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

  LEDController _leds;          // independent of all measurement state
  uint8_t       _led_selected;  // index of LED currently being adjusted (0 or 1)

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
    _menu_items.push_back(LED_CONTROL_STR);
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

  // ---- Serial input (interrupt mode) --------------------------------------
  // update() routes here when _app_state == APP_INTERRUPT.
  //
  // This handler does the absolute minimum: blank, and mode switches.
  // It never reads a sensor and never calls any display function.
  // All measurement output is the exclusive responsibility of application code.
  void _handleInterruptSerial() {
    if (!Serial.available()) return;

    char cmd = Serial.read();
    uint32_t now = millis();
    if ((now - _last_button_ms) < 200) return;
    _last_button_ms = now;

    if (cmd == 'b') {
      blankSensor();                        // blanking is always permitted

    } else if (cmd == 'm') {
      _app_state = APP_MENU_DRIVEN;
      _mode      = MODE_MEASURE;
      Serial.println("Switched to menu-driven mode");

    } else if (cmd == 'p') {
      setAppState(APP_POLLING);

    } else if (cmd == '?') {
      Serial.println("INT: idle – call getters directly to read sensors");
      Serial.println("INT: commands: b=blank  m=menu  p=polling  ?=help");
    }
  }

  // ---- Serial input (polling mode) ----------------------------------------
  // update() routes here when _app_state == APP_POLLING.
  // Sensor reads and printing are NOT done here; they are gated in loop()
  // via isPollingActive() / consumeOneShot().
  void _handlePollingSerial() {
    if (!Serial.available()) return;

    char cmd = Serial.read();
    // Light debounce – shorter than menu mode since single keystrokes matter
    uint32_t now = millis();
    if ((now - _last_button_ms) < 200) return;
    _last_button_ms = now;

    if (cmd == 's') {
      _polling_state = POLLING_STREAM;
      Serial.println("POLL: streaming ON  ('x' to stop, Enter for one-shot)");

    } else if (cmd == 'x' || cmd == 'q') {
      _polling_state = POLLING_IDLE;
      Serial.println("POLL: idle  ('s' to stream, Enter for one-shot)");

    } else if (cmd == '\n' || cmd == '\r') {
      // One-shot: trigger a single read regardless of current stream state
      _polling_state = POLLING_ONESHOT;

    } else if (cmd == 'b') {
      blankSensor();

    } else if (cmd == 'm') {
      // Switch back to full menu-driven mode
      _app_state     = APP_MENU_DRIVEN;
      _polling_state = POLLING_IDLE;
      _mode          = MODE_MEASURE;
      Serial.println("Switched to menu-driven mode");

    } else if (cmd == '?') {
      _printPollingHelp();
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
          setAppState(APP_POLLING);
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
            // Check for LED Control entry
            if (sel == LED_CONTROL_STR) {
              _led_selected = 0;          // default focus to LED 1
              _mode         = MODE_LED_CONTROL;
            } else {
              _measurement_name = sel;
              _mode = MODE_MEASURE;
            }
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

      case MODE_LED_CONTROL:
        if      (cmd == '1')             _led_selected = 0;
        else if (cmd == '2')             _led_selected = 1;
        else if (cmd == '+' || cmd == 'u') _leds.stepUp(_led_selected);
        else if (cmd == '-' || cmd == 'd') _leds.stepDown(_led_selected);
        else if (cmd == 't')             _leds.toggle(_led_selected);
        else if (cmd == 'a')             _leds.allOff();
        else if (cmd == 'm' || cmd == 'r') _mode = MODE_MENU;
        break;
    }
  }

  // ---- Display ------------------------------------------------------------

  void _updateDisplay() {
    switch (_mode) {
      case MODE_MEASURE:     _displayMeasure(); break;
      case MODE_MENU:        _displayMenu();    break;
      case MODE_MESSAGE:     _displayMessage(); break;
      case MODE_ABORT:       _displayAbort();   break;
      case MODE_LED_CONTROL: _displayLED();     break;
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

  void _displayLED() {
    Serial.println("\n=== LED CONTROL ===");
    for (uint8_t i = 0; i < LEDController::NUM_LEDS; i++) {
      Serial.print(i == _led_selected ? " > " : "   ");
      Serial.print(i + 1);
      Serial.print("  ");
      Serial.print(_leds.bar(i));
      Serial.print("  ");
      // Right-align percentage (3 chars)
      uint8_t pct = _leds.getPercent(i);
      if (pct < 100) Serial.print(' ');
      if (pct <  10) Serial.print(' ');
      Serial.print(pct);
      Serial.print("%  ");
      Serial.print(_leds.isOn(i) ? "ON " : "OFF");
      Serial.print("  ");
      Serial.print(LEDController::name(i));
      Serial.print("  GPIO");
      Serial.println(LEDController::pin(i));
    }
    Serial.println("   1/2=select  +/-=brightness  t=toggle  a=all off  m=back");
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

  void _printPollingHelp() {
    Serial.println("POLL commands: s=stream  x=stop  Enter=one-shot  b=blank  m=menu  ?=help");
    Serial.print  ("POLL state   : ");
    switch (_polling_state) {
      case POLLING_IDLE:    Serial.println("IDLE");    break;
      case POLLING_STREAM:  Serial.println("STREAM");  break;
      case POLLING_ONESHOT: Serial.println("ONESHOT"); break;
    }
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
const String Colorimeter::LED_CONTROL_STR   = "LED Control";
const String Colorimeter::ABOUT_STR         = "About";

#endif // COLORIMETER_H