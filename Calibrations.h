/*
 * Calibrations.h
 * Loads and applies polynomial calibration curves from calibrations.json
 *
 * ESP32-S3 changes vs. original Arduino Uno port:
 *   - LittleFS replaces SD card.  Files live in the ESP32-S3 internal flash
 *     partition and are uploaded once via the Arduino LittleFS uploader or
 *     esptool.  No SPI wiring required.
 *   - JSON buffer increased to 16 384 bytes (was 4 096).  ESP32-S3 has
 *     512 KB SRAM so this is trivial; it allows large calibration libraries
 *     with no risk of truncation.
 *   - CalibrationsError class removed.  Error state is now carried internally
 *     and surfaced through hasErrors() / popError(), matching the Python
 *     original's design without requiring C++ exceptions.
 */

#ifndef CALIBRATIONS_H
#define CALIBRATIONS_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <map>
#include <vector>

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

struct PolynomialCoeff {
  float   coeff[10];  // up to 10th-order polynomial
  uint8_t size;
};

struct CalibrationData {
  String         name;
  String         units;
  String         fit_type;
  PolynomialCoeff fit_coef;
  String         led;
  float          range_min;
  float          range_max;
};

// ---------------------------------------------------------------------------
// Calibrations
// ---------------------------------------------------------------------------
class Calibrations {
public:
  // File must exist at the root of the LittleFS partition
  static constexpr const char* FILE_PATH = "/calibrations.json";

  // 16 KB – comfortably handles large calibration libraries on ESP32-S3.
  // Reduce to 4096 if porting back to a memory-constrained target.
  static const size_t JSON_BUFFER_SIZE = 16384;

  Calibrations() {}

  // ---- Loading ------------------------------------------------------------

  // Parse calibrations.json from LittleFS.
  // Returns true if the file was read and parsed without a fatal error.
  // Individual calibrations that fail validation are skipped and their errors
  // are queued in the error dict (retrievable via hasErrors() / popError()).
  bool load() {
    _calibrations.clear();
    _errors.clear();

    if (!LittleFS.exists(FILE_PATH)) {
      Serial.println("WARNING: /calibrations.json not found in LittleFS");
      return false;
    }

    File file = LittleFS.open(FILE_PATH, "r");
    if (!file) {
      Serial.println("ERROR: Unable to open /calibrations.json");
      return false;
    }

    DynamicJsonDocument doc(JSON_BUFFER_SIZE);
    DeserializationError err = deserializeJson(doc, file);
    file.close();

    if (err) {
      Serial.print("ERROR: calibrations.json parse failed: ");
      Serial.println(err.f_str());
      return false;
    }

    if (!doc.is<JsonObject>()) {
      Serial.println("ERROR: calibrations.json root must be a JSON object");
      return false;
    }

    for (JsonPair p : doc.as<JsonObject>()) {
      _parseCalibration(p.key().c_str(), p.value().as<JsonObject>());
    }

    return true;
  }

  // ---- Error handling -----------------------------------------------------

  bool hasErrors() const { return !_errors.empty(); }

  // Pop and return the next error message, or "" if none remain.
  // Errors are returned one at a time so the caller can display them
  // sequentially (matching the original CircuitPython behaviour).
  String popError() {
    if (_errors.empty()) return "";

    auto it = _errors.begin();
    if (!it->second.empty()) {
      String msg = it->second.back();
      it->second.pop_back();
      if (it->second.empty()) _errors.erase(it);
      return msg;
    }
    return "";
  }

  // ---- Accessors ----------------------------------------------------------

  const CalibrationData* getCalibration(const String& name) const {
    auto it = _calibrations.find(name);
    return (it != _calibrations.end()) ? &it->second : nullptr;
  }

  const std::map<String, CalibrationData>& getAllCalibrations() const {
    return _calibrations;
  }

  String getUnits(const String& name) const {
    const CalibrationData* c = getCalibration(name);
    return c ? c->units : "";
  }

  String getLED(const String& name) const {
    const CalibrationData* c = getCalibration(name);
    return c ? c->led : "";
  }

  // ---- Calibration application --------------------------------------------

  // Apply a named calibration to an absorbance value.
  // Returns the calibrated value, or -1.0 if:
  //   - the calibration name is not found
  //   - absorbance is outside the calibration's valid range
  float apply(const String& name, float absorbance) {
    const CalibrationData* cal = getCalibration(name);
    if (!cal) return -1.0f;

    if (absorbance < cal->range_min || absorbance > cal->range_max) {
      return -1.0f;  // out of range
    }

    // Horner's method would be marginally faster but this is clearer and
    // called infrequently enough that it does not matter.
    float result = 0.0f;
    float power  = 1.0f;
    for (uint8_t i = 0; i < cal->fit_coef.size; i++) {
      result += cal->fit_coef.coeff[i] * power;
      power  *= absorbance;
    }
    return result;
  }

  void clear() {
    _calibrations.clear();
    _errors.clear();
  }

private:
  std::map<String, CalibrationData>          _calibrations;
  std::map<String, std::vector<String>>      _errors;

  // Parse a single calibration entry from the JSON object.
  // On validation failure the entry is skipped and errors are queued.
  void _parseCalibration(const String& name, JsonObject cal) {
    CalibrationData data;
    data.name      = name;
    data.range_min = 0.0f;
    data.range_max = 0.0f;
    bool has_error = false;

    // -- fit_type ------------------------------------------------------------
    if (!cal.containsKey("fit_type")) {
      _addError(name, "missing fit_type");
      has_error = true;
    } else {
      data.fit_type = cal["fit_type"].as<String>();
      if (data.fit_type != "linear" && data.fit_type != "polynomial") {
        _addError(name, "unknown fit_type: " + data.fit_type);
        has_error = true;
      }
    }

    // -- fit_coef ------------------------------------------------------------
    if (!cal.containsKey("fit_coef")) {
      _addError(name, "missing fit_coef");
      has_error = true;
    } else {
      JsonArray arr = cal["fit_coef"];
      data.fit_coef.size = static_cast<uint8_t>(arr.size());

      if (data.fit_coef.size == 0) {
        _addError(name, "fit_coef array is empty");
        has_error = true;
      } else if (data.fit_coef.size > 10) {
        _addError(name, "fit_coef has more than 10 coefficients");
        has_error = true;
      } else {
        for (uint8_t i = 0; i < data.fit_coef.size; i++) {
          if (!arr[i].is<float>()) {
            _addError(name, "fit_coef contains non-numeric value");
            has_error = true;
            break;
          }
          data.fit_coef.coeff[i] = arr[i].as<float>();
        }
        if (!has_error && data.fit_type == "linear" && data.fit_coef.size > 2) {
          _addError(name, "too many fit_coef for linear fit (max 2)");
          has_error = true;
        }
      }
    }

    // -- range ---------------------------------------------------------------
    if (!cal.containsKey("range")) {
      if (data.fit_type != "linear") {
        _addError(name, "range data missing");
        has_error = true;
      }
      // Linear fits without a range are valid; leave range at defaults (0,0)
      // so the caller can still apply the fit without a bounds check.
      data.range_min = -1e6f;
      data.range_max =  1e6f;
    } else {
      JsonObject range = cal["range"];

      if (!range.containsKey("min")) {
        _addError(name, "range.min missing");
        has_error = true;
      } else {
        data.range_min = range["min"].as<float>();
      }

      if (!range.containsKey("max")) {
        _addError(name, "range.max missing");
        has_error = true;
      } else {
        data.range_max = range["max"].as<float>();
      }

      if (!has_error && data.range_min >= data.range_max) {
        _addError(name, "range.min >= range.max");
        has_error = true;
      }
    }

    // -- optional fields -----------------------------------------------------
    if (cal.containsKey("units")) data.units = cal["units"].as<String>();
    if (cal.containsKey("led"))   data.led   = cal["led"].as<String>();

    // Only store the calibration if it passed all checks
    if (!has_error) {
      _calibrations[name] = data;
    }
  }

  void _addError(const String& name, const String& msg) {
    Serial.print("Calibration error [");
    Serial.print(name);
    Serial.print("]: ");
    Serial.println(msg);
    _errors[name].push_back(msg);
  }
};

#endif // CALIBRATIONS_H
