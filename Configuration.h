/*
 * Configuration.h
 * Loads device settings from /configuration.json stored in LittleFS.
 *
 * Changes in this revision:
 *   - "sensor" key renamed semantically to "primary_sensor".
 *     Both sensors are always initialised if present; this key only controls
 *     which one drives Absorbance and Transmittance calculations.
 *     "Raw TCS34725" and "Raw BH1750" are always available as separate
 *     menu items regardless of this setting.
 */

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "LightSensor_TCS34725.h"
#include "LightSensor_BH1750.h"

enum SensorType : uint8_t {
  SENSOR_TYPE_TCS34725 = 0,
  SENSOR_TYPE_BH1750   = 1
};

class Configuration {
public:
  static constexpr const char* FILE_PATH = "/configuration.json";
  static const size_t JSON_BUFFER_SIZE   = 4096;

  static const SensorType        DEFAULT_PRIMARY_SENSOR = SENSOR_TYPE_TCS34725;
  static const Gain_t            DEFAULT_GAIN            = GAIN_16X;
  static const IntegrationTime_t DEFAULT_ITIME           = INTEGRATIONTIME_500MS;
  static const uint8_t           DEFAULT_PRECISION       = 2;
  static const BH1750MTregPreset DEFAULT_BH1750_MTREG    = MTREG_DEFAULT;
  static const BH1750ModePreset  DEFAULT_BH1750_MODE     = BH1750_MODE_HIGH_RES;

  Configuration()
    : _primary_sensor(DEFAULT_PRIMARY_SENSOR),
      _gain(DEFAULT_GAIN),
      _itime(DEFAULT_ITIME),
      _precision(DEFAULT_PRECISION),
      _bh1750_mtreg(DEFAULT_BH1750_MTREG),
      _bh1750_mode(DEFAULT_BH1750_MODE),
      _gain_set(false),
      _itime_set(false),
      _bh1750_mtreg_set(false),
      _bh1750_mode_set(false) {}

  bool load() {
    if (!LittleFS.exists(FILE_PATH)) {
      Serial.println("WARNING: /configuration.json not found, using defaults");
      return true;
    }

    File file = LittleFS.open(FILE_PATH, "r");
    if (!file) {
      Serial.println("ERROR: Unable to open /configuration.json");
      return false;
    }

    DynamicJsonDocument doc(JSON_BUFFER_SIZE);
    DeserializationError err = deserializeJson(doc, file);
    file.close();

    if (err) {
      Serial.print("ERROR: configuration.json parse failed: ");
      Serial.println(err.f_str());
      return false;
    }

    if (!doc.is<JsonObject>()) {
      Serial.println("ERROR: configuration.json root must be a JSON object");
      return false;
    }

    JsonObject obj = doc.as<JsonObject>();

    // -- primary_sensor (also accepts legacy key "sensor") -------------------
    // Controls which sensor is used for Absorbance / Transmittance.
    // Both sensors are always initialised; this does not disable either one.
    const char* sensor_key = obj.containsKey("primary_sensor")
                               ? "primary_sensor" : "sensor";
    if (obj.containsKey(sensor_key)) {
      String s = obj[sensor_key].as<String>();
      if      (s == "bh1750")   _primary_sensor = SENSOR_TYPE_BH1750;
      else if (s == "tcs34725") _primary_sensor = SENSOR_TYPE_TCS34725;
      else {
        Serial.print("WARNING: unknown primary_sensor \"");
        Serial.print(s);
        Serial.println("\" – defaulting to tcs34725");
      }
    }

    // -- TCS34725: gain ------------------------------------------------------
    if (obj.containsKey("gain")) {
      Gain_t g;
      if (_parseGainString(obj["gain"].as<String>(), g)) {
        _gain = g; _gain_set = true;
      } else {
        Serial.print("WARNING: unknown gain \"");
        Serial.print(obj["gain"].as<String>()); Serial.println("\"");
      }
    }

    // -- TCS34725: integration_time ------------------------------------------
    if (obj.containsKey("integration_time")) {
      IntegrationTime_t t;
      if (_parseItimeString(obj["integration_time"].as<String>(), t)) {
        _itime = t; _itime_set = true;
      } else {
        Serial.print("WARNING: unknown integration_time \"");
        Serial.print(obj["integration_time"].as<String>()); Serial.println("\"");
      }
    }

    // -- BH1750: sensitivity (MTreg preset) ----------------------------------
    if (obj.containsKey("bh1750_sensitivity")) {
      BH1750MTregPreset p;
      if (_parseMTregString(obj["bh1750_sensitivity"].as<String>(), p)) {
        _bh1750_mtreg = p; _bh1750_mtreg_set = true;
      } else {
        Serial.print("WARNING: unknown bh1750_sensitivity \"");
        Serial.print(obj["bh1750_sensitivity"].as<String>()); Serial.println("\"");
      }
    }

    // -- BH1750: mode --------------------------------------------------------
    if (obj.containsKey("bh1750_mode")) {
      BH1750ModePreset m;
      if (_parseBH1750ModeString(obj["bh1750_mode"].as<String>(), m)) {
        _bh1750_mode = m; _bh1750_mode_set = true;
      } else {
        Serial.print("WARNING: unknown bh1750_mode \"");
        Serial.print(obj["bh1750_mode"].as<String>()); Serial.println("\"");
      }
    }

    // -- precision -----------------------------------------------------------
    if (obj.containsKey("precision")) {
      uint8_t p = obj["precision"].as<uint8_t>();
      if (p >= 2 && p <= 4) _precision = p;
      else Serial.println("WARNING: precision must be 2-4, using default");
    }

    // -- startup -------------------------------------------------------------
    if (obj.containsKey("startup")) _startup = obj["startup"].as<String>();

    return true;
  }

  // Accessors
  SensorType        getPrimarySensor()     const { return _primary_sensor; }
  Gain_t            getGain()              const { return _gain; }
  bool              isGainSet()            const { return _gain_set; }
  IntegrationTime_t getIntegrationTime()   const { return _itime; }
  bool              isIntegrationTimeSet() const { return _itime_set; }
  uint8_t           getPrecision()         const { return _precision; }
  String            getStartup()           const { return _startup; }
  BH1750MTregPreset getBH1750MTreg()       const { return _bh1750_mtreg; }
  bool              isBH1750MTregSet()     const { return _bh1750_mtreg_set; }
  BH1750ModePreset  getBH1750Mode()        const { return _bh1750_mode; }
  bool              isBH1750ModeSet()      const { return _bh1750_mode_set; }

private:
  SensorType        _primary_sensor;
  Gain_t            _gain;
  IntegrationTime_t _itime;
  uint8_t           _precision;
  String            _startup;
  BH1750MTregPreset _bh1750_mtreg;
  BH1750ModePreset  _bh1750_mode;
  bool              _gain_set;
  bool              _itime_set;
  bool              _bh1750_mtreg_set;
  bool              _bh1750_mode_set;

  static bool _parseGainString(const String& s, Gain_t& out) {
    if (s == "1x"  || s == "low")             { out = GAIN_1X;  return true; }
    if (s == "4x")                             { out = GAIN_4X;  return true; }
    if (s == "16x" || s == "med")             { out = GAIN_16X; return true; }
    if (s == "60x" || s == "high"|| s=="max") { out = GAIN_60X; return true; }
    return false;
  }

  static bool _parseItimeString(const String& s, IntegrationTime_t& out) {
    if (s == "100ms") { out = INTEGRATIONTIME_100MS; return true; }
    if (s == "200ms") { out = INTEGRATIONTIME_200MS; return true; }
    if (s == "300ms") { out = INTEGRATIONTIME_300MS; return true; }
    if (s == "400ms") { out = INTEGRATIONTIME_400MS; return true; }
    if (s == "500ms") { out = INTEGRATIONTIME_500MS; return true; }
    if (s == "600ms") { out = INTEGRATIONTIME_600MS; return true; }
    return false;
  }

  static bool _parseMTregString(const String& s, BH1750MTregPreset& out) {
    if (s == "low")     { out = MTREG_LOW;     return true; }
    if (s == "default") { out = MTREG_DEFAULT; return true; }
    if (s == "high")    { out = MTREG_HIGH;    return true; }
    if (s == "max")     { out = MTREG_MAX;     return true; }
    return false;
  }

  static bool _parseBH1750ModeString(const String& s, BH1750ModePreset& out) {
    if (s == "high_res")   { out = BH1750_MODE_HIGH_RES;   return true; }
    if (s == "high_res_2") { out = BH1750_MODE_HIGH_RES_2; return true; }
    if (s == "low_res")    { out = BH1750_MODE_LOW_RES;    return true; }
    return false;
  }
};

#endif // CONFIGURATION_H
