/*
 * Configuration.h
 * Loads device settings from configuration.json stored in LittleFS.
 *
 * ESP32-S3 changes vs. original Arduino Uno port:
 *   - LittleFS replaces SD card.
 *   - JSON buffer increased to 4 096 bytes (was 1 024); config files are
 *     small so this is conservative, but there is no reason to be tight.
 *   - GainValue enum aligned with Gain_t in LightSensor_TCS34725.h so
 *     the direct cast used in Colorimeter::begin() is correct for all 4 steps.
 *   - Accepted gain strings updated: "1x" | "4x" | "16x" | "60x"
 *     Legacy strings "low" / "med" / "high" / "max" are also accepted so
 *     existing configuration.json files do not need to be changed.
 *   - ConfigurationError class removed; errors are logged to Serial and
 *     defaults are used, avoiding any dependence on C++ exceptions.
 */

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "LightSensor_TCS34725.h"  // for Gain_t and IntegrationTime_t

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
class Configuration {
public:
  static constexpr const char* FILE_PATH = "/configuration.json";
  static const size_t JSON_BUFFER_SIZE   = 4096;

  // Default values applied when a key is absent or invalid
  static const Gain_t            DEFAULT_GAIN  = GAIN_16X;
  static const IntegrationTime_t DEFAULT_ITIME = INTEGRATIONTIME_500MS;
  static const uint8_t           DEFAULT_PRECISION = 2;

  Configuration()
    : _gain(DEFAULT_GAIN),
      _itime(DEFAULT_ITIME),
      _precision(DEFAULT_PRECISION),
      _gain_set(false),
      _itime_set(false) {}

  // ---- Loading ------------------------------------------------------------

  // Returns true even when the file is absent (defaults are used).
  // Returns false only on a JSON parse error or non-object root.
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

    // -- gain ----------------------------------------------------------------
    if (obj.containsKey("gain")) {
      String s = obj["gain"].as<String>();
      Gain_t g;
      if (_parseGainString(s, g)) {
        _gain     = g;
        _gain_set = true;
      } else {
        Serial.print("WARNING: unknown gain value \"");
        Serial.print(s);
        Serial.println("\", using default");
      }
    }

    // -- integration_time ----------------------------------------------------
    if (obj.containsKey("integration_time")) {
      String s = obj["integration_time"].as<String>();
      IntegrationTime_t t;
      if (_parseItimeString(s, t)) {
        _itime     = t;
        _itime_set = true;
      } else {
        Serial.print("WARNING: unknown integration_time value \"");
        Serial.print(s);
        Serial.println("\", using default");
      }
    }

    // -- precision -----------------------------------------------------------
    if (obj.containsKey("precision")) {
      uint8_t p = obj["precision"].as<uint8_t>();
      if (p >= 2 && p <= 4) {
        _precision = p;
      } else {
        Serial.println("WARNING: precision must be 2, 3, or 4 – using default");
      }
    }

    // -- startup -------------------------------------------------------------
    if (obj.containsKey("startup")) {
      _startup = obj["startup"].as<String>();
    }

    return true;
  }

  // ---- Accessors ----------------------------------------------------------

  Gain_t            getGain()            const { return _gain; }
  bool              isGainSet()          const { return _gain_set; }
  IntegrationTime_t getIntegrationTime() const { return _itime; }
  bool              isIntegrationTimeSet() const { return _itime_set; }
  uint8_t           getPrecision()       const { return _precision; }
  String            getStartup()         const { return _startup; }

private:
  Gain_t            _gain;
  IntegrationTime_t _itime;
  uint8_t           _precision;
  String            _startup;
  bool              _gain_set;
  bool              _itime_set;

  // Parse gain string → Gain_t.  Returns false on unknown string.
  // Accepts both the new "Nx" style and the legacy "low/med/high/max" style.
  static bool _parseGainString(const String& s, Gain_t& out) {
    // Canonical names matching the 4 hardware steps
    if (s == "1x")   { out = GAIN_1X;  return true; }
    if (s == "4x")   { out = GAIN_4X;  return true; }
    if (s == "16x")  { out = GAIN_16X; return true; }
    if (s == "60x")  { out = GAIN_60X; return true; }
    // Legacy names (backwards compatibility)
    if (s == "low")  { out = GAIN_1X;  return true; }
    if (s == "med")  { out = GAIN_16X; return true; }
    if (s == "high") { out = GAIN_60X; return true; }
    if (s == "max")  { out = GAIN_60X; return true; }
    return false;
  }

  // Parse integration time string → IntegrationTime_t.
  static bool _parseItimeString(const String& s, IntegrationTime_t& out) {
    if (s == "100ms") { out = INTEGRATIONTIME_100MS; return true; }
    if (s == "200ms") { out = INTEGRATIONTIME_200MS; return true; }
    if (s == "300ms") { out = INTEGRATIONTIME_300MS; return true; }
    if (s == "400ms") { out = INTEGRATIONTIME_400MS; return true; }
    if (s == "500ms") { out = INTEGRATIONTIME_500MS; return true; }
    if (s == "600ms") { out = INTEGRATIONTIME_600MS; return true; }
    return false;
  }
};

#endif // CONFIGURATION_H
