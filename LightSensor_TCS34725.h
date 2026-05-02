/*
 * LightSensor_TCS34725.h
 * Wrapper class for the TCS34725 RGB+Clear light sensor.
 *
 * Changes in this revision:
 *   - SensorResult moved to SensorCommon.h so BH1750 driver can share it.
 *   - getValue(float& out) overload added alongside getValue(uint16_t& out).
 *     Colorimeter calls the float overload for both sensors so the same
 *     arithmetic path (raw / blank → transmittance → absorbance) works
 *     regardless of which sensor is active.
 */

#ifndef LIGHT_SENSOR_TCS34725_H
#define LIGHT_SENSOR_TCS34725_H

#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "SensorCommon.h"

// ---------------------------------------------------------------------------
// Gain
//
// Ordinal values 0-3 match TCS34725 register values so static_cast to
// tcs34725Gain_t is always safe.
//
// Accepted configuration.json strings: "1x" | "4x" | "16x" | "60x"
// Legacy strings "low" / "med" / "high" / "max" also accepted in
// Configuration.h for backwards compatibility.
// ---------------------------------------------------------------------------
enum Gain_t : uint8_t {
  GAIN_1X  = TCS34725_GAIN_1X,    // 0x00 –  1x  (bright environments)
  GAIN_4X  = TCS34725_GAIN_4X,    // 0x01 –  4x
  GAIN_16X = TCS34725_GAIN_16X,   // 0x02 – 16x  (default)
  GAIN_60X = TCS34725_GAIN_60X    // 0x03 – 60x  (dark environments)
};

static const Gain_t GAIN_CYCLE_ORDER[] = {
  GAIN_1X, GAIN_4X, GAIN_16X, GAIN_60X
};
static const uint8_t GAIN_CYCLE_LEN =
  sizeof(GAIN_CYCLE_ORDER) / sizeof(GAIN_CYCLE_ORDER[0]);

// ---------------------------------------------------------------------------
// Integration time
// ---------------------------------------------------------------------------
enum IntegrationTime_t : uint8_t {
  INTEGRATIONTIME_100MS = TCS34725_INTEGRATIONTIME_101MS,
  INTEGRATIONTIME_200MS = TCS34725_INTEGRATIONTIME_199MS,
  INTEGRATIONTIME_300MS = TCS34725_INTEGRATIONTIME_300MS,
  INTEGRATIONTIME_400MS = TCS34725_INTEGRATIONTIME_401MS,
  INTEGRATIONTIME_500MS = TCS34725_INTEGRATIONTIME_499MS,  // default
  INTEGRATIONTIME_600MS = TCS34725_INTEGRATIONTIME_614MS
};

static const IntegrationTime_t ITIME_CYCLE_ORDER[] = {
  INTEGRATIONTIME_100MS,
  INTEGRATIONTIME_200MS,
  INTEGRATIONTIME_300MS,
  INTEGRATIONTIME_400MS,
  INTEGRATIONTIME_500MS,
  INTEGRATIONTIME_600MS
};
static const uint8_t ITIME_CYCLE_LEN =
  sizeof(ITIME_CYCLE_ORDER) / sizeof(ITIME_CYCLE_ORDER[0]);

// ---------------------------------------------------------------------------
// LightSensor (TCS34725)
// ---------------------------------------------------------------------------
class LightSensor {
public:
  static const uint16_t MAX_COUNTS_100MS = 36863;  // 0x8FFF
  static const uint16_t MAX_COUNTS       = 65535;  // 0xFFFF

  LightSensor()
    : tcs(TCS34725_INTEGRATIONTIME_499MS, TCS34725_GAIN_16X),
      _gain(GAIN_16X),
      _integration_time(INTEGRATIONTIME_500MS),
      _initialized(false) {}

  // ---- Lifecycle ----------------------------------------------------------

  bool initialize() {
    if (!tcs.begin()) return false;
    _initialized = true;
    setGain(_gain);
    setIntegrationTime(_integration_time);
    return true;
  }

  bool isInitialized() const { return _initialized; }

  // ---- Gain ---------------------------------------------------------------

  void setGain(Gain_t gain) {
    _gain = gain;
    tcs.setGain(static_cast<tcs34725Gain_t>(gain));
  }

  Gain_t getGain() const { return _gain; }

  void cycleGain() {
    for (uint8_t i = 0; i < GAIN_CYCLE_LEN; i++) {
      if (GAIN_CYCLE_ORDER[i] == _gain) {
        setGain(GAIN_CYCLE_ORDER[(i + 1) % GAIN_CYCLE_LEN]);
        return;
      }
    }
    setGain(GAIN_CYCLE_ORDER[0]);
  }

  // ---- Integration time ---------------------------------------------------

  void setIntegrationTime(IntegrationTime_t itime) {
    _integration_time = itime;
    tcs.setIntegrationTime(static_cast<uint8_t>(itime));
  }

  IntegrationTime_t getIntegrationTime() const { return _integration_time; }

  void cycleIntegrationTime() {
    for (uint8_t i = 0; i < ITIME_CYCLE_LEN; i++) {
      if (ITIME_CYCLE_ORDER[i] == _integration_time) {
        setIntegrationTime(ITIME_CYCLE_ORDER[(i + 1) % ITIME_CYCLE_LEN]);
        return;
      }
    }
    setIntegrationTime(ITIME_CYCLE_ORDER[0]);
  }

  // ---- Saturation ceiling -------------------------------------------------

  uint16_t getMaxCounts() const {
    return (_integration_time == INTEGRATIONTIME_100MS)
             ? MAX_COUNTS_100MS : MAX_COUNTS;
  }

  // ---- Primary measurement ------------------------------------------------

  // Integer overload – returns raw 16-bit clear channel count.
  SensorResult getValue(uint16_t& out) {
    if (!_initialized) return SENSOR_IO_ERROR;

    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);

    if (r == 0 && g == 0 && b == 0 && c == 0) return SENSOR_IO_ERROR;
    if (c >= getMaxCounts())                   return SENSOR_OVERFLOW;

    out = c;
    return SENSOR_OK;
  }

  // Float overload – same value cast to float.
  // Colorimeter uses this overload so the same arithmetic works for both
  // the TCS34725 (counts) and BH1750 (lux) without separate code paths.
  SensorResult getValue(float& out) {
    uint16_t raw;
    SensorResult r = getValue(raw);
    if (r == SENSOR_OK) out = static_cast<float>(raw);
    return r;
  }

  // ---- Auxiliary readings -------------------------------------------------

  void getRawData(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c) {
    tcs.getRawData(&r, &g, &b, &c);
  }

  uint16_t getColorTemperature() {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    return tcs.calculateColorTemperature_dn40(r, g, b, c);
  }

  uint16_t getLux() {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    return tcs.calculateLux(r, g, b);
  }

  // ---- Display helpers ----------------------------------------------------

  static const char* gainToString(Gain_t g) {
    switch (g) {
      case GAIN_1X:  return "1x";
      case GAIN_4X:  return "4x";
      case GAIN_16X: return "16x";
      case GAIN_60X: return "60x";
      default:       return "?";
    }
  }

  static const char* integrationTimeToString(IntegrationTime_t t) {
    switch (t) {
      case INTEGRATIONTIME_100MS: return "100ms";
      case INTEGRATIONTIME_200MS: return "200ms";
      case INTEGRATIONTIME_300MS: return "300ms";
      case INTEGRATIONTIME_400MS: return "400ms";
      case INTEGRATIONTIME_500MS: return "500ms";
      case INTEGRATIONTIME_600MS: return "600ms";
      default:                    return "?";
    }
  }

private:
  Adafruit_TCS34725 tcs;
  Gain_t            _gain;
  IntegrationTime_t _integration_time;
  bool              _initialized;
};

#endif // LIGHT_SENSOR_TCS34725_H
