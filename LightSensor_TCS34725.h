/*
 * LightSensor_TCS34725.h
 * Wrapper class for TCS34725 color sensor
 *
 * ESP32-S3 changes vs. original Arduino Uno port:
 *   - All four TCS34725 hardware gain steps are now exposed (1x/4x/16x/60x).
 *     The previous version silently mapped both GAIN_HIGH and GAIN_MAX to 60x,
 *     making one cycle step a no-op.
 *   - C++ exceptions replaced with a SensorResult return-code pattern.
 *     The ESP32 Arduino core disables exceptions by default; enabling them
 *     adds overhead and is non-idiomatic for embedded targets.
 *   - cycleGain() and cycleIntegrationTime() moved here from Colorimeter so
 *     the cycling logic lives alongside the enum definitions it depends on.
 */

#ifndef LIGHT_SENSOR_TCS34725_H
#define LIGHT_SENSOR_TCS34725_H

#include <Wire.h>
#include "Adafruit_TCS34725.h"

// ---------------------------------------------------------------------------
// Result codes returned by getValue() in place of thrown exceptions
// ---------------------------------------------------------------------------
enum SensorResult : uint8_t {
  SENSOR_OK       = 0,
  SENSOR_OVERFLOW = 1,  // clear channel >= saturation limit
  SENSOR_IO_ERROR = 2   // device not responding on I2C
};

// ---------------------------------------------------------------------------
// Gain
//
// Ordinal values (0-3) deliberately match the TCS34725 register values
// (TCS34725_GAIN_1X = 0x00 … TCS34725_GAIN_60X = 0x03) so a direct
// static_cast to tcs34725Gain_t is always safe.
//
// Accepted configuration.json strings: "1x" | "4x" | "16x" | "60x"
// Legacy strings "low" / "med" / "high" / "max" remain accepted in
// Configuration.h for backwards compatibility with existing JSON files.
// ---------------------------------------------------------------------------
enum Gain_t : uint8_t {
  GAIN_1X  = TCS34725_GAIN_1X,    // 0x00 –  1x  (bright environments)
  GAIN_4X  = TCS34725_GAIN_4X,    // 0x01 –  4x
  GAIN_16X = TCS34725_GAIN_16X,   // 0x02 – 16x  (default)
  GAIN_60X = TCS34725_GAIN_60X    // 0x03 – 60x  (dark environments)
};

// Ordered sequence used by cycleGain()
static const Gain_t GAIN_CYCLE_ORDER[] = {
  GAIN_1X, GAIN_4X, GAIN_16X, GAIN_60X
};
static const uint8_t GAIN_CYCLE_LEN =
  sizeof(GAIN_CYCLE_ORDER) / sizeof(GAIN_CYCLE_ORDER[0]);

// ---------------------------------------------------------------------------
// Integration time
//
// Register values are non-contiguous, so cycling uses ITIME_CYCLE_ORDER.
// ---------------------------------------------------------------------------
enum IntegrationTime_t : uint8_t {
  INTEGRATIONTIME_100MS = TCS34725_INTEGRATIONTIME_101MS,  // ~100 ms
  INTEGRATIONTIME_200MS = TCS34725_INTEGRATIONTIME_199MS,  // ~200 ms
  INTEGRATIONTIME_300MS = TCS34725_INTEGRATIONTIME_300MS,  // ~300 ms
  INTEGRATIONTIME_400MS = TCS34725_INTEGRATIONTIME_401MS,  // ~400 ms
  INTEGRATIONTIME_500MS = TCS34725_INTEGRATIONTIME_499MS,  // ~500 ms (default)
  INTEGRATIONTIME_600MS = TCS34725_INTEGRATIONTIME_614MS   // ~614 ms
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
// LightSensor
// ---------------------------------------------------------------------------
class LightSensor {
public:
  // Saturation counts differ at 100 ms vs. all longer integration times
  static const uint16_t MAX_COUNTS_100MS = 36863;  // 0x8FFF
  static const uint16_t MAX_COUNTS       = 65535;  // 0xFFFF

  LightSensor()
    : tcs(TCS34725_INTEGRATIONTIME_499MS, TCS34725_GAIN_16X),
      _gain(GAIN_16X),
      _integration_time(INTEGRATIONTIME_500MS),
      _initialized(false) {}

  // ---- Lifecycle ----------------------------------------------------------

  // Call once in setup().  Returns false if the device is not found on I2C.
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

  // Advance to the next step in the 4-step cycle; wraps around.
  void cycleGain() {
    for (uint8_t i = 0; i < GAIN_CYCLE_LEN; i++) {
      if (GAIN_CYCLE_ORDER[i] == _gain) {
        setGain(GAIN_CYCLE_ORDER[(i + 1) % GAIN_CYCLE_LEN]);
        return;
      }
    }
    setGain(GAIN_CYCLE_ORDER[0]);  // unknown value → reset
  }

  // ---- Integration time ---------------------------------------------------

  void setIntegrationTime(IntegrationTime_t itime) {
    _integration_time = itime;
    tcs.setIntegrationTime(static_cast<uint8_t>(itime));
  }

  IntegrationTime_t getIntegrationTime() const { return _integration_time; }

  // Advance to the next integration time; wraps around.
  void cycleIntegrationTime() {
    for (uint8_t i = 0; i < ITIME_CYCLE_LEN; i++) {
      if (ITIME_CYCLE_ORDER[i] == _integration_time) {
        setIntegrationTime(ITIME_CYCLE_ORDER[(i + 1) % ITIME_CYCLE_LEN]);
        return;
      }
    }
    setIntegrationTime(ITIME_CYCLE_ORDER[0]);
  }

  // ---- Saturation ceiling for the current integration time ----------------

  uint16_t getMaxCounts() const {
    return (_integration_time == INTEGRATIONTIME_100MS)
             ? MAX_COUNTS_100MS
             : MAX_COUNTS;
  }

  // ---- Primary measurement ------------------------------------------------
  //
  // Reads the clear (broadband) channel, which is used for absorbance and
  // transmittance calculations.  Writes the result to `out`.
  //
  // Returns:
  //   SENSOR_OK       – successful read, `out` is valid
  //   SENSOR_OVERFLOW – sensor is saturated; reduce gain or integration time
  //   SENSOR_IO_ERROR – device unreachable (check wiring / I2C address)

  SensorResult getValue(uint16_t& out) {
    if (!_initialized) return SENSOR_IO_ERROR;

    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);

    // All-zero with a live device is diagnostic of an I2C fault
    if (r == 0 && g == 0 && b == 0 && c == 0) return SENSOR_IO_ERROR;

    if (c >= getMaxCounts()) return SENSOR_OVERFLOW;

    out = c;
    return SENSOR_OK;
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
