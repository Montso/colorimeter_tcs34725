/*
 * LightSensor_BH1750.h
 * Wrapper class for the BH1750FVI ambient light sensor.
 *
 * The BH1750 returns lux values rather than raw photon counts.  Lux is still
 * linearly proportional to light intensity, so it works correctly as the raw
 * intensity value in the absorbance / transmittance calculation:
 *
 *   transmittance = lux_sample / lux_blank
 *   absorbance    = -log10(transmittance)
 *
 * The getValue(float&) / SensorResult interface deliberately mirrors
 * LightSensor_TCS34725 so Colorimeter can call the same code path regardless
 * of which sensor is active.
 *
 * Sensitivity controls
 * --------------------
 * The BH1750 has no hardware gain register.  Sensitivity is adjusted via the
 * MTreg (measurement time register, range 31–254).  Higher values give more
 * counts per lux and are analogous to higher gain on the TCS34725.  Four
 * presets are provided and cycle with the 'g' command in raw sensor mode.
 *
 * Resolution / speed is controlled by the measurement mode, cycled with 'i':
 *   HIGH_RES   – 1.0 lux resolution, ~120 ms  (default, best for colorimetry)
 *   HIGH_RES_2 – 0.5 lux resolution, ~120 ms  (highest precision)
 *   LOW_RES    – 4.0 lux resolution, ~16 ms   (fastest, lowest precision)
 *
 * Continuous modes are used so readings are always fresh without needing to
 * manage one-shot reconfiguration in the measurement loop.
 *
 * Maximum lux: 65 535 (limited by the 16-bit return type of the library).
 * Readings at or above 65 535 lux are treated as overflow.
 */

#ifndef LIGHT_SENSOR_BH1750_H
#define LIGHT_SENSOR_BH1750_H

#include <Wire.h>
#include <BH1750.h>
#include "SensorCommon.h"

// ---------------------------------------------------------------------------
// MTreg presets – sensitivity analogue of TCS34725 gain.
// Accepted configuration.json strings: "low" | "default" | "high" | "max"
// ---------------------------------------------------------------------------
enum BH1750MTregPreset : uint8_t {
  MTREG_LOW     = 32,   // least sensitive; use in very bright light
  MTREG_DEFAULT = 69,   // factory default (~1 lux/count at standard mode)
  MTREG_HIGH    = 138,  // 2× default; useful for moderately dim samples
  MTREG_MAX     = 254   // ~3.7× default; use in very dim light / dark samples
};

static const BH1750MTregPreset MTREG_CYCLE_ORDER[] = {
  MTREG_LOW, MTREG_DEFAULT, MTREG_HIGH, MTREG_MAX
};
static const uint8_t MTREG_CYCLE_LEN =
  sizeof(MTREG_CYCLE_ORDER) / sizeof(MTREG_CYCLE_ORDER[0]);

// ---------------------------------------------------------------------------
// Measurement modes – speed / resolution analogue of TCS34725 integration time.
// Accepted configuration.json strings: "high_res" | "high_res_2" | "low_res"
// ---------------------------------------------------------------------------
enum BH1750ModePreset : uint8_t {
  BH1750_MODE_HIGH_RES   = 0,  // CONTINUOUS_HIGH_RES_MODE   – 1 lux,  ~120 ms
  BH1750_MODE_HIGH_RES_2 = 1,  // CONTINUOUS_HIGH_RES_MODE_2 – 0.5 lux, ~120 ms
  BH1750_MODE_LOW_RES    = 2   // CONTINUOUS_LOW_RES_MODE    – 4 lux,  ~16 ms
};

static const BH1750ModePreset MODE_CYCLE_ORDER[] = {
  BH1750_MODE_HIGH_RES, BH1750_MODE_HIGH_RES_2, BH1750_MODE_LOW_RES
};
static const uint8_t MODE_CYCLE_LEN =
  sizeof(MODE_CYCLE_ORDER) / sizeof(MODE_CYCLE_ORDER[0]);

// ---------------------------------------------------------------------------
// LightSensorBH1750
// ---------------------------------------------------------------------------
class LightSensorBH1750 {
public:
  // Readings at or above this value are treated as saturated
  static constexpr float MAX_LUX = 65535.0f;

  LightSensorBH1750()
    : _mtreg(MTREG_DEFAULT),
      _mode_preset(BH1750_MODE_HIGH_RES),
      _initialized(false) {}

  // ---- Lifecycle ----------------------------------------------------------

  // Call once in setup() after Wire.begin().
  // addr: 0x23 (ADDR pin LOW, default) or 0x5C (ADDR pin HIGH)
  bool initialize(uint8_t addr = 0x23) {
    _addr = addr;
    if (!_bh1750.begin(_toBH1750Mode(_mode_preset), addr)) return false;
    _initialized = true;
    _applyMTreg();
    return true;
  }

  bool isInitialized() const { return _initialized; }

  // ---- MTreg (sensitivity) ------------------------------------------------

  void setMTreg(BH1750MTregPreset preset) {
    _mtreg = preset;
    _applyMTreg();
  }

  BH1750MTregPreset getMTreg() const { return _mtreg; }

  // Advance through the four presets; wraps around.
  // Mapped to the 'g' (gain) command in raw sensor mode.
  void cycleMTreg() {
    for (uint8_t i = 0; i < MTREG_CYCLE_LEN; i++) {
      if (MTREG_CYCLE_ORDER[i] == _mtreg) {
        setMTreg(MTREG_CYCLE_ORDER[(i + 1) % MTREG_CYCLE_LEN]);
        return;
      }
    }
    setMTreg(MTREG_CYCLE_ORDER[0]);
  }

  // ---- Mode (resolution / speed) -----------------------------------------

  void setMode(BH1750ModePreset preset) {
    _mode_preset = preset;
    _bh1750.configure(_toBH1750Mode(preset));
    // Re-apply MTreg after mode change (some library versions reset it)
    _applyMTreg();
  }

  BH1750ModePreset getMode() const { return _mode_preset; }

  // Advance through the three modes; wraps around.
  // Mapped to the 'i' (integration time) command in raw sensor mode.
  void cycleMode() {
    for (uint8_t i = 0; i < MODE_CYCLE_LEN; i++) {
      if (MODE_CYCLE_ORDER[i] == _mode_preset) {
        setMode(MODE_CYCLE_ORDER[(i + 1) % MODE_CYCLE_LEN]);
        return;
      }
    }
    setMode(MODE_CYCLE_ORDER[0]);
  }

  // ---- Primary measurement ------------------------------------------------

  // Reads current lux level and writes it to `out`.
  //
  // Returns:
  //   SENSOR_OK       – read succeeded; out is valid
  //   SENSOR_OVERFLOW – reading >= 65 535 lux (sensor saturated)
  //   SENSOR_IO_ERROR – library returned -1 (I2C fault or unconfigured)
  //
  // Note: the BH1750 library blocks for the integration period internally
  // when using continuous modes, so this call takes ~16–120 ms to return
  // on the very first call after a mode change.  Subsequent calls return
  // immediately from the continuously-updated register.
  SensorResult getValue(float& out) {
    if (!_initialized) return SENSOR_IO_ERROR;

    float lux = _bh1750.readLightLevel();

    if (lux < 0.0f)    return SENSOR_IO_ERROR;
    if (lux >= MAX_LUX) return SENSOR_OVERFLOW;

    out = lux;
    return SENSOR_OK;
  }

  // ---- Display helpers ----------------------------------------------------

  static const char* mtregToString(BH1750MTregPreset p) {
    switch (p) {
      case MTREG_LOW:     return "low(32)";
      case MTREG_DEFAULT: return "def(69)";
      case MTREG_HIGH:    return "high(138)";
      case MTREG_MAX:     return "max(254)";
      default:            return "?";
    }
  }

  static const char* modeToString(BH1750ModePreset m) {
    switch (m) {
      case BH1750_MODE_HIGH_RES:   return "1lx/120ms";
      case BH1750_MODE_HIGH_RES_2: return "0.5lx/120ms";
      case BH1750_MODE_LOW_RES:    return "4lx/16ms";
      default:                     return "?";
    }
  }

private:
  BH1750           _bh1750;
  BH1750MTregPreset _mtreg;
  BH1750ModePreset  _mode_preset;
  bool              _initialized;
  uint8_t           _addr;

  // Map our preset enum to the BH1750 library's Mode enum
  static BH1750::Mode _toBH1750Mode(BH1750ModePreset p) {
    switch (p) {
      case BH1750_MODE_HIGH_RES_2: return BH1750::CONTINUOUS_HIGH_RES_MODE_2;
      case BH1750_MODE_LOW_RES:    return BH1750::CONTINUOUS_LOW_RES_MODE;
      case BH1750_MODE_HIGH_RES:
      default:                     return BH1750::CONTINUOUS_HIGH_RES_MODE;
    }
  }

  void _applyMTreg() {
    _bh1750.setMTreg(static_cast<byte>(_mtreg));
  }
};

#endif // LIGHT_SENSOR_BH1750_H
