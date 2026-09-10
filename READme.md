# Basic colorimeter and PAR reading project for a marine sensing prototype (EEE4113F design project).

This was a demonstrator prototype for a low-cost, continuous sampling test device for dissolved oxygen (colorimetry proxyn using TCS34725), turbidity, and PAR (BH1750 sensor) for marine sampling.

The test hardware was hugely based off the open-source [Open Colorimetry Project](https://blog.iorodeo.com/open-colorimeter-product-guide/). Code was written in Arduino for ESP32-S3.

<img width="850" height="600" alt="img1" src="https://github.com/user-attachments/assets/9db92729-b276-4d24-910d-91c05a8f208a" />
<img width="850" alt="img2" src="https://github.com/user-attachments/assets/9c2be614-81f0-471b-b3e8-1ec35aa018a2" />

---

## File map

| File | Role |
|------|------|
| `colorimeter_tcs34725.ino` | Entry point. Build switches and application loop. |
| `TCS34725_Colorimeter.h` | `Colorimeter` class — sensor init, measurements, menu, mode state machine. |
| `MockColorimeter.h` | Drop-in mock with identical API. No hardware required. |
| `LightSensor_TCS34725.h` | TCS34725 driver wrapper. |
| `LightSensor_BH1750.h` | BH1750 driver wrapper. |
| `SensorCommon.h` | `SensorResult` enum shared by both drivers. |
| `Configuration.h` | Loads `configuration.json` from LittleFS. |
| `Calibrations.h` | Loads `calibrations.json` and applies polynomial curves. |
| `data/configuration.json` | Device settings (uploaded to LittleFS). |
| `data/calibrations.json` | Calibration curves (uploaded to LittleFS). |

---

## Wiring

Both sensors share the same I2C bus.

```
TCS34725 / BH1750    ESP32-S3
VCC              →   3.3 V
GND              →   GND
SCL              →   GPIO 9  (or board-default SCL)
SDA              →   GPIO 8  (or board-default SDA)

BH1750 ADDR pin  →   GND    → address 0x23 (default)
                 →   3.3 V  → address 0x5C
```

---

## Build switches

Both switches are at the top of `colorimeter_tcs34725.ino`.

### `USE_MOCK_COLORIMETER`

```cpp
#define USE_MOCK_COLORIMETER   // comment out for real hardware
```

When defined: `MockColorimeter` is used instead of `Colorimeter`. No I2C, no LittleFS, no sensor libraries are initialised. The public API is identical so `loop()` application code compiles and runs unchanged.

When not defined: real hardware path. LittleFS is mounted, both sensors initialised.

### `STARTUP_MODE`

```cpp
#define STARTUP_MODE APP_INTERRUPT  // APP_INTERRUPT | APP_POLLING | APP_MENU_DRIVEN
```

Selects the initial `AppState`. Can also be changed at runtime via Serial commands.

---

## Application modes

| Mode | `update()` reads sensors? | `update()` prints? | Who triggers reads? |
|------|--------------------------|-------------------|---------------------|
| `APP_MENU_DRIVEN` | Yes, every cycle | Yes | `update()` automatically |
| `APP_POLLING` | No | No | `loop()` when `isPollingActive()` / `consumeOneShot()` |
| `APP_INTERRUPT` | **Never** | No | Application code exclusively |

### APP_MENU_DRIVEN

Full interactive serial menu. `update()` reads sensors and prints measurements every loop cycle. No application code needed in `loop()`.

### APP_POLLING

No automatic output. `update()` services streaming control commands. Application code gates sensor reads behind:

```cpp
if (colorimeter.isPollingActive() || colorimeter.consumeOneShot()) {
    float abs = colorimeter.getAbsorbance();
    // ...
}
```

`isPollingActive()` is `true` continuously while streaming. `consumeOneShot()` is `true` exactly once per one-shot request, then auto-clears to idle.

### APP_INTERRUPT

Fully idle. `update()` only checks for blank (`b`) and mode-switch (`m`, `p`) Serial commands — no sensor reads occur inside `update()` under any circumstance. Application code is the sole initiator of every I2C transaction:

```cpp
// Called on your schedule — timer ISR, hardware pin, RTOS task, test harness, etc.
float abs = colorimeter.getAbsorbance();   // one I2C read, here, now
```

The distinction from `APP_POLLING` idle: polling idle still monitors Serial for an `s`/Enter command that would cause `update()` to trigger a read on the next cycle. In interrupt mode no such command path exists — only the application can initiate a read.

---

## Serial commands

### Menu-driven mode

| Key | Action |
|-----|--------|
| `b` | Blank sensor |
| `m` | Open / close menu |
| `u` / `d` | Navigate menu |
| `r` | Select item |
| `g` | Cycle gain (Raw TCS34725 view only) |
| `i` | Cycle integration time / BH1750 mode (raw view only) |
| `p` | Switch to polling mode |

### Polling mode

| Key | Action |
|-----|--------|
| `s` | Start streaming |
| `x` | Stop streaming (return to idle) |
| Enter | One-shot read |
| `b` | Blank sensor |
| `m` | Return to menu-driven mode |
| `?` | Print current polling state |

### Interrupt mode

| Key | Action |
|-----|--------|
| `b` | Blank sensor |
| `m` | Switch to menu-driven mode |
| `p` | Switch to polling mode |
| `?` | Print help |

---

## Configuration file (`data/configuration.json`)

```json
{
  "primary_sensor":       "tcs34725",
  "gain":                 "16x",
  "integration_time":     "500ms",
  "bh1750_sensitivity":   "default",
  "bh1750_mode":          "high_res",
  "precision":            2,
  "startup":              "Absorbance"
}
```

`primary_sensor` controls which sensor drives Absorbance and Transmittance. Both sensors are always initialised if present; this key does not disable either one.

**Gain options:** `1x` `4x` `16x` `60x` (also accepts legacy `low` `med` `high` `max`)  
**Integration time:** `100ms` `200ms` `300ms` `400ms` `500ms` `600ms`  
**BH1750 sensitivity:** `low` `default` `high` `max` (MTreg 32 / 69 / 138 / 254)  
**BH1750 mode:** `high_res` (1 lux / 120 ms) · `high_res_2` (0.5 lux / 120 ms) · `low_res` (4 lux / 16 ms)  
**Precision:** `2` `3` `4` (decimal places)  
**Startup:** any menu item name, or omit to default to `Absorbance`

If the primary sensor is absent at boot the other sensor is used as a fallback automatically.

---

## Calibration file (`data/calibrations.json`)

Polynomial calibration curves. Coefficients are ordered `[c0, c1, c2, …]` for:

```
result = c0 + c1·x + c2·x² + …    where x = absorbance
```

```json
{
  "Nitrite API": {
    "units":    "ppm",
    "led":      "520",
    "fit_type": "polynomial",
    "fit_coef": [0.131, 1.259, 0.0],
    "range":    { "min": 0.0, "max": 1.4 }
  }
}
```

`led` is documentation only (the wavelength of the LED used when the curve was measured). `range` defines the valid absorbance window; readings outside it return `-1.0`. Linear fits (`"fit_type": "linear"`) accept at most 2 coefficients and do not require a `range`.

---

## Mock scenarios

Select with `colorimeter.setScenario(MockScenario::XXXX)` in `setup()`, or send the key over Serial at runtime (no line ending).

| Key | Scenario | Purpose |
|-----|----------|---------|
| `0` | `STATIC_NORMAL` | Baseline happy path |
| `1` | `STATIC_ZERO` | Zero absorbance / blank-level |
| `2` | `STATIC_HIGH` | Near-maximum absorbance |
| `3` | `RAMP` | 0 → 1.5 AU over 30 s, repeating |
| `4` | `SINE` | Oscillates 0.1 → 0.9 AU over 20 s |
| `5` | `NOISY` | Mid-range + deterministic noise |
| `6` | `OVERFLOW_TCS` | TCS saturated, BH1750 normal |
| `7` | `OVERFLOW_BH1750` | BH1750 saturated, TCS normal |
| `8` | `IO_ERROR_PRIMARY` | Primary sensor I/O fault |
| `9` | `BOTH_SENSORS_FAIL` | Both absent → abort path |
| `o` | `OUT_OF_RANGE` | Absorbance outside calibration range |

Additional mock Serial commands: `b` = blank · `?` = print current scenario + absorbance · `m` = toggle display.

Raw values in dynamic scenarios (RAMP, SINE, NOISY) are derived from absorbance via the Beer-Lambert relation so all three getters stay physically coherent with each other throughout the cycle.

---

## Return values and error handling

All measurement getters return `float`. A return value of `-1.0` signals:
- sensor overflow
- I2C fault
- absorbance outside the calibration range (for named calibrations)

`getTCSRaw()` and `getBH1750Raw()` additionally return a `SensorResult`:

```cpp
float raw;
SensorResult r = colorimeter.getTCSRaw(raw);
// r == SENSOR_OK | SENSOR_OVERFLOW | SENSOR_IO_ERROR
```

`blankSensor()` takes `NUM_BLANK_SAMPLES` (50) readings spaced `BLANK_DT_MS` (50 ms) apart and stores the median. Calling it with `set_blanked = false` performs the read without marking the device as blanked (used internally at startup).
