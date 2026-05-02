/*
 * colorimeter_tcs34725.ino
 * ESP32-S3 Colorimeter – main sketch
 *
 * ============================================================
 *  MOCK / REAL HARDWARE SWITCH
 * ============================================================
 * Comment / uncomment ONE line to switch between real hardware
 * and the isolated mock for application-code testing.
 *
 * When USE_MOCK_COLORIMETER is defined:
 *   - No I2C, no LittleFS, no sensor libraries are used.
 *   - A MockColorimeter object is substituted; it has an
 *     identical public API so all loop() application code
 *     compiles and runs unchanged.
 *   - Select test scenarios via Serial at runtime (see below).
 *
 * When USE_MOCK_COLORIMETER is NOT defined:
 *   - Real TCS34725 + BH1750 sensors, LittleFS config files.
 * ============================================================
 */
#define USE_MOCK_COLORIMETER          // ← comment out for real hardware
// #undef USE_MOCK_COLORIMETER        // ← alternative explicit disable

// ============================================================
//  APPLICATION MODE  (applies to both mock and real builds)
// ============================================================
//  APP_MENU_DRIVEN – full interactive serial menu
//  APP_POLLING     – no menu; your code reads values via getters
// ============================================================
#define STARTUP_MODE APP_POLLING

// ============================================================
//  INCLUDES – conditional on mock vs. real
// ============================================================
#include <Arduino.h>

#ifdef USE_MOCK_COLORIMETER
  // ---- MOCK BUILD --------------------------------------------------------
  // No hardware libraries needed. SensorCommon.h is the only dependency.
  #include "SensorCommon.h"
  #include "MockColorimeter.h"
  MockColorimeter colorimeter;

#else
  // ---- REAL BUILD --------------------------------------------------------
  // All three library includes must be here (not inside .h files) so the
  // Arduino IDE library discovery compiles their .cpp implementations.
  #include <Wire.h>
  #include <LittleFS.h>
  #include <ArduinoJson.h>
  #include <Adafruit_TCS34725.h>
  #include <BH1750.h>
  #include "TCS34725_Colorimeter.h"
  Colorimeter colorimeter;
#endif

// ===========================================================================
//  setup()
// ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== ESP32-S3 Colorimeter ===");

#ifndef USE_MOCK_COLORIMETER
  // LittleFS only needed for real build
  if (!LittleFS.begin(true)) {
    Serial.println("FATAL: LittleFS mount failed – run ESP32 LittleFS Data Upload");
    while (true) { delay(1000); }
  }
  Serial.println("LittleFS mounted");
#else
  Serial.println("[MOCK] Hardware bypassed – running in mock mode");
  // Optionally pre-select a scenario before begin():
  // colorimeter.setScenario(MockScenario::RAMP);
#endif

  if (!colorimeter.begin()) {
    Serial.println("Colorimeter init failed – running in abort mode");
    return;
  }

  colorimeter.setAppState(STARTUP_MODE);

#ifndef USE_MOCK_COLORIMETER
  Serial.print("Primary sensor : ");
  Serial.println(colorimeter.getPrimarySensor() == SENSOR_TYPE_BH1750
                   ? "BH1750" : "TCS34725");
  Serial.print("TCS34725 present: "); Serial.println(colorimeter.isTCSPresent()    ? "yes" : "no");
  Serial.print("BH1750   present: "); Serial.println(colorimeter.isBH1750Present() ? "yes" : "no");
#endif

  if (STARTUP_MODE == APP_POLLING) {
    Serial.println("Mode: POLLING – reading values in loop()");
    Serial.println("Send 'b' to blank, 'm' to toggle display mode");
  } else {
    Serial.println("Mode: MENU-DRIVEN");
    Serial.println("Commands: b=blank  m=menu  u=up  d=down  r=select");
  }
}

// ===========================================================================
//  loop()
//
//  update() must always be called first – it handles Serial input and
//  (in menu mode) prints the current measurement.  The application code
//  below is only reached in POLLING mode.
// ===========================================================================
void loop() {
  colorimeter.update();

  // ---- POLLING MODE: application code goes here --------------------------
  // This block is the part you are testing.  It calls only public getters
  // and therefore compiles identically against MockColorimeter or Colorimeter.
  if (colorimeter.getAppState() == APP_POLLING) {

    // -- Absorbance & transmittance (primary sensor) -----------------------
    float absorbance    = colorimeter.getAbsorbance();
    float transmittance = colorimeter.getTransmittance();

    // -- Per-sensor raw readings -------------------------------------------
    float        tcs_raw    = -1.0f;
    float        bh1750_raw = -1.0f;
    SensorResult tcs_r      = SENSOR_IO_ERROR;
    SensorResult bh1750_r   = SENSOR_IO_ERROR;

    if (colorimeter.isTCSPresent())    tcs_r    = colorimeter.getTCSRaw(tcs_raw);
    if (colorimeter.isBH1750Present()) bh1750_r = colorimeter.getBH1750Raw(bh1750_raw);

    // -- Print results to Serial -------------------------------------------
    // Replace this block with your own processing / display / logging logic.
    Serial.print("ABS:");
    if (absorbance < 0.0f) Serial.print("ERR ");
    else                   { Serial.print(absorbance, 4); Serial.print(" "); }

    Serial.print("  TRANS:");
    if (transmittance < 0.0f) Serial.print("ERR ");
    else                      { Serial.print(transmittance, 4); Serial.print(" "); }

    Serial.print("  TCS:");
    switch (tcs_r) {
      case SENSOR_OK:       Serial.print(tcs_raw, 1);    break;
      case SENSOR_OVERFLOW: Serial.print("OVF");         break;
      default:              Serial.print("ERR");         break;
    }

    Serial.print("  BH:");
    switch (bh1750_r) {
      case SENSOR_OK:       Serial.print(bh1750_raw, 1); break;
      case SENSOR_OVERFLOW: Serial.print("OVF");         break;
      default:              Serial.print("ERR");         break;
    }

    Serial.print("  BLANK:");
    Serial.print(colorimeter.getIsBlanked() ? "Y" : "N");

    Serial.println();
  }
  #ifdef USE_MOCK_COLORIMETER
  delay(MockColorimeter::LOOP_DT_MS);
  #else
  delay(Colorimeter::LOOP_DT_MS);
  #endif
}