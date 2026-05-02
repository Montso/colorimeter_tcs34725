// colorimeter_tcs34725.ino  –  ESP32-S3 colorimeter sketch
// See GUIDE.md for setup, wiring, modes, and mock scenarios.

// ---- Build switches -------------------------------------------------------
#define USE_MOCK_COLORIMETER   // comment out for real hardware
#define STARTUP_MODE APP_POLLING // APP_INTERRUPT | APP_POLLING | APP_MENU_DRIVEN

// ---- Includes -------------------------------------------------------------
#include <Arduino.h>

#ifdef USE_MOCK_COLORIMETER
  #include "SensorCommon.h"
  #include "MockColorimeter.h"
  MockColorimeter colorimeter;
#else
  // Library includes must live here so the Arduino IDE discovers and
  // compiles their .cpp files. See GUIDE.md § "Library discovery".
  #include <Wire.h>
  #include <LittleFS.h>
  #include <ArduinoJson.h>
  #include <Adafruit_TCS34725.h>
  #include <BH1750.h>
  #include "TCS34725_Colorimeter.h"
  Colorimeter colorimeter;
#endif

// ---- Helpers --------------------------------------------------------------
static void _readAndPrint() {
  float abs  = colorimeter.getAbsorbance();
  float tran = colorimeter.getTransmittance();

  float        tcs_raw = -1.0f, bh_raw = -1.0f;
  SensorResult tcs_r   = SENSOR_IO_ERROR, bh_r = SENSOR_IO_ERROR;

  if (colorimeter.isTCSPresent())    tcs_r = colorimeter.getTCSRaw(tcs_raw);
  if (colorimeter.isBH1750Present()) bh_r  = colorimeter.getBH1750Raw(bh_raw);

  Serial.print("ABS:");
  abs  < 0 ? Serial.print("ERR  ") : (Serial.print(abs,  4), Serial.print("  "));
  Serial.print("TRANS:");
  tran < 0 ? Serial.print("ERR  ") : (Serial.print(tran, 4), Serial.print("  "));

  Serial.print("TCS:");
  if      (tcs_r == SENSOR_OK)       Serial.print(tcs_raw, 1);
  else if (tcs_r == SENSOR_OVERFLOW) Serial.print("OVF");
  else                               Serial.print("ERR");

  Serial.print("  BH:");
  if      (bh_r == SENSOR_OK)       Serial.print(bh_raw, 1);
  else if (bh_r == SENSOR_OVERFLOW) Serial.print("OVF");
  else                              Serial.print("ERR");

  Serial.print("  BLANK:"); Serial.println(colorimeter.getIsBlanked() ? "Y" : "N");
}

// ---- setup() --------------------------------------------------------------
void setup() {
  Serial.begin(115200); 
  delay(1500);
  Serial.println("\n=== ESP32-S3 Colorimeter ===");

#ifndef USE_MOCK_COLORIMETER
  if (!LittleFS.begin(true)) {
    Serial.println("FATAL: LittleFS mount failed – run ESP32 LittleFS Data Upload");
    while (true) delay(1000);
  }
  Serial.println("LittleFS mounted");
#else
  Serial.println("[MOCK] running – no hardware required");
  // colorimeter.setScenario(MockScenario::RAMP);  // optional pre-select
#endif

  if (!colorimeter.begin()) {
    Serial.println("init failed – abort mode");
    return;
  }

#ifndef USE_MOCK_COLORIMETER
  Serial.print("Primary: ");
  Serial.println(colorimeter.getPrimarySensor() == SENSOR_TYPE_BH1750 ? "BH1750" : "TCS34725");
  Serial.print("TCS34725: "); Serial.println(colorimeter.isTCSPresent()    ? "ok" : "--");
  Serial.print("BH1750:   "); Serial.println(colorimeter.isBH1750Present() ? "ok" : "--");
#endif

  colorimeter.setAppState(STARTUP_MODE);
}

// ---- loop() ---------------------------------------------------------------
void loop() {
  colorimeter.update();

  // Interrupt mode: application code is sole initiator of sensor reads.
  if (colorimeter.getAppState() == APP_INTERRUPT) {
    _readAndPrint();  // replace with your own trigger / processing logic
  }

  // Polling mode: reads gated by 's' / Enter commands handled in update().
  if (colorimeter.getAppState() == APP_POLLING &&
      (colorimeter.isPollingActive() || colorimeter.consumeOneShot())) {
    _readAndPrint();
  }

  #ifdef USE_MOCK_COLORIMETER
  delay(MockColorimeter::LOOP_DT_MS);
  #else
  delay(Colorimeter::LOOP_DT_MS);
  #endif
}