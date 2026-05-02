/*
 * colorimeter_tcs34725.ino
 * ESP32-S3 Colorimeter – main sketch
 *
 * Target : ESP32-S3 with 8 MB flash
 * Sensor : TCS34725 (I2C) and / or BH1750 (I2C)
 * Storage: LittleFS on internal flash (no SD card required)
 *
 * First-time setup
 * ----------------
 * 1. In Arduino IDE select:
 *      Tools → Board    → ESP32S3 Dev Module
 *      Tools → Flash    → 8MB
 *      Tools → Partition Scheme → Default 4MB with spiffs  (or any scheme
 *              that includes a LittleFS/SPIFFS partition of at least 1 MB)
 *
 * 2. Install the ESP32 LittleFS filesystem uploader plugin:
 *      https://github.com/lorol/arduino-esp32fs-plugin
 *
 * 3. Place configuration.json and calibrations.json in a folder called
 *    "data" inside the sketch folder, then run:
 *      Tools → ESP32 LittleFS Data Upload
 *    This writes the files to the LittleFS partition once.  They persist
 *    across normal firmware uploads.
 *
 * 4. Upload this sketch.
 *
 * Libraries required (install via Library Manager)
 * -------------------------------------------------
 *   ArduinoJson       – Benoit Blanchon       (v6 or v7)
 *   Adafruit TCS34725 – Adafruit Industries
 *
 * ============================================================
 * APPLICATION MODE
 * ============================================================
 * Uncomment exactly one of the two lines below.
 *
 *   APP_MENU_DRIVEN  –  Full interactive serial menu.
 *                       Serial commands (115 200 baud, no line ending):
 *                         b  – blank sensor
 *                         m  – toggle menu
 *                         u  – menu up
 *                         d  – menu down
 *                         r  – select / right
 *                         g  – cycle gain          (raw TCS34725 mode only)
 *                         i  – cycle integration time (raw TCS34725 mode only)
 *                         p  – switch to polling mode at runtime
 *
 *   APP_POLLING      –  No menu.  update() keeps sensors refreshed; your
 *                       code calls public getters to read measurements.
 *                       Serial commands (115 200 baud):
 *                         b  – blank sensor
 *                         m  – return to menu-driven mode at runtime
 * ============================================================
 */

#define STARTUP_MODE APP_MENU_DRIVEN
// #define STARTUP_MODE APP_POLLING

#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include "TCS34725_Colorimeter.h"

Colorimeter colorimeter;

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n=== ESP32-S3 TCS34725 Colorimeter ===");

  if (!LittleFS.begin(true)) {
    Serial.println("FATAL: LittleFS mount failed");
    Serial.println("Run 'ESP32 LittleFS Data Upload' from the Tools menu, then reset.");
    while (true) { delay(1000); }
  }
  Serial.println("LittleFS mounted");

  if (!colorimeter.begin()) {
    Serial.println("Colorimeter init failed – running in abort mode");
  } else {
    colorimeter.setAppState(STARTUP_MODE);

    if (STARTUP_MODE == APP_POLLING) {
      Serial.println("Colorimeter ready – POLLING mode");
      Serial.println("Commands: b=blank  m=return to menu");
    } else {
      Serial.println("Colorimeter ready – MENU-DRIVEN mode");
      Serial.println("Commands: b=blank  m=menu  u=up  d=down  r=select  p=polling mode");
      Serial.println("          g=cycle gain  i=cycle itime  (last two: raw TCS34725 only)");
    }
  }
}

void loop() {
  colorimeter.update();

#if STARTUP_MODE == APP_POLLING
  // ---- Polling mode: read any measurement via public getters ---------------
  //
  // All getters are available regardless of which sensors are fitted.
  // A return value of -1.0 signals an error or out-of-range condition.
  // getTCSRaw() / getBH1750Raw() additionally return a SensorResult code.

  float absorbance    = colorimeter.getAbsorbance();
  float transmittance = colorimeter.getTransmittance();

  float tcs_raw    = -1.0f;
  float bh1750_raw = -1.0f;
  SensorResult tcs_result    = SENSOR_IO_ERROR;
  SensorResult bh1750_result = SENSOR_IO_ERROR;

  if (colorimeter.isTCSPresent())    tcs_result    = colorimeter.getTCSRaw(tcs_raw);
  if (colorimeter.isBH1750Present()) bh1750_result = colorimeter.getBH1750Raw(bh1750_raw);

  // Example output – replace with your own processing logic.
  Serial.print("ABS:");   Serial.print(absorbance,    4);
  Serial.print("  TRANS:"); Serial.print(transmittance, 4);
  if (tcs_result    == SENSOR_OK) { Serial.print("  TCS:");  Serial.print(tcs_raw,    1); }
  if (bh1750_result == SENSOR_OK) { Serial.print("  BH:");   Serial.print(bh1750_raw, 1); }
  Serial.println();
#endif

  delay(Colorimeter::LOOP_DT_MS);
}
