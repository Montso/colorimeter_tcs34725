/*
 * colorimeter_tcs34725.ino
 * ESP32-S3 Colorimeter – main sketch
 *
 * Target : ESP32-S3 with 8 MB flash
 * Sensor : TCS34725 (I2C)
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
 * Serial commands (115 200 baud, no line ending)
 * -----------------------------------------------
 *   b  – blank sensor
 *   m  – toggle menu
 *   u  – menu up
 *   d  – menu down
 *   r  – select / right
 *   g  – cycle gain          (raw sensor mode only)
 *   i  – cycle integration time (raw sensor mode only)
 *
 * Wiring
 * ------
 *   TCS34725 VCC → 3.3 V
 *   TCS34725 GND → GND
 *   TCS34725 SCL → GPIO 9  (or the board's default SCL)
 *   TCS34725 SDA → GPIO 8  (or the board's default SDA)
 */

#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include "TCS34725_Colorimeter.h"

Colorimeter colorimeter;

void setup() {
  Serial.begin(115200);
  delay(1500);  // time for the host terminal to connect

  Serial.println("\n=== ESP32-S3 TCS34725 Colorimeter ===");

  // ---- LittleFS -----------------------------------------------------------
  // formatOnFail = true  → automatically formats a blank or corrupted partition
  // on first boot so the device does not brick if the partition is uninitialised.
  if (!LittleFS.begin(true)) {
    Serial.println("FATAL: LittleFS mount failed");
    Serial.println("Run 'ESP32 LittleFS Data Upload' from the Tools menu,");
    Serial.println("then reset.");
    while (true) { delay(1000); }
  }
  Serial.println("LittleFS mounted");

  // ---- Colorimeter --------------------------------------------------------
  if (!colorimeter.begin()) {
    // begin() already set MODE_ABORT internally; the update loop will show
    // the abort message.  We do not halt here so the Serial output is visible.
    Serial.println("Colorimeter init failed – running in abort mode");
  } else {
    Serial.println("Colorimeter ready");
    Serial.println("Commands: b=blank  m=menu  u=up  d=down  r=select");
    Serial.println("          g=cycle gain  i=cycle itime  (last two: raw mode only)");
  }
}

void loop() {
  colorimeter.update();
  delay(Colorimeter::LOOP_DT_MS);
}
