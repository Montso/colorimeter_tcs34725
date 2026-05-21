//#define USE_MOCK_COLORIMETER   // comment out for real hardware

#define STARTUP_MODE APP_INTERRUPT // APP_INTERRUPT | APP_POLLING | APP_MENU_DRIVEN
#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include <DFRobot_LWNode.h>
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_sleep.h"

#ifdef USE_MOCK_COLORIMETER
  #include "SensorCommon.h"
  #include "MockColorimeter.h"
  MockColorimeter colorimeter;
#else
  #include <ArduinoJson.h>
  #include <Adafruit_TCS34725.h>
  #include <BH1750.h>
  #include "TCS34725_Colorimeter.h"
  Colorimeter colorimeter;
#endif

#define READ_INTERVAL_MS      5000UL
#define TRANSMIT_INTERVAL_MS  30000UL
#define STATUS_INTERVAL_MS    60000UL

#define DATA_FILE      "/datalog.csv"
#define MAX_FREE_BYTES 1024

unsigned long lastStatus   = 0;
static unsigned long lastRead     = 0;
static unsigned long lastTransmit = 0;
RTC_DATA_ATTR uint16_t seqNum = 0;

DFRobot_LWNode_IIC node(1);

// ── LoRa verbose helper ─────────────────────────────────────────────────────
// LoRa TX/ACK Serial output is only printed when the user is at the menu root.
// This avoids cluttering the measure screen or pump/LED control screens.
static bool loraVerbose() {
  return colorimeter.getMode() == MODE_MENU;
}

// ── S-Box ───────────────────────────────────────────────────────────────────
static const uint8_t sbox[256] = {
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16
};

static const uint8_t rcon[11] = {
    0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x1b,0x36
};

static const uint8_t key[16] = {
    0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,
    0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c
};

uint8_t roundKeys[176];

uint8_t gmul(uint8_t a, uint8_t b) {
    uint8_t result = 0;
    while (b) {
        if (b & 1) result ^= a;
        if (a & 0x80) a = (a << 1) ^ 0x1b;
        else          a <<= 1;
        b >>= 1;
    }
    return result;
}

void keyExpansion() {
    for (int i = 0; i < 16; i++) roundKeys[i] = key[i];
    for (int i = 4; i < 44; i++) {
        uint8_t temp[4];
        for (int j = 0; j < 4; j++) temp[j] = roundKeys[(i-1)*4+j];
        if (i % 4 == 0) {
            uint8_t t = temp[0];
            temp[0]=temp[1]; temp[1]=temp[2]; temp[2]=temp[3]; temp[3]=t;
            for (int j = 0; j < 4; j++) temp[j] = sbox[temp[j]];
            temp[0] ^= rcon[i/4];
        }
        for (int j = 0; j < 4; j++)
            roundKeys[i*4+j] = roundKeys[(i-4)*4+j] ^ temp[j];
    }
}

void addRoundKey(uint8_t state[4][4], int round) {
    for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
            state[r][c] ^= roundKeys[round*16 + c*4 + r];
}

void subBytes(uint8_t state[4][4]) {
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
            state[r][c] = sbox[state[r][c]];
}

void shiftRows(uint8_t state[4][4]) {
    uint8_t t;
    t=state[1][0]; state[1][0]=state[1][1]; state[1][1]=state[1][2]; state[1][2]=state[1][3]; state[1][3]=t;
    t=state[2][0]; state[2][0]=state[2][2]; state[2][2]=t;
    t=state[2][1]; state[2][1]=state[2][3]; state[2][3]=t;
    t=state[3][3]; state[3][3]=state[3][2]; state[3][2]=state[3][1]; state[3][1]=state[3][0]; state[3][0]=t;
}

void mixColumns(uint8_t state[4][4]) {
    for (int c = 0; c < 4; c++) {
        uint8_t s0=state[0][c], s1=state[1][c], s2=state[2][c], s3=state[3][c];
        state[0][c] = gmul(0x02,s0)^gmul(0x03,s1)^s2^s3;
        state[1][c] = s0^gmul(0x02,s1)^gmul(0x03,s2)^s3;
        state[2][c] = s0^s1^gmul(0x02,s2)^gmul(0x03,s3);
        state[3][c] = gmul(0x03,s0)^s1^s2^gmul(0x02,s3);
    }
}

void aes_encrypt(uint8_t block[16]) {
    uint8_t state[4][4];
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
            state[r][c] = block[c*4+r];

    addRoundKey(state, 0);
    for (int round = 1; round <= 10; round++) {
        subBytes(state);
        shiftRows(state);
        if (round < 10) mixColumns(state);
        addRoundKey(state, round);
    }

    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
            block[c*4+r] = state[r][c];
}

void printHex(const char* label, uint8_t* data, int len) {
    Serial.print(label);
    for (int i = 0; i < len; i++) {
        if (data[i] < 0x10) Serial.print("0");
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

void printMem(const char* stage) {
    Serial.println("\n==============================");
    Serial.print("MEM STAGE: "); Serial.println(stage);
    Serial.println("==============================");
    Serial.print("Free heap: ");            Serial.println(ESP.getFreeHeap());
    Serial.print("Min free heap: ");        Serial.println(ESP.getMinFreeHeap());
    Serial.print("Largest free block: ");   Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    Serial.print("Sketch size: ");          Serial.println(ESP.getSketchSize());
    Serial.print("Free sketch space: ");    Serial.println(ESP.getFreeSketchSpace());
    Serial.print("Stack high water mark: "); Serial.println(uxTaskGetStackHighWaterMark(NULL));
    Serial.println("==============================\n");
}

void logToFlash(const char* sensorData) {
    if (LittleFS.totalBytes() - LittleFS.usedBytes() < MAX_FREE_BYTES) {
        Serial.println("[FS] Flash nearly full — skipping log");
        return;
    }
    File f;
    int attempts = 0;
    while (!f && attempts < 3) {
        f = LittleFS.open(DATA_FILE, FILE_APPEND);
        if (!f) {
            Serial.printf("[FS] ERROR: Open failed, attempt %d/3\n", attempts + 1);
            attempts++;
            delay(200);
        }
    }
    if (!f) {
        Serial.println("[FS] ERROR: Could not open file after 3 attempts — reading lost");
        return;
    }
    f.printf("%s\n", sensorData);
    f.close();
    Serial.printf("[FS] Stored: %s\n", sensorData);
}

volatile bool ackReceived = false;

void onReceive(uint8_t from, void* data, uint16_t len, int8_t rssi, int8_t snr) {
    if (ackReceived) return;
    char* msg = (char*)data;
    // Raw RX only printed at menu root to avoid corrupting active screens
    if (loraVerbose()) { Serial.print("[RX] raw: "); Serial.println(msg); }
    if (msg[0] == '1') ackReceived = true;
}

bool checkACK() {
    ackReceived = false;
    node.sleep(3000);
    if (ackReceived) {
        if (loraVerbose()) Serial.println("[ACK] Received");
        return true;
    }
    if (loraVerbose()) Serial.println("[ACK] Timeout — out of range");
    return false;
}

uint16_t crc16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

void flushAllToTransmit() {
    bool verbose = loraVerbose();
    if (verbose) Serial.println("[FS] --- Flushing stored records ---");

    while (LittleFS.exists(DATA_FILE)) {
        File f = LittleFS.open(DATA_FILE, FILE_READ);
        if (!f) return;

        String line = f.readStringUntil('\n');
        line.trim();

        if (line.length() == 0) {
            f.close();
            LittleFS.remove(DATA_FILE);
            if (verbose) Serial.println("[FS] Flush complete");
            return;
        }

        File tmp = LittleFS.open("/tmp.csv", FILE_WRITE);
        while (f.available()) {
            String remaining = f.readStringUntil('\n');
            remaining.trim();
            if (remaining.length() > 0) tmp.printf("%s\n", remaining.c_str());
        }
        f.close();
        tmp.close();

        LittleFS.remove(DATA_FILE);

        File check = LittleFS.open("/tmp.csv", FILE_READ);
        bool hasData = check && check.size() > 0;
        if (check) check.close();

        if (hasData) LittleFS.rename("/tmp.csv", DATA_FILE);
        else         LittleFS.remove("/tmp.csv");

        unsigned long startTx = millis();
        node.sendPacket(2, line);
        if (verbose) Serial.println("[FS] Replayed: " + line);
        unsigned long endTx = millis();
        if (verbose) { Serial.print("[FS] TX time 1 packet (ms): "); Serial.println(endTx - startTx); }

        if (!checkACK()) {
            if (verbose) Serial.println("[FS] Lost range during flush — stopping");
            return;
        }
    }

    if (verbose) Serial.println("[FS] Flush complete");
}

static void _readSensors(float& abs, float& tran) {
    abs  = colorimeter.getAbsorbance();
    tran = colorimeter.getTransmittance();
}

// Build, encrypt, and return the 48-char hex packet for a sensor reading.
// Caller is responsible for logging and/or transmitting.
static void buildPacket(char hexStr[49], float abs, float tran) {
    uint8_t sensorType = colorimeter.getPrimarySensor();
    uint8_t block[16]  = {0};
    snprintf((char*)block, sizeof(block), "%d,%.2f,%.2f", sensorType, abs, tran);
    int msgLen = strlen((char*)block);
    uint8_t padVal = 16 - msgLen;
    for (int i = msgLen; i < 16; i++) block[i] = padVal;

    aes_encrypt(block);

    for (int i = 0; i < 16; i++) sprintf(&hexStr[i*2], "%02x", block[i]);
    uint16_t crc = crc16(block, 16);
    sprintf(&hexStr[32], "%04x", crc);
    sprintf(&hexStr[36], "%04x", seqNum);
    sprintf(&hexStr[40], "%08lx", millis());
    seqNum++;
    hexStr[48] = '\0';
}

uint8_t getBatteryPercent() {
    return 85;
}

void sendStatusPacket() {
    bool verbose = loraVerbose();

    size_t totalBytes  = LittleFS.totalBytes();
    size_t usedBytes   = LittleFS.usedBytes();
    size_t storageUsed = (totalBytes > 0) ? (usedBytes * 100) / totalBytes : 0;
    uint8_t batteryPercent = getBatteryPercent();

    uint8_t block[16] = {0};
    int written = snprintf((char*)block, sizeof(block), "S,%lu,%u",
                           (unsigned long)storageUsed, batteryPercent);
    if (written < 0 || written >= (int)sizeof(block)) {
        Serial.println("[STATUS] Payload too long");
        return;
    }
    uint8_t padVal = 16 - written;
    for (int i = written; i < 16; i++) block[i] = padVal;

    aes_encrypt(block);

    char hexStr[49];
    for (int i = 0; i < 16; i++) sprintf(&hexStr[i*2], "%02x", block[i]);
    uint16_t crc = crc16(block, 16);
    if (verbose) { Serial.print("[STATUS] CRC = "); Serial.println(crc, HEX); }
    sprintf(&hexStr[32], "%04x", crc);
    sprintf(&hexStr[36], "%04x", seqNum);
    sprintf(&hexStr[40], "%08lx", millis());
    seqNum++;
    hexStr[48] = '\0';

    node.sendPacket(2, String(hexStr));

    if (checkACK()) {
        if (verbose) Serial.println("[STATUS] Sent successfully");
    } else {
        if (verbose) Serial.println("[STATUS] No ACK");
    }
}

// ── setup() ─────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("\n=== ESP32-S3 Colorimeter ===");

    if (!LittleFS.begin(true)) {
        Serial.println("FATAL: LittleFS mount failed!");
        while (true) delay(1000);
    }
    Serial.printf("[FS] Mounted OK — used: %u / %u bytes\n",
                  LittleFS.usedBytes(), LittleFS.totalBytes());

#ifdef USE_MOCK_COLORIMETER
    Serial.println("[MOCK] running – no hardware required");
    colorimeter.setScenario(MockScenario::RAMP);
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
    colorimeter.getLEDs().setOn(0, true);
    colorimeter.getLEDs().setBrightness(0, 180);
    keyExpansion();
    delay(2000);

    printMem("BOOT");
    delay(5000);

    node.begin(&Wire, &Serial);
    node.setRxCB(onReceive);

    printMem("AFTER node.begin");

    while (!node.setFreq(868000000) ||
           !node.setEIRP(13)        ||
           !node.setBW(125000)      ||
           !node.setSF(11)          ||
           !node.start()) {
        Serial.println("LoRa init failed, retrying...");
        delay(2000);
        printMem("INIT FAIL LOOP");
    }

    Serial.println("LoRa ready!");
    printMem("AFTER LoRa INIT");
}

// ── loop() ──────────────────────────────────────────────────────────────────

void loop() {
    colorimeter.update();

    unsigned long now = millis();

    // ---- READ every READ_INTERVAL_MS ----------------------------------------
    if (now - lastRead >= READ_INTERVAL_MS) {
        lastRead = now;

        float abs, tran;
        _readSensors(abs, tran);

        char hexStr[49];
        buildPacket(hexStr, abs, tran);

        // All storage and transmission only active in TESTING state
        if (colorimeter.getPump().getState() == PUMP_TESTING) {
            logToFlash(hexStr);

            bool verbose = loraVerbose();
            node.sendPacket(2, String(hexStr));
            if (verbose) {
                Serial.print("[TX] Testing immediate: ");
                Serial.println(hexStr);
            }
            checkACK();
        }

        return;
    }

    // ---- FLUSH every TRANSMIT_INTERVAL_MS (TESTING state only) -------------
    if (colorimeter.getPump().getState() == PUMP_TESTING &&
        now - lastTransmit >= TRANSMIT_INTERVAL_MS) {
        lastTransmit = now;
        if (LittleFS.exists(DATA_FILE)) {
            if (loraVerbose()) Serial.println("[TX] Scheduled flush...");
            flushAllToTransmit();
        }
        return;
    }

    // ---- STATUS every STATUS_INTERVAL_MS (TESTING state only) --------------
    if (colorimeter.getPump().getState() == PUMP_TESTING &&
        now - lastStatus >= STATUS_INTERVAL_MS) {
        lastStatus = now;
        sendStatusPacket();
        return;
    }
}
