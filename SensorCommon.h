/*
 * SensorCommon.h
 * Types shared between LightSensor_TCS34725 and LightSensor_BH1750.
 *
 * Keeping SensorResult here avoids duplicating the definition across drivers
 * and prevents the include-order dependency that would arise if one driver
 * header included the other just for this enum.
 */

#ifndef SENSOR_COMMON_H
#define SENSOR_COMMON_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Return codes used by every sensor's getValue() method in place of C++
// exceptions (which are disabled by default in the ESP32 Arduino core).
// ---------------------------------------------------------------------------
enum SensorResult : uint8_t {
  SENSOR_OK       = 0,  // read succeeded; output parameter is valid
  SENSOR_OVERFLOW = 1,  // sensor saturated; lower sensitivity / integration time
  SENSOR_IO_ERROR = 2   // device not responding on I2C
};

#endif // SENSOR_COMMON_H
