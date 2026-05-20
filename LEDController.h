/*
 * LEDController.h
 * Independent PWM control for two LEDs on GPIO 4 (LED 1) and GPIO 5 (LED 2).
 *
 * Completely decoupled from Colorimeter measurement state.  Brightness and
 * on/off values persist across mode changes, blanking, and menu navigation.
 * Application code may call getLEDs() on the Colorimeter at any time to
 * drive the LEDs directly, independently of whatever mode is active.
 *
 * Uses the ESP32 Arduino core 3.x pin-based LEDC API:
 *   ledcAttach(pin, freq, resolution_bits)
 *   ledcWrite(pin, duty)
 * If you are on core 2.x, replace with ledcSetup / ledcAttachPin.
 *
 * Brightness is stored as 0-255 duty cycle and reported as 0-100 %.
 * stepUp() / stepDown() move in increments of PWM_STEP (default ~10 %).
 */

#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <Arduino.h>

class LEDController {
public:
  static const uint8_t  NUM_LEDS = 2;
  static const uint8_t  PIN_LED1 = 4;     // intended for absorbance measurements
  static const uint8_t  PIN_LED2 = 5;     // intended for transmittance measurements
  static const uint32_t PWM_FREQ = 5000;  // Hz — above audible range, below ADC noise
  static const uint8_t  PWM_BITS = 8;     // 0–255 duty range
  static const uint8_t  PWM_STEP = 13;    // ~5 % per up/down step

  // Human-readable names used by the menu and display
  static const char* name(uint8_t i) {
    static const char* _names[] = { "LED 1 (abs)", "LED 2 (trans)" };
    return (i < NUM_LEDS) ? _names[i] : "?";
  }

  // GPIO pin for a given LED index
  static uint8_t pin(uint8_t i) {
    static const uint8_t _pins[] = { PIN_LED1, PIN_LED2 };
    return (i < NUM_LEDS) ? _pins[i] : 0;
  }

  LEDController() {
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
      _brightness[i] = 0;
      _on[i]         = false;
    }
  }

  // Call once in setup() after Serial.begin().
  void begin() {
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
      ledcAttach(pin(i), PWM_FREQ, PWM_BITS);
      _apply(i);
    }
  }

  // ---- Per-LED control (index: 0 or 1) ------------------------------------

  // Turn on or off without changing stored brightness.
  void setOn(uint8_t i, bool on) {
    if (!_valid(i)) return;
    _on[i] = on;
    _apply(i);
  }

  void toggle(uint8_t i) {
    if (!_valid(i)) return;
    _on[i] = !_on[i];
    _apply(i);
  }

  // Set absolute duty (0–255).  Turns the LED on if duty > 0.
  void setBrightness(uint8_t i, uint8_t duty) {
    if (!_valid(i)) return;
    _brightness[i] = duty;
    if (duty > 0) _on[i] = true;
    _apply(i);
  }

  // Increment brightness by PWM_STEP, clamped at 255.  Turns LED on.
  void stepUp(uint8_t i) {
    if (!_valid(i)) return;
    _brightness[i] = (_brightness[i] + PWM_STEP > 255) ? 255 : _brightness[i] + PWM_STEP;
    _on[i] = true;
    _apply(i);
  }

  // Decrement brightness by PWM_STEP, clamped at 0.  Turns LED off at 0.
  void stepDown(uint8_t i) {
    if (!_valid(i)) return;
    _brightness[i] = (_brightness[i] < PWM_STEP) ? 0 : _brightness[i] - PWM_STEP;
    if (_brightness[i] == 0) _on[i] = false;
    _apply(i);
  }

  // ---- Accessors ----------------------------------------------------------

  uint8_t getBrightness(uint8_t i) const { return _valid(i) ? _brightness[i] : 0; }
  bool    isOn(uint8_t i)          const { return _valid(i) && _on[i]; }

  // Brightness as 0–100 % (rounded)
  uint8_t getPercent(uint8_t i) const {
    return _valid(i) ? static_cast<uint8_t>((_brightness[i] * 100u) / 255u) : 0;
  }

  // 10-character filled bar, e.g. "[#######   ]"
  // Useful for Serial display without needing a separate rendering step.
  String bar(uint8_t i) const {
    uint8_t filled = _valid(i) ? (_brightness[i] * 10u) / 255u : 0;
    String s = "[";
    for (uint8_t c = 0; c < 10; c++) s += (c < filled) ? '#' : ' ';
    s += "]";
    return s;
  }

  // ---- Bulk helpers -------------------------------------------------------

  void allOff() {
    for (uint8_t i = 0; i < NUM_LEDS; i++) { _on[i] = false; _apply(i); }
  }

  void allOn() {
    for (uint8_t i = 0; i < NUM_LEDS; i++) { _on[i] = true;  _apply(i); }
  }

private:
  uint8_t _brightness[NUM_LEDS];
  bool    _on[NUM_LEDS];

  bool _valid(uint8_t i) const { return i < NUM_LEDS; }

  void _apply(uint8_t i) {
    ledcWrite(pin(i), _on[i] ? _brightness[i] : 0u);
  }
};

#endif // LED_CONTROLLER_H