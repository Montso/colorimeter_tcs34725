/*
 * PumpController.h
 * State-machine controller for a peristaltic pump and a reagent valve.
 *
 * Four states drive two GPIO outputs:
 *
 *   State       Pump   Valve   Purpose
 *   ─────────── ────── ─────── ──────────────────────────────────────────
 *   PUMP_OFF      OFF    OFF   Idle / between runs
 *   PUMP_WATER    ON     OFF   Flush / prime with water sample
 *   PUMP_REAGENT  ON     ON    Add reagent to sample
 *   PUMP_TESTING  OFF    OFF   Pump stopped; measurement in progress
 *
 * Transitions are explicit and manual – the controller never advances
 * state on its own.  Application code or the menu calls setState().
 *
 * Fully independent of Colorimeter measurement state, identical design
 * pattern to LEDController.
 */

#ifndef PUMP_CONTROLLER_H
#define PUMP_CONTROLLER_H

#include <Arduino.h>

enum PumpState : uint8_t {
  PUMP_OFF      = 0,
  PUMP_WATER    = 1,
  PUMP_REAGENT  = 2,
  PUMP_TESTING  = 3
};

class PumpController {
public:
  static const uint8_t PIN_PUMP  = 6;   // HIGH = pump motor on
  static const uint8_t PIN_VALVE = 7;   // HIGH = valve open (reagent flows)

  PumpController() : _state(PUMP_OFF) {}

  // Call once in setup() after Serial.begin().
  void begin() {
    pinMode(PIN_PUMP,  OUTPUT);
    pinMode(PIN_VALVE, OUTPUT);
    _apply();
    Serial.print("Pump controller ready  (pump=GPIO");
    Serial.print(PIN_PUMP);
    Serial.print("  valve=GPIO");
    Serial.print(PIN_VALVE);
    Serial.println(")");
  }

  // ---- State control ------------------------------------------------------

  void setState(PumpState s) {
    _state = s;
    _apply();
    Serial.print("Pump → "); Serial.println(stateName());
  }

  PumpState getState()    const { return _state; }
  bool      isPumpOn()    const { return _state == PUMP_WATER || _state == PUMP_REAGENT; }
  bool      isValveOn()   const { return _state == PUMP_REAGENT; }

  // ---- Display helper -----------------------------------------------------

  const char* stateName() const {
    switch (_state) {
      case PUMP_OFF:      return "OFF";
      case PUMP_WATER:    return "WATER SAMPLE";
      case PUMP_REAGENT:  return "REAGENT";
      case PUMP_TESTING:  return "TESTING";
      default:            return "?";
    }
  }

private:
  PumpState _state;

  void _apply() {
    digitalWrite(PIN_PUMP,  isPumpOn()  ? HIGH : LOW);
    digitalWrite(PIN_VALVE, isValveOn() ? HIGH : LOW);
  }
};

#endif // PUMP_CONTROLLER_H
