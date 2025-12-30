#pragma once
#include <Arduino.h>

class LineSensorQTR {
public:
  explicit LineSensorQTR(uint8_t pin);

  void begin();
  uint16_t readUs() const;

  uint8_t pin() const { return _pin; }

private:
  uint8_t _pin;

  // The timing parameters are kept in Config.h (shared for both sensors)
};
