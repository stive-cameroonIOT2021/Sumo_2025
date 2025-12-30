#pragma once
#include <Arduino.h>

class MotorDriver {
public:
  struct MotorPins {
    uint8_t pwm;
    uint8_t in1;
    uint8_t in2;
  };

  MotorDriver(MotorPins left, MotorPins right);

  void begin();
  void setInvert(bool invertLeft, bool invertRight);

  // + = forward, - = backward (robot coordinates)
  void drive(int16_t leftSpeed, int16_t rightSpeed);

  // Optional helper
  void stop();

private:
  MotorPins _L, _R;
  bool _invertL = false;
  bool _invertR = false;

  static int16_t clamp(int16_t v, int16_t lo, int16_t hi);

  void driveOne(const MotorPins& m, int16_t speed);
};
