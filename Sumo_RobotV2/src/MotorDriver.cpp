#include "MotorDriver.h"

MotorDriver::MotorDriver(MotorPins left, MotorPins right)
: _L(left), _R(right) {}

int16_t MotorDriver::clamp(int16_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void MotorDriver::begin() {
  pinMode(_L.pwm, OUTPUT);
  pinMode(_L.in1, OUTPUT);
  pinMode(_L.in2, OUTPUT);

  pinMode(_R.pwm, OUTPUT);
  pinMode(_R.in1, OUTPUT);
  pinMode(_R.in2, OUTPUT);

  stop();
}

void MotorDriver::setInvert(bool invertLeft, bool invertRight) {
  _invertL = invertLeft;
  _invertR = invertRight;
}

void MotorDriver::drive(int16_t leftSpeed, int16_t rightSpeed) {
  if (_invertL) leftSpeed = -leftSpeed;
  if (_invertR) rightSpeed = -rightSpeed;

  leftSpeed  = clamp(leftSpeed,  -255, 255);
  rightSpeed = clamp(rightSpeed, -255, 255);

  driveOne(_L, leftSpeed);
  driveOne(_R, rightSpeed);
}

void MotorDriver::stop() {
  drive(0, 0);
}

void MotorDriver::driveOne(const MotorPins& m, int16_t speed) {
  if (speed > 0) {
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, LOW);
    analogWrite(m.pwm, (uint8_t)speed);
  } else if (speed < 0) {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, HIGH);
    analogWrite(m.pwm, (uint8_t)(-speed));
  } else {
    // coast
    analogWrite(m.pwm, 0);
  }
}
