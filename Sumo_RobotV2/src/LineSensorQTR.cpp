#include "LineSensorQTR.h"
#include "Config.h"

LineSensorQTR::LineSensorQTR(uint8_t pin) : _pin(pin) {}

void LineSensorQTR::begin() {
  // Nothing special; reading function handles pinMode changes.
  pinMode(_pin, INPUT);
}

// Fast, bounded RC timing read for QTR-1RC (blocking but short)
uint16_t LineSensorQTR::readUs() const {
  // 1) Charge the capacitor
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, HIGH);

  const unsigned long chargeStart = micros();
  while ((micros() - chargeStart) < Config::QTR_CHARGE_TIME_US) {
    // tight wait
  }

  // 2) Measure discharge time
  pinMode(_pin, INPUT);
  const unsigned long dischargeStart = micros();

  while (digitalRead(_pin) == HIGH) {
    const unsigned long elapsed = micros() - dischargeStart;
    if (elapsed >= Config::QTR_TIMEOUT_US) {
      return (uint16_t)Config::QTR_TIMEOUT_US;
    }
  }

  return (uint16_t)(micros() - dischargeStart);
}
