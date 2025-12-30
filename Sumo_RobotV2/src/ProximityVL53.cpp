#include "ProximityVL53.h"

ProximityVL53::ProximityVL53(uint8_t xshutRightPin,
                             uint8_t xshutFrontPin,
                             uint8_t xshutLeftPin,
                             uint8_t addrRight,
                             uint8_t addrFront,
                             uint8_t addrLeft)
: m_xshutRightPin(xshutRightPin),
  m_xshutFrontPin(xshutFrontPin),
  m_xshutLeftPin(xshutLeftPin),
  m_addrRight(addrRight),
  m_addrFront(addrFront),
  m_addrLeft(addrLeft)
{
  // defaults already set
}

bool ProximityVL53::begin() {
  // Keep "no object" aligned to max valid unless user changed it explicitly
  if (m_noObjectMm == 0) {
    m_noObjectMm = m_maxValidMm;
  }

  disableAll();
  delay(10); // setup-only

  bool okRight = initSingle(m_right, m_xshutRightPin, m_addrRight);
  bool okFront = initSingle(m_front, m_xshutFrontPin, m_addrFront);
  bool okLeft  = initSingle(m_left,  m_xshutLeftPin,  m_addrLeft);

  // Keep all enabled
  digitalWrite(m_xshutRightPin, HIGH);
  digitalWrite(m_xshutFrontPin, HIGH);
  digitalWrite(m_xshutLeftPin,  HIGH);

  // Initialize outputs as "no object"
  m_distCm.right = mmToCm(m_noObjectMm);
  m_distCm.front = mmToCm(m_noObjectMm);
  m_distCm.left  = mmToCm(m_noObjectMm);

  return (okRight && okFront && okLeft);
}

void ProximityVL53::update() {
  const uint16_t rMm = readSafeMm(m_right);
  const uint16_t fMm = readSafeMm(m_front);
  const uint16_t lMm = readSafeMm(m_left);

  m_distCm.right = mmToCm(rMm);
  m_distCm.front = mmToCm(fMm);
  m_distCm.left  = mmToCm(lMm);
}

void ProximityVL53::disableAll() {
  pinMode(m_xshutRightPin, OUTPUT);
  pinMode(m_xshutFrontPin, OUTPUT);
  pinMode(m_xshutLeftPin,  OUTPUT);

  digitalWrite(m_xshutRightPin, LOW);
  digitalWrite(m_xshutFrontPin, LOW);
  digitalWrite(m_xshutLeftPin,  LOW);
}

void ProximityVL53::enableSingle(uint8_t pin) {
  digitalWrite(pin, HIGH);
}

bool ProximityVL53::initSingle(VL53L0X &sensor, uint8_t xshutPin, uint8_t newAddr) {
  enableSingle(xshutPin);
  delay(10); // setup-only

  if (!sensor.init()) {
    return false;
  }

  sensor.setTimeout(m_timeoutMs);
  sensor.setAddress(newAddr);
  sensor.startContinuous(m_continuousIntervalMs);

  return true;
}

uint16_t ProximityVL53::readSafeMm(VL53L0X &sensor) {
  const uint16_t d = sensor.readRangeContinuousMillimeters();

  if (sensor.timeoutOccurred()) {
    return m_noObjectMm;
  }

  if (d < m_minValidMm) {
    return m_minValidMm;      // <-- clamp close values instead of NO OBJECT
  }

  if (d > m_maxValidMm) {
    return m_noObjectMm;
  }

  return d;
}

