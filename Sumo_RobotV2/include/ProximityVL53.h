#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

class ProximityVL53 {
public:
  struct DistancesCm {
    float left  = 0.0f;
    float front = 0.0f;
    float right = 0.0f;
  };

  ProximityVL53(uint8_t xshutRightPin,
                uint8_t xshutFrontPin,
                uint8_t xshutLeftPin,
                uint8_t addrRight = 0x30,
                uint8_t addrFront = 0x31,
                uint8_t addrLeft  = 0x32);

  // Call once in setup AFTER Wire.begin()
  bool begin();

  // Call frequently in loop (non-blocking)
  void update();

  // Last valid distances in cm (no flags)
  const DistancesCm& getDistancesCm() const { return m_distCm; }

  // Optional tuning (set BEFORE begin())
  void setMinValidMm(uint16_t v)          { m_minValidMm = v; }
  void setMaxValidMm(uint16_t v)          { m_maxValidMm = v; }
  void setNoObjectDistanceMm(uint16_t v)  { m_noObjectMm = v; }
  void setTimeoutMs(uint16_t v)           { m_timeoutMs = v; }
  void setContinuousIntervalMs(uint32_t v){ m_continuousIntervalMs = v; }

private:
  // Pins
  uint8_t m_xshutRightPin;
  uint8_t m_xshutFrontPin;
  uint8_t m_xshutLeftPin;

  // Addresses
  uint8_t m_addrRight;
  uint8_t m_addrFront;
  uint8_t m_addrLeft;

  // Sensors
  VL53L0X m_right;
  VL53L0X m_front;
  VL53L0X m_left;

  // Config
  uint16_t m_minValidMm = 50;
  uint16_t m_maxValidMm = 1200;
  uint16_t m_noObjectMm = 1200;
  uint16_t m_timeoutMs  = 50;
  uint32_t m_continuousIntervalMs = 0;

  DistancesCm m_distCm;

private:
  void disableAll();
  void enableSingle(uint8_t pin);
  bool initSingle(VL53L0X &sensor, uint8_t xshutPin, uint8_t newAddr);
  uint16_t readSafeMm(VL53L0X &sensor);
  static float mmToCm(uint16_t mm) { return static_cast<float>(mm) * 0.1f; }
};
