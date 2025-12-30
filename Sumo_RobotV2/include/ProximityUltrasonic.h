#pragma once
#include <Arduino.h>
#include <PinChangeInterrupt.h>

class ProximityUltrasonic {
public:
  struct DistancesCm {
    float left  = 0.0f;
    float front = 0.0f;
    float right = 0.0f;
  };

  ProximityUltrasonic(uint8_t trigPin,
                      uint8_t echoLeftPin,
                      uint8_t echoFrontPin,
                      uint8_t echoRightPin);

  // Call once in setup()
  void begin();

  // Call fast in loop() (non-blocking)
  void update();

  // Get last distances (cm)
  const DistancesCm& getDistancesCm() const { return m_dist; }

  // ---- Optional tuning (set any time) ----
  void setMaxDistanceCm(uint16_t v);         // default 400
  void setMinValidCm(uint16_t v);            // default 2
  void setTriggerPeriodMs(uint32_t v);       // default 60
  void setStaleForceFarMs(uint32_t v);       // default 400

private:
  struct EchoState {
    volatile uint32_t tRiseUs = 0;
    volatile uint32_t pulseUs = 0;
    volatile bool     waitingFall = false;
    volatile bool     newPulseReady = false;

    float    lastGoodCm = 0.0f;
    uint32_t lastGoodMs = 0;
  };

  // Pins
  uint8_t m_trigPin;
  uint8_t m_echoLeftPin;
  uint8_t m_echoFrontPin;
  uint8_t m_echoRightPin;

  // Config
  uint16_t m_maxDistanceCm    = 400;
  uint16_t m_minValidCm       = 2;
  uint32_t m_triggerPeriodMs  = 60;
  uint32_t m_staleForceFarMs  = 400;

  // Derived
  float    m_usToCm           = 1.0f / 58.0f;
  uint32_t m_echoTimeoutUs    = 400UL * 58UL;

  // State
  EchoState m_left;
  EchoState m_front;
  EchoState m_right;

  uint32_t m_lastTriggerMs = 0;

  DistancesCm m_dist;

  // Singleton for PCINT callbacks
  static ProximityUltrasonic* s_instance;

private:
  // ISR glue
  static void isrLeft();
  static void isrFront();
  static void isrRight();

  void handleEchoChange(EchoState &s, uint8_t pin);

  // Core logic
  void triggerAll();
  void updateSensor(EchoState &s);
  bool isValidCm(float cm) const;

  void recomputeDerived();
};



/* #pragma once
#include <Arduino.h>

class ProximityUltrasonic {
public:
  // index must be 0,1,2 (left/front/right) for ISR mapping
  ProximityUltrasonic(uint8_t trigPin, uint8_t echoPin, uint8_t index);

  void begin();
  void triggerNow();                 // send a trigger pulse (short)
  void update();                     // process new pulses + timeouts + stale logic
  float getCm() const;               // always returns a number (never NA)

  uint8_t echoPin() const { return _echoPin; }
  uint8_t index()  const { return _index;  }

  // Called from ISR trampoline
  void onEchoChangeISR();

  // Static ISR trampolines (mapped to index 0..2)
  static void isr0();
  static void isr1();
  static void isr2();

private:
  bool isValidCm(float cm) const;

private:
  uint8_t _trigPin;
  uint8_t _echoPin;
  uint8_t _index;

  // ISR-written fields
  volatile uint32_t _tRiseUs = 0;
  volatile uint32_t _pulseUs = 0;
  volatile bool     _waitingFall = false;
  volatile bool     _newPulseReady = false;

  // Loop-side fields
  float    _lastGoodCm = 0.0f;
  uint32_t _lastGoodMs = 0;

  // Instance registry for ISR trampolines
  static ProximityUltrasonic* _instances[3];
};
 */