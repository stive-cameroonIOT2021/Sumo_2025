#include "ProximityUltrasonic.h"

ProximityUltrasonic* ProximityUltrasonic::s_instance = nullptr;

ProximityUltrasonic::ProximityUltrasonic(uint8_t trigPin,
                                         uint8_t echoLeftPin,
                                         uint8_t echoFrontPin,
                                         uint8_t echoRightPin)
: m_trigPin(trigPin),
  m_echoLeftPin(echoLeftPin),
  m_echoFrontPin(echoFrontPin),
  m_echoRightPin(echoRightPin)
{
  recomputeDerived();

  // start with FAR until first good reading arrives
  m_left.lastGoodCm  = (float)m_maxDistanceCm;
  m_front.lastGoodCm = (float)m_maxDistanceCm;
  m_right.lastGoodCm = (float)m_maxDistanceCm;

  m_dist.left  = m_left.lastGoodCm;
  m_dist.front = m_front.lastGoodCm;
  m_dist.right = m_right.lastGoodCm;
}

void ProximityUltrasonic::recomputeDerived() {
  m_usToCm        = 1.0f / 58.0f;
  m_echoTimeoutUs = (uint32_t)m_maxDistanceCm * 58UL;
}

void ProximityUltrasonic::setMaxDistanceCm(uint16_t v) {
  if (v == 0) v = 1;
  m_maxDistanceCm = v;
  recomputeDerived();
}

void ProximityUltrasonic::setMinValidCm(uint16_t v) {
  m_minValidCm = v;
}

void ProximityUltrasonic::setTriggerPeriodMs(uint32_t v) {
  if (v < 10) v = 10; // safety
  m_triggerPeriodMs = v;
}

void ProximityUltrasonic::setStaleForceFarMs(uint32_t v) {
  m_staleForceFarMs = v;
}

void ProximityUltrasonic::begin() {
  s_instance = this;

  pinMode(m_trigPin, OUTPUT);
  digitalWrite(m_trigPin, LOW);

  pinMode(m_echoLeftPin, INPUT);
  pinMode(m_echoFrontPin, INPUT);
  pinMode(m_echoRightPin, INPUT);

  attachPCINT(digitalPinToPCINT(m_echoLeftPin),  ProximityUltrasonic::isrLeft,  CHANGE);
  attachPCINT(digitalPinToPCINT(m_echoFrontPin), ProximityUltrasonic::isrFront, CHANGE);
  attachPCINT(digitalPinToPCINT(m_echoRightPin), ProximityUltrasonic::isrRight, CHANGE);

  const uint32_t now = millis();
  m_left.lastGoodMs  = now;
  m_front.lastGoodMs = now;
  m_right.lastGoodMs = now;

  m_lastTriggerMs = now;
}

void ProximityUltrasonic::isrLeft()  { if (s_instance) s_instance->handleEchoChange(s_instance->m_left,  s_instance->m_echoLeftPin); }
void ProximityUltrasonic::isrFront() { if (s_instance) s_instance->handleEchoChange(s_instance->m_front, s_instance->m_echoFrontPin); }
void ProximityUltrasonic::isrRight() { if (s_instance) s_instance->handleEchoChange(s_instance->m_right, s_instance->m_echoRightPin); }

void ProximityUltrasonic::handleEchoChange(EchoState &s, uint8_t pin) {
  const bool level = digitalRead(pin);
  const uint32_t now = micros();

  if (level) {
    // rising edge
    s.tRiseUs = now;
    s.waitingFall = true;
  } else {
    // falling edge
    if (s.waitingFall) {
      s.pulseUs = now - s.tRiseUs;
      s.waitingFall = false;
      s.newPulseReady = true;
    }
  }
}

void ProximityUltrasonic::triggerAll() {
  digitalWrite(m_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(m_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(m_trigPin, LOW);
}

bool ProximityUltrasonic::isValidCm(float cm) const {
  return (cm >= (float)m_minValidCm) && (cm <= (float)m_maxDistanceCm);
}

void ProximityUltrasonic::updateSensor(EchoState &s) {
  // 1) If we got a complete pulse, process it
  if (s.newPulseReady) {
    uint32_t p;
    noInterrupts();
    p = s.pulseUs;
    s.newPulseReady = false;
    interrupts();

  if (p > 0 && p <= m_echoTimeoutUs) {
    float cm = (float)p * m_usToCm;

    // ---- FIX: if too close, CLAMP instead of letting it go stale -> FAR ----
    if (cm < (float)m_minValidCm) {
      cm = (float)m_minValidCm;      // or 1.0f if you prefer
      s.lastGoodCm = cm;
      s.lastGoodMs = millis();
      return;                        // done
    }

    // normal valid range
    if (cm <= (float)m_maxDistanceCm) {
      s.lastGoodCm = cm;
      s.lastGoodMs = millis();
    }
    // else: too far -> ignore (stale logic will push to FAR)
  }

    // else: ignore timeout-like pulse
  }

  // 2) If we saw rising but never saw falling, timeout it safely
  if (s.waitingFall) {
    const uint32_t nowUs = micros();
    const uint32_t ageUs = nowUs - s.tRiseUs;
    if (ageUs > m_echoTimeoutUs) {
      noInterrupts();
      s.waitingFall = false;
      interrupts();
      // timeout -> keep lastGoodCm
    }
  }

  // 3) If too long since last good reading, force FAR
  const uint32_t nowMs = millis();
  if ((nowMs - s.lastGoodMs) > m_staleForceFarMs) {
    // If we were very close recently, keep that instead of jumping to FAR
    // (prevents 2cm -> 400cm popping)
    constexpr float CLOSE_HOLD_CM = 8.0f;

    if (s.lastGoodCm <= CLOSE_HOLD_CM) {
      // keep lastGoodCm, do NOT force far
    } else {
      s.lastGoodCm = (float)m_maxDistanceCm;
    }
  }

}

void ProximityUltrasonic::update() {
  const uint32_t nowMs = millis();

  // Periodic common trigger
  if (nowMs - m_lastTriggerMs >= m_triggerPeriodMs) {
    m_lastTriggerMs = nowMs;
    triggerAll();
  }

  // Update all sensors (non-blocking)
  updateSensor(m_left);
  updateSensor(m_front);
  updateSensor(m_right);

  // Publish distances
  m_dist.left  = m_left.lastGoodCm;
  m_dist.front = m_front.lastGoodCm;
  m_dist.right = m_right.lastGoodCm;
}



/* #include "ProximityUltrasonic.h"
#include "Config.h"
#include "Pins.h"

// Requires library: PinChangeInterrupt
#include <PinChangeInterrupt.h>

ProximityUltrasonic* ProximityUltrasonic::_instances[3] = { nullptr, nullptr, nullptr };

ProximityUltrasonic::ProximityUltrasonic(uint8_t trigPin, uint8_t echoPin, uint8_t index)
: _trigPin(trigPin), _echoPin(echoPin), _index(index)
{
  if (_index < 3) _instances[_index] = this;
}

void ProximityUltrasonic::begin()
{
  pinMode(_trigPin, OUTPUT);
  digitalWrite(_trigPin, LOW);

  pinMode(_echoPin, INPUT);

  // Start with FAR so your robot doesn't freak out at boot
  _lastGoodCm = (float)US_MAX_DISTANCE_CM;
  _lastGoodMs = millis();

  // Attach PCINT ISR based on index
  switch (_index) {
    case 0: attachPCINT(digitalPinToPCINT(_echoPin), ProximityUltrasonic::isr0, CHANGE); break;
    case 1: attachPCINT(digitalPinToPCINT(_echoPin), ProximityUltrasonic::isr1, CHANGE); break;
    case 2: attachPCINT(digitalPinToPCINT(_echoPin), ProximityUltrasonic::isr2, CHANGE); break;
    default: / unsupported index / break;
  }
}

void ProximityUltrasonic::triggerNow()
{
  // HC-SR04 needs ~10us HIGH pulse
  digitalWrite(_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigPin, LOW);
}

bool ProximityUltrasonic::isValidCm(float cm) const
{
  return (cm >= (float)US_MIN_VALID_CM) && (cm <= (float)US_MAX_DISTANCE_CM);
}

void ProximityUltrasonic::update()
{
  // 1) Consume new pulse if available
  if (_newPulseReady) {
    uint32_t p;
    noInterrupts();
    p = _pulseUs;
    _newPulseReady = false;
    interrupts();

    if (p > 0 && p <= US_ECHO_TIMEOUT_US) {
      float cm = p * US_US_TO_CM;
      if (isValidCm(cm)) {
        _lastGoodCm = cm;
        _lastGoodMs = millis();
      }
      // else: invalid -> ignore (hold last good)
    }
    // else: timeout -> ignore (hold last good)
  }

  // 2) If we saw a rising edge but no falling edge, cancel on timeout
  if (_waitingFall) {
    uint32_t nowUs = micros();
    uint32_t ageUs = nowUs - _tRiseUs;
    if (ageUs > US_ECHO_TIMEOUT_US) {
      noInterrupts();
      _waitingFall = false;
      interrupts();
      // hold last good
    }
  }

  // 3) If too stale, force FAR (optional safety)
  uint32_t nowMs = millis();
  if ((nowMs - _lastGoodMs) > US_STALE_FORCE_FAR_MS) {
    _lastGoodCm = (float)US_MAX_DISTANCE_CM;
  }
}

float ProximityUltrasonic::getCm() const
{
  return _lastGoodCm; // never NA
}

void ProximityUltrasonic::onEchoChangeISR()
{
  bool level = digitalRead(_echoPin);
  uint32_t now = micros();

  if (level) {
    // Rising edge
    _tRiseUs = now;
    _waitingFall = true;
  } else {
    // Falling edge
    if (_waitingFall) {
      _pulseUs = now - _tRiseUs;
      _waitingFall = false;
      _newPulseReady = true;
    }
  }
}

// ---- ISR trampolines ----
void ProximityUltrasonic::isr0() { if (_instances[0]) _instances[0]->onEchoChangeISR(); }
void ProximityUltrasonic::isr1() { if (_instances[1]) _instances[1]->onEchoChangeISR(); }
void ProximityUltrasonic::isr2() { if (_instances[2]) _instances[2]->onEchoChangeISR(); }
 */