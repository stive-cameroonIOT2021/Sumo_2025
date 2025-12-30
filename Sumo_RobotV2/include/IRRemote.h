#pragma once
#include <Arduino.h>

// Small wrapper around Arduino-IRremote (IRremote.hpp)
class IRRemote {
public:
  IRRemote() = default;

  void begin(uint8_t rxPin, bool disableLedFeedback = true);

  // Call often in loop
  void update();

  // True only when a NEW (non-repeat) command arrived since last consume()
  bool hasNewCommand() const { return _newCmd; }

  // Last received command byte (NEC etc.)
  uint8_t command() const { return _cmd; }

  // Returns true if a new command exists and matches, and consumes it.
  bool consumeIf(uint8_t cmd);

  // Consume the “new” flag (keep last cmd value)
  void consume() { _newCmd = false; }

private:
  uint8_t _pin   = 255;
  uint8_t _cmd   = 0xFF;
  bool    _newCmd = false;

  // optional small “debounce” for rapid frames
  uint32_t _lastAcceptMs = 0;
  uint16_t _minGapMs     = 120;  // ignore new commands arriving too fast
};
