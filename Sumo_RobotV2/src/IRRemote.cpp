#include "IRRemote.h"
#include <IRremote.hpp>

void IRRemote::begin(uint8_t rxPin, bool disableLedFeedback) {
  _pin = rxPin;

  // IRremote uses internal globals
  if (disableLedFeedback) {
    IrReceiver.begin(_pin, DISABLE_LED_FEEDBACK);
  } else {
    IrReceiver.begin(_pin, ENABLE_LED_FEEDBACK);
  }

  _cmd = 0xFF;
  _newCmd = false;
  _lastAcceptMs = 0;
}

void IRRemote::update() {
  if (!IrReceiver.decode()) return;

  auto &d = IrReceiver.decodedIRData;

  // Ignore repeats — you asked for “real non-repeat frame only”
  const bool isRepeat = (d.flags & IRDATA_FLAGS_IS_REPEAT);

  if (!isRepeat) {
    const uint32_t now = millis();
    if ((now - _lastAcceptMs) >= _minGapMs) {
      _cmd = (uint8_t)d.command;
      _newCmd = true;
      _lastAcceptMs = now;
    }
  }

  IrReceiver.resume();
}

bool IRRemote::consumeIf(uint8_t cmd) {
  if (_newCmd && _cmd == cmd) {
    _newCmd = false;
    return true;
  }
  return false;
}
