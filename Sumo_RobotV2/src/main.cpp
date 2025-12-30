#include <Arduino.h>
#include <Wire.h>

#include "MotorDriver.h"
#include "LineSensorQTR.h"
#include "Config.h"
#include "IRRemote.h"
#include "Pins.h"

#if CONFIG_USE_ULTRASONIC
  #include "ProximityUltrasonic.h"
#elif CONFIG_USE_VL53
  #include "ProximityVL53.h"
#endif

#define DEBUG_ENABLED 1
#include "Debug.h"

IRRemote ir;

/* ===================== 3. OBJECTS ===================== */
LineSensorQTR qtrL(Pins::QTR_LEFT_PIN);
LineSensorQTR qtrR(Pins::QTR_RIGHT_PIN);

MotorDriver motors(
  MotorDriver::MotorPins{ Pins::LEFT_MOTOR_PWM_PIN,  Pins::LEFT_MOTOR_IN1_PIN,  Pins::LEFT_MOTOR_IN2_PIN },
  MotorDriver::MotorPins{ Pins::RIGHT_MOTOR_PWM_PIN, Pins::RIGHT_MOTOR_IN1_PIN, Pins::RIGHT_MOTOR_IN2_PIN }
);

#if CONFIG_USE_ULTRASONIC
  ProximityUltrasonic prox(Pins::PIN_TRIG, Pins::PIN_ECHO_LEFT, Pins::PIN_ECHO_FRONT, Pins::PIN_ECHO_RIGHT);
  using ProxDistances = decltype(prox.getDistancesCm()); // has .front .left .right (int cm)
#elif CONFIG_USE_VL53
  ProximityVL53 prox(Pins::TOF_XSHUT_RIGHT_PIN, Pins::TOF_XSHUT_FRONT_PIN, Pins::TOF_XSHUT_LEFT_PIN, 0x30, 0x31, 0x32);
  using ProxDistances = ProximityVL53::DistancesCm;      // has .front .left .right (float cm)
#endif

/* ===================== 4. STATE ===================== */
enum RobotState {
  STATE_HALT,        // waiting for IR start or stopped by IR
  STATE_START_DELAY, // 5s delay after START
  STATE_BASE,        // strategy motion
  STATE_ATTACK,
  STATE_SURVIVE_BRAKE,
  STATE_SURVIVE_BACK,
  STATE_SURVIVE_TURN
};

RobotState currentState = STATE_HALT;
unsigned long stateStartTime = 0;

bool turnRightAfterBackup = true;
bool bothHitLine = false;
uint8_t g_lineCount = 0;

bool g_lineActive = false;
unsigned long g_lineStartMs = 0;

// Push tracking (for speed ramp)
bool g_pushingNow = false;
unsigned long g_pushStartMs = 0;

unsigned long lastDebugTime = 0;

// LED blink
unsigned long g_ledLastMs = 0;
bool g_ledOn = false;

/* ===================== 5. HELPERS ===================== */
static inline void enterState(RobotState s, unsigned long now) {
  currentState = s;
  stateStartTime = now;
}

// Works for both int(cm) and float(cm)
static inline bool enemyInRange(float cm) {
  return (cm > 0.0f && cm < Config::DETECT_RANGE_CM);
}

static inline bool enemyFront(const ProxDistances& d) { return enemyInRange((float)d.front); }
static inline bool enemyLeft (const ProxDistances& d) { return enemyInRange((float)d.left);  }
static inline bool enemyRight(const ProxDistances& d) { return enemyInRange((float)d.right); }

/* IR sequence lock decoder
   - Press START_1 then CONFIRM within window => startEvent = true
   - Press STOP_1  then CONFIRM within window => stopEvent  = true */
void updateIrSequence(unsigned long now, bool &startEvent, bool &stopEvent) {
  startEvent = false;
  stopEvent  = false;

  // Expire pending first step if too old
  if (Config::g_irPendingCmd != 0xFF && (now - Config::g_irPendingMs > Config::IR_SEQ_WINDOW_MS)) {
    Config::g_irPendingCmd = 0xFF;
  }

  // No new command? Nothing to do.
  if (!ir.hasNewCommand()) return;

  const uint8_t cmd = ir.command();
  ir.consume(); // consume the new-command flag

  DEBUG_PRINT(F("IR cmd: 0x"));
  DEBUG_PRINTLN(cmd, HEX);

  // First step: arm sequence
  if (cmd == Config::IR_CMD_START_1 || cmd == Config::IR_CMD_STOP_1) {
    Config::g_irPendingCmd = cmd;
    Config::g_irPendingMs  = now;
    return;
  }

  // Confirm step: must be within time window
  if (cmd == Config::IR_CMD_CONFIRM && Config::g_irPendingCmd != 0xFF) {
    if (Config::g_irPendingCmd == Config::IR_CMD_START_1) startEvent = true;
    if (Config::g_irPendingCmd == Config::IR_CMD_STOP_1)  stopEvent  = true;
    Config::g_irPendingCmd = 0xFF; // clear pending after confirm
    return;
  }

  // Any other key cancels pending (optional safety)
  Config::g_irPendingCmd = 0xFF;
}

bool checkLineSensorsConfirmed() {
  const uint16_t leftVal  = qtrL.readUs();
  const uint16_t rightVal = qtrR.readUs();

  const bool leftHit  = (leftVal  < Config::QTR_THRESHOLD);
  const bool rightHit = (rightVal < Config::QTR_THRESHOLD);

  if (leftHit || rightHit) {
    if (g_lineCount < 255) g_lineCount++;
  } else {
    g_lineCount = 0;
    bothHitLine = false;
    return false;
  }

  bothHitLine = (leftHit && rightHit);

  // If Left sensor hit, we should turn Right. If Right hit, turn Left.
  if (leftHit && !rightHit) turnRightAfterBackup = true;
  else if (rightHit && !leftHit) turnRightAfterBackup = false;
  else turnRightAfterBackup = !turnRightAfterBackup; // both hit: alternate

  return (g_lineCount >= Config::LINE_CONFIRM_COUNT);
}

void runBaseStrategy() {
  if (Config::g_strategy == Config::STRAT_SEARCH) {
    motors.drive(Config::SPEED_SEARCH, Config::SPEED_SEARCH);
  } else if (Config::g_strategy == Config::STRAT_SPIN_LEFT) {
    motors.drive(-Config::SPEED_SPIN_SCAN + Config::SPEED_SPIN_CREEP, Config::SPEED_SPIN_SCAN);
  } else {
    motors.drive(Config::SPEED_SPIN_SCAN, -Config::SPEED_SPIN_SCAN + Config::SPEED_SPIN_CREEP);
  }
}

void updateStatusLed(unsigned long now) {
  if (currentState == STATE_HALT) {
    constexpr uint32_t BLINK_MS = 250UL;
    if (now - g_ledLastMs >= BLINK_MS) {
      g_ledLastMs = now;
      g_ledOn = !g_ledOn;
      digitalWrite(Pins::LED_PIN, g_ledOn ? HIGH : LOW);
    }
  } else {
    digitalWrite(Pins::LED_PIN, HIGH);
    g_ledOn = true;
  }
}

static inline void dbgDist(float v) {
#if CONFIG_USE_VL53
  DEBUG_PRINT(v, 1);
#else
  DEBUG_PRINT((int)v);
#endif
}

void printSensorDebug(const ProxDistances& d) {
  if (millis() - lastDebugTime < Config::DEBUG_PERIOD_MS) return;
  lastDebugTime = millis();

  const uint16_t valL = qtrL.readUs();
  const uint16_t valR = qtrR.readUs();

  DEBUG_PRINT(F("STRAT:")); DEBUG_PRINT((int)Config::g_strategy);
  DEBUG_PRINT(F(" STATE:")); DEBUG_PRINT((int)currentState);

  DEBUG_PRINT(F(" | F:")); dbgDist((float)d.front);
  DEBUG_PRINT(F(" L:"));   dbgDist((float)d.left);
  DEBUG_PRINT(F(" R:"));   dbgDist((float)d.right);

  DEBUG_PRINT(F(" | QTR_L:")); DEBUG_PRINT(valL);
  DEBUG_PRINT(F(" R:")); DEBUG_PRINT(valR);

  DEBUG_PRINT(F(" | push:")); DEBUG_PRINT(g_pushingNow ? 1 : 0);
  DEBUG_PRINT(F(" tPush:"));
  DEBUG_PRINTLN(g_pushingNow ? (millis() - g_pushStartMs) : 0);
}

/* ===================== 6. SETUP ===================== */
void setup() {
  DEBUG_BEGIN(115200);
  DEBUG_PRINTLN(F("=== SUMO ROBOT STARTING ==="));

  pinMode(Pins::LED_PIN, OUTPUT);
  digitalWrite(Pins::LED_PIN, LOW);

  ir.begin(Pins::IR_RX_PIN, true);

  motors.begin();
  motors.setInvert(Config::invertLeftMotorDirection,
                   Config::invertRightMotorDirection);

  qtrL.begin();
  qtrR.begin();

#if CONFIG_USE_ULTRASONIC
  prox.begin();
  prox.setMaxDistanceCm(400);
#elif CONFIG_USE_VL53
  Wire.begin();
  const bool ok = prox.begin();
  DEBUG_PRINT(F("VL53 begin: "));
  DEBUG_PRINTLN(ok ? F("OK") : F("FAIL"));
#endif

  motors.stop();
  enterState(STATE_HALT, millis());
  DEBUG_PRINTLN(F("HALT: START=0x45 then 0x46. STOP=0x47 then 0x46."));
}

/* ===================== 7. LOOP ===================== */
void loop() {
  const unsigned long now = millis();

  /* ======= IR HANDLING (SEQUENCE-LOCKED) ======= */
  ir.update();

  bool startEvent = false;
  bool stopEvent  = false;
  updateIrSequence(now, startEvent, stopEvent);

  // STOP works ANYTIME (but requires 0x47 then 0x46)
  if (stopEvent) {
    DEBUG_PRINTLN(F("IR STOP(seq) -> HALT"));
    motors.stop();
    g_pushingNow = false;
    g_pushStartMs = 0;
    g_lineActive = false;
    g_lineCount = 0;
    Config::g_irPendingCmd = 0xFF;
    enterState(STATE_HALT, now);
  }

  // If halted: wait for START (0x45 then 0x46)
  if (currentState == STATE_HALT) {
    motors.stop();
    updateStatusLed(now);

    if (startEvent) {
      DEBUG_PRINTLN(F("IR START(seq) -> START DELAY"));
      enterState(STATE_START_DELAY, now);
    }
    return; // nothing else runs while halted
  }

  // robot active -> LED solid on
  updateStatusLed(now);

  /* ======= NORMAL ROBOT LOGIC ======= */
  prox.update();
  const ProxDistances d = prox.getDistancesCm();
  printSensorDebug(d);

  // START DELAY (non-blocking)
  if (currentState == STATE_START_DELAY) {
    motors.stop();
    if (now - stateStartTime >= Config::START_DELAY_MS) {
      DEBUG_PRINTLN(F("GO!"));
      enterState(STATE_BASE, now);
    }
    return;
  }

  // EDGE DETECTION ALWAYS WINS (except while already in survival)
  if (currentState != STATE_SURVIVE_BRAKE &&
      currentState != STATE_SURVIVE_BACK  &&
      currentState != STATE_SURVIVE_TURN) {

    const bool lineHit = checkLineSensorsConfirmed();

    if (lineHit) {
      if (!g_lineActive) {
        g_lineActive = true;
        g_lineStartMs = now;
      } else if (now - g_lineStartMs >= Config::OUT_TOO_LONG_MS) {
        motors.stop();
        while (true) { /* SAFETY STOP */ }
      }

      motors.stop();
      enterState(STATE_SURVIVE_BRAKE, now);
    } else {
      g_lineActive = false;
    }
  }

  switch (currentState) {

    case STATE_SURVIVE_BRAKE:
      motors.stop();
      if (now - stateStartTime >= Config::BRAKE_TIME_MS) enterState(STATE_SURVIVE_BACK, now);
      break;

    case STATE_SURVIVE_BACK: {
      motors.drive(-Config::SPEED_SURVIVE, -Config::SPEED_SURVIVE);
      const uint32_t backTime = bothHitLine ? Config::BACKUP_TIME_BOTH_MS : Config::BACKUP_TIME_MS;
      if (now - stateStartTime >= backTime) enterState(STATE_SURVIVE_TURN, now);
    } break;

    case STATE_SURVIVE_TURN:
      if (turnRightAfterBackup) motors.drive(Config::SPEED_SURVIVE, -Config::SPEED_SURVIVE);
      else                     motors.drive(-Config::SPEED_SURVIVE, Config::SPEED_SURVIVE);

      if (now - stateStartTime >= Config::TURN_AWAY_MS) {
        g_lineCount = 0;
        g_pushingNow = false;
        g_pushStartMs = 0;
        enterState(STATE_BASE, now);
      }
      break;

    case STATE_BASE: {
      const bool f = enemyFront(d);
      const bool l = enemyLeft(d);
      const bool r = enemyRight(d);

      if (f || l || r) {
        g_pushingNow = false;
        g_pushStartMs = 0;
        enterState(STATE_ATTACK, now);
      } else {
        runBaseStrategy();
      }
    } break;

    case STATE_ATTACK: {
      // Read again for freshest chase
      const ProxDistances dd = prox.getDistancesCm();

      const bool f = enemyFront(dd);
      const bool l = enemyLeft(dd);
      const bool r = enemyRight(dd);

      // Lost target -> go back to base strategy
      if (!f && !l && !r) {
        g_pushingNow = false;
        g_pushStartMs = 0;
        enterState(STATE_BASE, now);
        break;
      }

      // If FRONT: we are pushing -> ramp speed up to max and keep pushing
      if (f) {
        if (!g_pushingNow) {
          g_pushingNow = true;
          g_pushStartMs = now;
        }

        int16_t spd = Config::SPEED_ATTACK_BASE;

        // After PUSH_BOOST_AFTER_MS -> go to max (and stay max)
        if ((now - g_pushStartMs) >= Config::PUSH_BOOST_AFTER_MS) {
          spd = Config::SPEED_ATTACK_MAX;
        }

        motors.drive(spd, spd);
        break;
      }

      // Not front anymore -> reset push timer and chase side
      g_pushingNow = false;
      g_pushStartMs = 0;

      if (l) {
        motors.drive(-Config::SPEED_TURN_ATTACK, Config::SPEED_TURN_ATTACK);
      } else if (r) {
        motors.drive(Config::SPEED_TURN_ATTACK, -Config::SPEED_TURN_ATTACK);
      }
    } break;

    default:
      enterState(STATE_BASE, now);
      break;
  }
}
