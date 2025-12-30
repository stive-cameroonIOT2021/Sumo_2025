#pragma once
#include <Arduino.h>

/* ===================== PROX SENSOR SELECT ===================== */
// Choose ONE (0/1)
#define CONFIG_USE_ULTRASONIC  1
#define CONFIG_USE_VL53        0

#if (CONFIG_USE_ULTRASONIC + CONFIG_USE_VL53) != 1
  #error "Select exactly one: CONFIG_USE_ULTRASONIC or CONFIG_USE_VL53"
#endif

namespace Config {

  // invert because of wiring / mounting
  constexpr bool invertLeftMotorDirection  = false;//false;
  constexpr bool invertRightMotorDirection = false;//false;//true;

  // QTR timing values used by LineSensorQTR.cpp  (MUST EXIST)
  static constexpr uint16_t QTR_CHARGE_TIME_US   = 10;
  static constexpr uint16_t QTR_TIMEOUT_US       = 3000;
  static constexpr uint16_t QTR_SAMPLE_PERIOD_MS = 10;
  static constexpr uint16_t QTR_WHITE_TH_US      = 800; // tune (optional)

  // IR sequence lock
  constexpr uint8_t  IR_CMD_START_1      = 0x45;
  constexpr uint8_t  IR_CMD_STOP_1       = 0x47;
  constexpr uint8_t  IR_CMD_CONFIRM      = 0x46;
  constexpr uint32_t IR_SEQ_WINDOW_MS    = 800UL;

  extern uint8_t       g_irPendingCmd;
  extern unsigned long g_irPendingMs;

  constexpr uint32_t START_DELAY_MS       = 000UL;

  enum StrategyMode : uint8_t { STRAT_SEARCH = 0, STRAT_SPIN_LEFT = 1, STRAT_SPIN_RIGHT = 2 };
  extern StrategyMode g_strategy;

  constexpr uint16_t QTR_THRESHOLD        = 1000;
  constexpr uint8_t  LINE_CONFIRM_COUNT   = 3;
  constexpr uint32_t OUT_TOO_LONG_MS      = 3000UL;

  constexpr uint32_t BRAKE_TIME_MS        = 60UL;
  constexpr uint32_t BACKUP_TIME_MS       = 100U;//300UL;
  constexpr uint32_t BACKUP_TIME_BOTH_MS  = 100UL;//380UL;
  constexpr uint32_t TURN_AWAY_MS         = 90UL;//280UL;
  constexpr int16_t  SPEED_SURVIVE        = 200;//125;

  constexpr int16_t  SPEED_SEARCH         = 40;//Search speed

  constexpr int16_t  SPEED_SPIN_SCAN      = 150;//100;
  constexpr int16_t  SPEED_SPIN_CREEP     = 25;

  constexpr float    DETECT_RANGE_CM      = 18.0f;

  constexpr int16_t  SPEED_ATTACK_BASE    = 170;
  constexpr int16_t  SPEED_ATTACK_MAX     = 250;
  constexpr int16_t  SPEED_TURN_ATTACK    = 200;//190;

  constexpr uint32_t PUSH_BOOST_AFTER_MS  = 900UL;
  constexpr uint32_t PUSH_GIVEUP_AFTER_MS = 2200UL;

  constexpr int16_t  REBACK_FAST          = 50;//90;
  constexpr int16_t  REBACK_SLOW          = 30;//80;
  constexpr uint32_t REATTACK_BACK_MS     = 260UL;
  constexpr uint32_t REATTACK_CHECK_MS    = 120UL;
  constexpr uint32_t REATTACK_TURN_MS     = 180UL;

  constexpr uint32_t DEBUG_PERIOD_MS      = 200UL;
}
