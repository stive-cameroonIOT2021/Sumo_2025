#include "Config.h"

namespace Config {
  uint8_t       g_irPendingCmd = 0xFF;
  unsigned long g_irPendingMs  = 0;

  StrategyMode  g_strategy     = STRAT_SEARCH;//STRAT_SPIN_RIGHT;//STRAT_SPIN_LEFT; // default strategy
}
