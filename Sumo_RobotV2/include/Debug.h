#pragma once
#include <Arduino.h>

// To enable debug, define this as 1 in your main file or Config.h
// To disable for competition, define as 0
#ifndef DEBUG_ENABLED
  #define DEBUG_ENABLED 0
#endif

#if DEBUG_ENABLED
  #define DEBUG_BEGIN(baud)        Serial.begin(baud)
  #define DEBUG_PRINT(...)         Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)       Serial.println(__VA_ARGS__)
#else
  #define DEBUG_BEGIN(baud)        do{}while(0)
  #define DEBUG_PRINT(...)         do{}while(0)
  #define DEBUG_PRINTLN(...)       do{}while(0)
#endif