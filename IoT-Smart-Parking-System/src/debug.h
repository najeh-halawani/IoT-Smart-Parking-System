#pragma once
#include <Arduino.h>

#ifdef DEBUG_MODE
  #define DEBUG(tag, format, ...) Serial.printf("[%s] " format "\n", tag, ##__VA_ARGS__)
#else
  #define DEBUG(tag, format, ...)
#endif