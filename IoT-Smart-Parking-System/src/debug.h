#pragma once
#include <Arduino.h>

#ifdef DEBUG_MODE
  #define DEBUG(tag, format, ...) Serial.printf("[" #tag "] " format "\n", ##__VA_ARGS__)
#else
  #define DEBUG(tag, format, ...)
#endif