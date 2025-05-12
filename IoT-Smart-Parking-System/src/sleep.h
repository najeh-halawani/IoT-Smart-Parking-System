#pragma once
#include "config.h"

/* Functions used for the scheduled routine of Deep and Light Sleep States of the system */

bool isTimeInRange(int currentHour, int currentMinute) {
    DEBUG(SLEEP, "Checking if %02d:%02d is in sleep time range (%02d:00-%02d:%02d)", currentHour, currentMinute, DEEP_SLEEP_START_HOUR, DEEP_SLEEP_END_HOUR, DEEP_SLEEP_END_MINUTE);
                
    if (currentHour < DEEP_SLEEP_START_HOUR || currentHour > DEEP_SLEEP_END_HOUR) {
      return false;
    }
  
    if (currentHour == DEEP_SLEEP_END_HOUR && currentMinute >= DEEP_SLEEP_END_MINUTE) {
      return false;
    }
  
    return true;
  }