#pragma once
#include <Preferences.h>

void sleepTask(void* pvParameters) {
  DEBUG(SLEEP, "Sleep task started on core %d", xPortGetCoreID());

  // Initialize Preferences
  preferences.begin("sleep-config", false);
  int wakeHour = preferences.getInt("wake-hour", DEEP_SLEEP_END_HOUR);
  int wakeMinute = preferences.getInt("wake-min", DEEP_SLEEP_END_MINUTE);
  DEBUG(SLEEP, "Wake time: %02d:%02d", wakeHour, wakeMinute);

  while (true) {
    // Get current time
    struct tm timeinfo;
    time_t now = time(nullptr);
    localtime_r(&now, &timeinfo);
    int hour = timeinfo.tm_hour;
    int minute = timeinfo.tm_min;
    DEBUG(SLEEP, "Current time: %02d:%02d:%02d", hour, minute, timeinfo.tm_sec);

    if (isTimeInRange(hour, minute)) {
      struct tm wakeupTime = timeinfo;
      wakeupTime.tm_hour = wakeHour;
      wakeupTime.tm_min = wakeMinute;
      wakeupTime.tm_sec = 0;

      if (hour > wakeHour || (hour == wakeHour && minute >= wakeMinute)) {
        wakeupTime.tm_mday += 1;
        DEBUG(SLEEP, "Wake time already passed, scheduling for tomorrow.");
      }

      time_t wakeupEpoch = mktime(&wakeupTime);
      int64_t sleepSec = wakeupEpoch - now;
      DEBUG(SLEEP, "Sleep duration: %llds", sleepSec);

      if (sleepSec > 0 && sleepSec < 86400) {
        DEBUG(SLEEP, "Saving state before deep sleep");
        preferences.putBool("firstboot", false);
        for (int i = 0; i < NUM_SENSORS; i++) {
          preferences.putBool(("state" + String(i)).c_str(), usSensorsLastState[i]);
          preferences.putBool(("state" + String(i) + "_tof").c_str(), tofSensorsLastState[i]);
          DEBUG(SLEEP, "Sensor %d saved: %s", i, usSensorsLastState[i] ? "OCCUPIED" : "VACANT");
          DEBUG(SLEEP, "Sensor %d TOF saved: %s", i, tofSensorsLastState[i] ? "OCCUPIED" : "VACANT");
        }
        preferences.end();

        esp_sleep_enable_timer_wakeup(sleepSec * 1000000ULL);
        DEBUG(SLEEP, "Entering deep sleep until %02d:%02d", wakeHour, wakeMinute);
        Serial.flush();
        esp_deep_sleep_start();
      } else {
        DEBUG(ERROR, "Invalid sleep duration: %llds", sleepSec);
      }
    } else {
      DEBUG(SLEEP, "Not deep sleep time. Light sleep for 1s.");
      esp_sleep_enable_timer_wakeup(LIGHT_SLEEP_DURATION * 1000000ULL);
      esp_light_sleep_start();
    }

    vTaskDelay(pdMS_TO_TICKS(60000));  // Delay task by 1 min
  }
}