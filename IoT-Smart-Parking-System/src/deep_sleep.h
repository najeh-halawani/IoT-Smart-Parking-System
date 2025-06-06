#pragma once
// #include <Arduino.h>
// #include <Preferences.h>
#include "config.h"
#include "debug.h"
#include "wifi_comms.h"


/* Functions used for the scheduled routine of Deep and Light Sleep States of the system */

/** 
 * @brief Check if the current time is within the deep sleep range.
 * 
 * @param currentHour The current hour (0-23).
 * @param currentMinute The current minute (0-59).
 * @return true if the current time is within the deep sleep range, false otherwise.
 */
bool isTimeInRange(int currentHour, int currentMinute) {
    DEBUG("SLEEP", "Checking if %02d:%02d is in sleep time range (%02d:00-%02d:%02d)", currentHour, currentMinute, DEEP_SLEEP_START_HOUR, DEEP_SLEEP_END_HOUR, DEEP_SLEEP_END_MINUTE);                
    if (currentHour < DEEP_SLEEP_START_HOUR || currentHour > DEEP_SLEEP_END_HOUR) {
      DEBUG("SLEEP", "Not in deep sleep time range");
      return false;
    }
    if (currentHour == DEEP_SLEEP_END_HOUR && currentMinute >= DEEP_SLEEP_END_MINUTE) {
      DEBUG("SLEEP", "Not in deep sleep time range");
      return false;
    }
    DEBUG("SLEEP", "In deep sleep time range");
    return true;
}


void syncInternalRTC(unsigned long lastSuccessfulConnection, unsigned long connectionAttempts) {
  WiFiUDP ntpUDP;
  NTPClient timeClient(ntpUDP, "pool.ntp.org", NTP_OFFSET, 60000);
  
  DEBUG("TIME", "Synchronizing internal RTC with NTP");
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi(lastSuccessfulConnection, connectionAttempts);
  }
  if (WiFi.status() == WL_CONNECTED) {
    timeClient.begin();
    if (timeClient.update()) {
      time_t now = timeClient.getEpochTime();
      struct tm timeinfo;
      localtime_r(&now, &timeinfo);
      
      struct timeval tv = { .tv_sec = now, .tv_usec = 0 };
      settimeofday(&tv, NULL);
      DEBUG("TIME", "RTC synchronized: %04d-%02d-%02d %02d:%02d:%02d", 
                 timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                 timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
      logError("NTP update failed");
    }
    WiFi.disconnect(true);
    delay(100);
  } else {
    logError("Failed to sync RTC: WiFi not connected");
  }
}

/* ----- FreeRTOS task function for sleep management ----- */
// ----------------------------------------------------------

// Struct to hold task parameters
struct SleepTaskParams {
  Preferences* prefs;
  DistanceSensor** sensors;
  int sensorCount;
};

// FreeRTOS task function
void deepSleepTask(void* pvParameters) {
  auto* params = static_cast<SleepTaskParams*>(pvParameters);
  Preferences& prefs = *params->prefs;
  DistanceSensor** sensors = params->sensors;
  int sensorCount = params->sensorCount;

  DEBUG("SLEEP", "Sleep task started on core %d", xPortGetCoreID());

  prefs.begin("sleep-config", false);
  int wakeHour = prefs.getInt("wake-hour", DEEP_SLEEP_END_HOUR);
  int wakeMinute = prefs.getInt("wake-min", DEEP_SLEEP_END_MINUTE);
  DEBUG("SLEEP", "Wake time: %02d:%02d", wakeHour, wakeMinute);

  while (true) {
    time_t now = time(nullptr);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    if (!isTimeInRange(timeinfo.tm_hour, timeinfo.tm_min)) {
      // Not yet in deep sleep window — calculate time until it starts
      struct tm sleepStartTime = timeinfo;
      sleepStartTime.tm_hour = DEEP_SLEEP_START_HOUR;
      sleepStartTime.tm_min = 0;
      sleepStartTime.tm_sec = 0;

      // If we're past today's sleep start, schedule for tomorrow
      if (mktime(&sleepStartTime) <= now) {
        sleepStartTime.tm_mday += 1;
      }

      time_t target = mktime(&sleepStartTime);
      int64_t waitSec = target - now;

      DEBUG("SLEEP", "Not in sleep window. Waiting %lld seconds until %02d:00", waitSec, DEEP_SLEEP_START_HOUR);
      vTaskDelay(pdMS_TO_TICKS(waitSec * 1000));
      continue;
    }

    // Inside sleep range — calculate how long to sleep
    struct tm wakeupTime = timeinfo;
    wakeupTime.tm_hour = wakeHour;
    wakeupTime.tm_min = wakeMinute;
    wakeupTime.tm_sec = 0;

    if (timeinfo.tm_hour > wakeHour || 
       (timeinfo.tm_hour == wakeHour && timeinfo.tm_min >= wakeMinute)) {
      wakeupTime.tm_mday += 1;  // schedule for tomorrow
    }

    int64_t sleepSec = mktime(&wakeupTime) - now;
    DEBUG("SLEEP", "In sleep window. Will sleep for %lld seconds until %02d:%02d", sleepSec, wakeHour, wakeMinute);

    if (sleepSec > 0 && sleepSec < 86400) {
      for (int i = 0; i < sensorCount; i++) {
        sensors[i]->saveState(prefs);
      }
      prefs.putBool("firstboot", false);
      prefs.end();

      esp_sleep_enable_timer_wakeup(sleepSec * 1000000ULL);
      Serial.flush();
      esp_deep_sleep_start();
    } else {
      DEBUG("SLEEP", "Invalid sleep duration, retrying in 60s");
      vTaskDelay(pdMS_TO_TICKS(60000));
    }
  }
}