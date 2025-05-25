#pragma once
#include <esp_task_wdt.h>

#include "DistanceSensor.h"
#include "sensor_laser.h"
#include "sensor_ultrasonic.h"
#include "config.h"
#include "debug.h"
/**
 * @brief Put the ESP32 into light sleep for a specified duration.
 * 
 * @param seconds Duration in seconds to sleep.
 */
inline void lightSleepForSeconds(uint32_t seconds) {
  esp_sleep_enable_timer_wakeup(seconds * 1000000ULL);
  esp_light_sleep_start();
}

/* 
* Struct to hold distance sensor readings for FreeRTOS tasks
*/
extern RTC_DATA_ATTR uint32_t lastSampleTime[NUM_SPOTS];

struct DistanceSensorReading {
  int sensorId;
  float distance;
};

/*
* Struct to hold parameters for sensor tasks (sensor array and queue)
*/
struct SensorTaskParams {
  DistanceSensor** sensors;
  QueueHandle_t readingQueue;
};

/*
----------------------------------------------------------------
* FreeRTOS Task for system management
* This task is responsible for notifying the laser and ultrasonic
* sensor tasks to start sampling. It also handles the light sleep
* functionality.
-----------------------------------------------------------------
*/

struct SystemTaskParams {
    TaskHandle_t laserHandle;
    TaskHandle_t usHandle;
    DistanceSensor** usSensors;
    DistanceSensor** tofSensors;
    Preferences* prefs; // Added for firstBoot
};


// Place this at global scope, before any function definitions
// RTC_DATA_ATTR uint32_t lastSampleTime[NUM_SPOTS];

void systemTask(void *pv) {
    auto *params = static_cast<SystemTaskParams*>(pv);
    TaskHandle_t laserSensingHandle = params->laserHandle;
    TaskHandle_t ultrasonicSensingHandle = params->usHandle;
    DistanceSensor** usSensors = params->usSensors;
    DistanceSensor** tofSensors = params->tofSensors;
    Preferences& prefs = *params->prefs;

    const uint32_t VACANT_SAMPLE_INTERVAL = 120000;
    const uint32_t OCCUPIED_SAMPLE_INTERVAL = 300000;

    TickType_t lastWake = xTaskGetTickCount();

    while (1) {
        uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Load occupancy states and firstBoot
        bool lastOccupied[NUM_SPOTS];
        prefs.begin("parking-sys", true);
        for (int i = 0; i < NUM_SPOTS; i++) {
            String key = String("spot") + i + "_occupied";
            lastOccupied[i] = prefs.getBool(key.c_str(), false);
        }
        bool firstBoot = prefs.getBool("firstboot", true);
        prefs.end();

        // Check if in 12 AM to 8 AM sleep window
        time_t now = time(nullptr);
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);
        if (isTimeInRange(timeinfo.tm_hour, timeinfo.tm_min)) {
            vTaskDelay(pdMS_TO_TICKS(1000)); // Let deepSleepTask handle
            continue;
        }

        // Determine which spots need sampling
        bool shouldSample[NUM_SPOTS] = {false};
        uint32_t minSleepTime = UINT32_MAX;
        for (int i = 0; i < NUM_SPOTS; i++) {
            uint32_t sampleInterval = lastOccupied[i] ? OCCUPIED_SAMPLE_INTERVAL : VACANT_SAMPLE_INTERVAL;
            if (firstBoot || (currentTime - lastSampleTime[i] >= sampleInterval)) {
                shouldSample[i] = true;
                // usSensors[i]->powerOn();
                // tofSensors[i]->powerOn();
            }
            uint32_t timeSinceLastSample = currentTime - lastSampleTime[i];
            uint32_t timeUntilNextSample = (timeSinceLastSample >= sampleInterval) ? 0 : (sampleInterval - timeSinceLastSample);
            minSleepTime = min(minSleepTime, timeUntilNextSample);
        }

        // Trigger sampling for due spots
        for (int i = 0; i < NUM_SPOTS; i++) {
            if (shouldSample[i]) {
                DEBUG("SYSTEM", "Triggering sampling for spot %d", i);
                xTaskNotifyGive(ultrasonicSensingHandle);
                xTaskNotifyGive(laserSensingHandle);
                // Wait for processing task to finish
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
                // usSensors[i]->powerOff();
                // tofSensors[i]->powerOff();
            }
        }

        // Clear firstBoot after initial sampling
        if (firstBoot && shouldSample[0] && shouldSample[1]) {
            prefs.begin("parking-sys", false);
            prefs.putBool("firstboot", false);
            prefs.end();
        }

        // Enter deep sleep if outside 12 AM to 8 AM window
        if (minSleepTime > 1000) {
            DEBUG("SYSTEM", "Entering deep sleep for %lu ms", minSleepTime);
            WiFi.disconnect(true);
            delay(100);
            Serial.flush();
            esp_sleep_enable_timer_wakeup(minSleepTime * 1000);
            esp_deep_sleep_start();
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(2000));
    }
}



/* 
* FreeRTOS Task for Laser Sensor
* When the task is notified, it will read the distance from each of the
* laser sensors.

*/

// This task waits until it's notified to start sampling
// via xTaskNotifyGive(laserSensingHandle), called by sampleSensors() task.
void laserTask(void* pv) {
  auto * params = static_cast<SensorTaskParams*>(pv);
  DistanceSensor** sensors = params->sensors;
  QueueHandle_t laserReadingQueue = params->readingQueue;

  DEBUG("SENSOR", "Laser task started on core %d", xPortGetCoreID());

  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    DEBUG("SENSOR", "Laser task triggered, sampling...");

    for (int i = 0; i < NUM_VL53L0X_SENSORS; i++) {
      DistanceSensorReading reading;
      reading.sensorId = i;
      reading.distance = sensors[i]->getMedianDistance();

      DEBUG(sensors[i]->debugTag().c_str(), "Reading %d: %.1f cm", i, reading.distance);

      if (xQueueSend(laserReadingQueue, &reading, pdMS_TO_TICKS(100)) != pdPASS) {
        DEBUG("SENSOR", "Failed to enqueue laser reading %d", i);
      }
    }
  }
}


/* 
* FreeRTOS Task for Ultrasonic Sensor
* When the task is notified, it will read the distance from each of the
* ultrasonic sensors.

*/
// This task waits until its notified to start sampling
// via xTaskNotifyGive(ultrasonicSensingHandle), called by sampleSensors() task.
void usSensorTask(void* pv) {
  auto* params = static_cast<SensorTaskParams*>(pv);
  DistanceSensor** sensors = params->sensors;
  QueueHandle_t queue = params->readingQueue;

  DEBUG("SENSOR", "Ultrasonic task started on core %d", xPortGetCoreID());

  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    DEBUG("SENSOR", "Ultrasonic task triggered, sampling...");

    for (int i = 0; i < NUM_ULTRASONIC_SENSORS; i++) {
      DistanceSensorReading reading;
      reading.sensorId = i;
      reading.distance = sensors[i]->getMedianDistance();

      DEBUG(sensors[i]->debugTag().c_str(), "Reading %d: %.1f cm", i, reading.distance);

      if (xQueueSend(queue, &reading, pdMS_TO_TICKS(100)) != pdPASS) {
        DEBUG("SENSOR", "Failed to enqueue ultrasonic reading %d", i);
      }
    }
  }
}