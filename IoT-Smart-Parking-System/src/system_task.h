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
};

void systemTask(void *pv) {
  auto * params = static_cast<SystemTaskParams*>(pv);
  TaskHandle_t laserSensingHandle = params->laserHandle;
  TaskHandle_t ultrasonicSensingHandle = params->usHandle;

  TickType_t lastWake = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(2000));

    DEBUG("SYSTEM", "Triggering sensor sampling...");

    // Notify both tasks to start sampling
    xTaskNotifyGive(ultrasonicSensingHandle);
    xTaskNotifyGive(laserSensingHandle);
    
    // Wait for processing task to finish
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    DEBUG("SYSTEM", "Processing Done. Sleeping for %u seconds", LIGHT_SLEEP_DURATION);
    Serial.flush();
    lightSleepForSeconds(LIGHT_SLEEP_DURATION);
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