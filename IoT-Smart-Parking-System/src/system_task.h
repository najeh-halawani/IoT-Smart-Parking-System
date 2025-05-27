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

struct DistanceSensorReading {
  int sensorId;
  float distance;
};

struct SensorTaskParams {
  DistanceSensor** sensors;
  QueueHandle_t readingQueue;
  QueueHandle_t usOccupancyQueue;
};

struct SystemTaskParams {
    TaskHandle_t laserHandle;
    TaskHandle_t usHandle;
    DistanceSensor** usSensors;
    DistanceSensor** tofSensors;
    Preferences* prefs;
    QueueHandle_t usOccupancyQueue;
};

void systemTask(void *pv) {
    auto *params = static_cast<SystemTaskParams*>(pv);
    TaskHandle_t laserSensingHandle = params->laserHandle;
    TaskHandle_t usHandle = params->usHandle;
    DistanceSensor** usSensors = params->usSensors;
    DistanceSensor** tofSensors = params->tofSensors;
    Preferences& prefs = *params->prefs;
    QueueHandle_t usOccupancyQueue = params->usOccupancyQueue;

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
        bool firstBoot = prefs.getBool("firstBoot", true);
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
        int spotsToSample = 0;
        for (int i = 0; i < NUM_SPOTS; i++) {
            uint32_t sampleInterval = lastOccupied[i] ? OCCUPIED_SAMPLE_INTERVAL : VACANT_SAMPLE_INTERVAL;
            uint32_t timeSinceLastSample = currentTime - lastSampleTime[i];
            if (firstBoot || timeSinceLastSample >= sampleInterval) {
                shouldSample[i] = true;
                spotsToSample++;
            }
        }

        // Process sampling if any spot is due
        if (spotsToSample > 0) {
            // Trigger ultrasonic sampling
            DEBUG("SYSTEM", "Triggering ultrasonic sampling for spots: %d, %d", shouldSample[0], shouldSample[1]);
            xTaskNotifyGive(usHandle);
            // Wait for ultrasonic task to complete (timeout after 1000ms)
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0) {
                DEBUG("SYSTEM", "Ultrasonic sampling timeout");
            } else {
                DEBUG("SYSTEM", "Ultrasonic sampling completed");
            }

            // Check which spots need laser sampling
            bool shouldSampleLaser[NUM_SPOTS] = {false};
            int laserSpots = 0;
            DistanceSensorReading usReading;
            while (xQueueReceive(usOccupancyQueue, &usReading, pdMS_TO_TICKS(100))) {
                int spotId = usReading.sensorId;
                if (usReading.distance > 0 && usReading.distance < ULTRASONIC_THRESHOLD_DISTANCE) {
                    shouldSampleLaser[spotId] = true;
                    laserSpots++;
                }
            }

            // Trigger laser sampling if needed
            if (laserSpots > 0) {
                DEBUG("SYSTEM", "Triggering laser sampling for spots: %d, %d", shouldSampleLaser[0], shouldSampleLaser[1]);
                xTaskNotifyGive(laserSensingHandle);
                // Wait for laser to complete (timeout after 1000ms)
                if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0) {
                    DEBUG("SYSTEM", "Laser sampling timeout");
                } else {
                    DEBUG("SYSTEM", "Laser sampling completed");
                }
            }

            // Wait for processing task to complete for all sampled spots
            for (int i = 0; i < spotsToSample; i++) {
                if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10000)) == 0) {
                    DEBUG("SYSTEM", "Processing task timeout for spot %d", i);
                } else {
                    DEBUG("SYSTEM", "Processing completed for spot %d", i);
                }
            }
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(2000));
    }
}

void laserTask(void* pv) {
    auto *params = static_cast<SensorTaskParams*>(pv);
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

            if (reading.distance < 0) {
                DEBUG("SENSOR", "Invalid laser reading for sensor %d: %.1f cm", i, reading.distance);
                continue;
            }

            DEBUG(sensors[i]->debugTag().c_str(), "Reading %d: %.1f cm", i, reading.distance);

            if (xQueueSend(laserReadingQueue, &reading, pdMS_TO_TICKS(100)) != pdPASS) {
                DEBUG("SENSOR", "Failed to enqueue laser reading %d", i);
            }
        }

        // Notify system task that laser sampling is complete
        xTaskNotifyGive(xTaskGetHandle("systemTask"));
    }
}

void usSensorTask(void* pv) {
    auto* params = static_cast<SensorTaskParams*>(pv);
    DistanceSensor** sensors = params->sensors;
    QueueHandle_t queue = params->readingQueue;
    QueueHandle_t usOccupancyQueue = params->usOccupancyQueue;

    DEBUG("SENSOR", "Ultrasonic task started on core %d", xPortGetCoreID());

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        DEBUG("SENSOR", "Ultrasonic task triggered, sampling...");

        for (int i = 0; i < NUM_ULTRASONIC_SENSORS; i++) {
            DistanceSensorReading reading;
            reading.sensorId = i;
            reading.distance = sensors[i]->getMedianDistance();

            if (reading.distance < 0) {
                DEBUG("SENSOR", "Invalid ultrasonic reading for sensor %d: %.1f cm", i, reading.distance);
                continue;
            }

            DEBUG(sensors[i]->debugTag().c_str(), "Reading %d: %.1f cm", i, reading.distance);

            // Send to ultrasonic reading queue for processing task
            if (xQueueSend(queue, &reading, pdMS_TO_TICKS(100)) != pdPASS) {
                DEBUG("SENSOR", "Failed to enqueue ultrasonic reading %d", i);
            }

            // Send to usOccupancyQueue for system task
            if (xQueueSend(usOccupancyQueue, &reading, pdMS_TO_TICKS(100)) != pdPASS) {
                DEBUG("SENSOR", "Failed to enqueue ultrasonic reading %d to occupancy queue", i);
            }
        }

        // Notify system task that ultrasonic sampling is complete
        xTaskNotifyGive(xTaskGetHandle("systemTask"));
    }
}