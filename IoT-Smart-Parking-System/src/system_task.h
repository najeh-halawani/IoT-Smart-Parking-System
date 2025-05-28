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
  uint32_t timestamp;
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

        // Check if in sleep window
        time_t now = time(nullptr);
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);
        if (isTimeInRange(timeinfo.tm_hour, timeinfo.tm_min)) {
            vTaskDelay(pdMS_TO_TICKS(60000)); // Let deepSleepTask handle
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
                DEBUG("SYSTEM", "Spot %d needs sampling (interval: %lu, since last: %lu)", 
                      i, sampleInterval, timeSinceLastSample);
            }
        }

        // Process sampling if any spot is due
        if (spotsToSample > 0) {
            DEBUG("SYSTEM", "Starting sensor sampling sequence for %d spots", spotsToSample);
            
            // Clear any stale data from occupancy queue
            // DistanceSensorReading staleReading;
            // while (xQueueReceive(usOccupancyQueue, &staleReading, 0)) {
            //     // Just clear the queue
            // }

            // Step: Process ultrasonic results and determine laser needs
            bool shouldSampleLaser[NUM_SPOTS] = {false};
            int laserSpots = 0;
            DistanceSensorReading usReading;
            
            // Process all ultrasonic readings
            int expectedReadings = NUM_ULTRASONIC_SENSORS;
            int receivedReadings = 0;
            TickType_t startTime = xTaskGetTickCount();
            
            // Trigger ultrasonic sampling --------------------
            DEBUG("SYSTEM", "Step 1: Triggering ultrasonic sampling");
            xTaskNotifyGive(usHandle);
            // Wait for ultrasonic task to complete with proper timeout
            uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(3000));
            if (notificationValue == 0) {
                DEBUG("SYSTEM", "ERROR: Ultrasonic sampling timeout");
                continue; // Skip this cycle
            }
            DEBUG("SYSTEM", "Ultrasonic sampling completed successfully");
            // --------------------------------------------------

            while (receivedReadings < expectedReadings && 
                   (xTaskGetTickCount() - startTime) < pdMS_TO_TICKS(1000)) {
                if (xQueueReceive(usOccupancyQueue, &usReading, pdMS_TO_TICKS(100))) {
                    receivedReadings++;
                    int spotId = usReading.sensorId;
                    
                    if (spotId >= 0 && spotId < NUM_SPOTS && shouldSample[spotId]) {
                        DEBUG("SYSTEM", "Processing US reading for spot %d: %.1f cm", spotId, usReading.distance);
                        
                        // Check if ultrasonic indicates potential occupancy
                        if (usReading.distance > 0 && usReading.distance < ULTRASONIC_THRESHOLD_DISTANCE) {
                            shouldSampleLaser[spotId] = true;
                            laserSpots++;
                            DEBUG("SYSTEM", "Spot %d needs laser validation (US: %.1f cm)", spotId, usReading.distance);
                        } else {
                            DEBUG("SYSTEM", "Spot %d is clearly vacant (US: %.1f cm)", spotId, usReading.distance);
                        }
                    }
                }
            }

            if (receivedReadings < expectedReadings) {
                DEBUG("SYSTEM", "WARNING: Only received %d/%d ultrasonic readings", receivedReadings, expectedReadings);
            }

            // Step 3: Trigger laser sampling if needed
            if (laserSpots > 0) {
                DEBUG("SYSTEM", "Step 3: Triggering laser sampling for %d spots", laserSpots);
                xTaskNotifyGive(laserSensingHandle);
                
                // Wait for laser to complete
                notificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(3000));
                if (notificationValue == 0) {
                    DEBUG("SYSTEM", "ERROR: Laser sampling timeout");
                    continue; // Skip this cycle
                }
                DEBUG("SYSTEM", "Laser sampling completed successfully");
            } else {
                DEBUG("SYSTEM", "No laser sampling needed - all spots clearly vacant");
            }

            // Step 4: Wait for processing task to complete
            DEBUG("SYSTEM", "Step 4: Waiting for processing task to complete");
            
            // Wait for processing notifications (one per spot processed)
            int processedSpots = 0;
            for (int i = 0; i < spotsToSample; i++) {
                notificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000));
                if (notificationValue > 0) {
                    processedSpots++;
                    DEBUG("SYSTEM", "Processing completed for spot %d/%d", processedSpots, spotsToSample);
                } else {
                    DEBUG("SYSTEM", "ERROR: Processing timeout for spot %d", i);
                }
            }

            if (processedSpots == spotsToSample) {
                DEBUG("SYSTEM", "All spots processed successfully (%d/%d)", processedSpots, spotsToSample);
            } else {
                DEBUG("SYSTEM", "WARNING: Only %d/%d spots processed", processedSpots, spotsToSample);
            }
        }

        // Calculate next wake time
        uint32_t nextWakeDelay = 30000; // Default 30 seconds
        if (spotsToSample == 0) {
            // Find the minimum time until next sample is needed
            uint32_t minTimeToNext = UINT32_MAX;
            prefs.begin("parking-sys", true);
            for (int i = 0; i < NUM_SPOTS; i++) {
                String key = String("spot") + i + "_occupied";
                bool occupied = prefs.getBool(key.c_str(), false);
                uint32_t sampleInterval = occupied ? OCCUPIED_SAMPLE_INTERVAL : VACANT_SAMPLE_INTERVAL;
                uint32_t timeSinceLastSample = currentTime - lastSampleTime[i];
                
                if (timeSinceLastSample < sampleInterval) {
                    uint32_t timeToNext = sampleInterval - timeSinceLastSample;
                    if (timeToNext < minTimeToNext) {
                        minTimeToNext = timeToNext;
                    }
                }
            }
            prefs.end();
            
            if (minTimeToNext != UINT32_MAX && minTimeToNext > 1000) {
                nextWakeDelay = minTimeToNext;
            }
        }

        DEBUG("SYSTEM", "System task cycle complete. Next wake in %lu ms", nextWakeDelay);
        vTaskDelay(pdMS_TO_TICKS(nextWakeDelay));
    }
}

void laserTask(void* pv) {
    auto *params = static_cast<SensorTaskParams*>(pv);
    DistanceSensor** sensors = params->sensors;
    QueueHandle_t laserReadingQueue = params->readingQueue;

    DEBUG("SENSOR", "Laser task started on core %d", xPortGetCoreID());

    while (true) {
        // Wait for notification from system task
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        DEBUG("SENSOR", "Laser task triggered, starting sampling...");

        uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        int samplesCollected = 0;

        for (int i = 0; i < NUM_VL53L0X_SENSORS; i++) {
            DistanceSensorReading reading;
            reading.sensorId = i;
            reading.timestamp = currentTime;
            reading.distance = sensors[i]->getMedianDistance();

            DEBUG(sensors[i]->debugTag().c_str(), "Laser reading %d: %.1f cm", i, reading.distance);

            if (reading.distance < 0) {
                DEBUG("SENSOR", "Invalid laser reading for sensor %d, using fallback", i);
                reading.distance = MAX_LASER_DISTANCE; // Assume no obstacle if sensor fails
            }

            if (xQueueSend(laserReadingQueue, &reading, pdMS_TO_TICKS(500)) == pdPASS) {
                samplesCollected++;
            } else {
                DEBUG("SENSOR", "Failed to enqueue laser reading %d", i);
            }
        }

        DEBUG("SENSOR", "Laser sampling complete: %d/%d samples collected", samplesCollected, NUM_VL53L0X_SENSORS);
        
        // Notify system task that laser sampling is complete
        TaskHandle_t systemHandle = xTaskGetHandle("systemTask");
        if (systemHandle != NULL) {
            xTaskNotifyGive(systemHandle);
        }
    }
}

void usSensorTask(void* pv) {
    auto* params = static_cast<SensorTaskParams*>(pv);
    DistanceSensor** sensors = params->sensors;
    QueueHandle_t queue = params->readingQueue;
    QueueHandle_t usOccupancyQueue = params->usOccupancyQueue;

    DEBUG("SENSOR", "Ultrasonic task started on core %d", xPortGetCoreID());

    while (true) {
        // Wait for notification from system task
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        DEBUG("SENSOR", "Ultrasonic task triggered, starting sampling...");

        uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        int samplesCollected = 0;

        for (int i = 0; i < NUM_ULTRASONIC_SENSORS; i++) {
            DistanceSensorReading reading;
            reading.sensorId = i;
            reading.timestamp = currentTime;
            reading.distance = sensors[i]->getMedianDistance();

            DEBUG(sensors[i]->debugTag().c_str(), "US reading %d: %.1f cm", i, reading.distance);

            if (reading.distance < 0) {
                DEBUG("SENSOR", "Invalid ultrasonic reading for sensor %d, using max distance", i);
                reading.distance = MAX_ULTRASONIC_DISTANCE; // Assume no obstacle if sensor fails
            }

            // Send to main processing queue
            if (xQueueSend(queue, &reading, pdMS_TO_TICKS(500)) == pdPASS) {
                samplesCollected++;
            } else {
                DEBUG("SENSOR", "Failed to enqueue ultrasonic reading %d to main queue", i);
            }

            // Send to occupancy queue for system task coordination
            if (xQueueSend(usOccupancyQueue, &reading, pdMS_TO_TICKS(500)) != pdPASS) {
                DEBUG("SENSOR", "Failed to enqueue ultrasonic reading %d to occupancy queue", i);
            }
        }

        DEBUG("SENSOR", "Ultrasonic sampling complete: %d/%d samples collected", samplesCollected, NUM_ULTRASONIC_SENSORS);
        
        // Notify system task that ultrasonic sampling is complete
        TaskHandle_t systemHandle = xTaskGetHandle("systemTask");
        if (systemHandle != NULL) {
            xTaskNotifyGive(systemHandle);
        }
    }
}