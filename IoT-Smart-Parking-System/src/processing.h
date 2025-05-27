#pragma once
#include <ArduinoJson.h>
#include <esp_sleep.h>
#include <Preferences.h>
#include "config.h"
#include "debug.h"
#include "spot_occupancy_data.h"

struct ProcessingTaskParams {
    QueueHandle_t usQueue;
    QueueHandle_t tofQueue;
    QueueHandle_t mqttDataQueue;
    QueueHandle_t loraDataQueue;
    TaskHandle_t systemTaskHandle;
    Preferences* prefs;
    bool firstBoot;
};

void processingTask(void *pv) {
    auto *params = static_cast<ProcessingTaskParams*>(pv);
    QueueHandle_t usQueue = params->usQueue;
    QueueHandle_t tofQueue = params->tofQueue;
    QueueHandle_t mqttDataQueue = params->mqttDataQueue;
    TaskHandle_t systemTaskHandle = params->systemTaskHandle;
    Preferences& prefs = *params->prefs;
    bool firstBoot = params->firstBoot;

    DistanceSensorReading usBuffer[NUM_SPOTS];
    DistanceSensorReading tofBuffer[NUM_SPOTS];
    bool lastOccupied[NUM_SPOTS] = {false};
    bool sentFirstRun[NUM_SPOTS] = {false};

    // Initialize last occupied states from NVS
    prefs.begin("parking-sys", true);
    for (int i = 0; i < NUM_SPOTS; i++) {
        String key = String("spot") + i + "_occupied";
        lastOccupied[i] = prefs.getBool(key.c_str(), false);
        DEBUG("PROCESS", "Loaded spot %d last state: %s", i, lastOccupied[i] ? "occupied" : "vacant");
    }
    prefs.end();

    DEBUG("PROCESS", "Processing task started on core %d (firstBoot: %s)", 
          xPortGetCoreID(), firstBoot ? "true" : "false");

    while (true) {
        uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Check which spots need processing
        bool needsProcessing[NUM_SPOTS] = {false};
        int spotsToProcess = 0;
        
        prefs.begin("parking-sys", true);
        bool currentFirstBoot = prefs.getBool("firstBoot", false);
        prefs.end();
        
        for (int i = 0; i < NUM_SPOTS; i++) {
            uint32_t sampleInterval = lastOccupied[i] ? OCCUPIED_SAMPLE_INTERVAL : VACANT_SAMPLE_INTERVAL;
            uint32_t timeSinceLastSample = currentTime - lastSampleTime[i];
            
            if (currentFirstBoot || timeSinceLastSample >= sampleInterval) {
                needsProcessing[i] = true;
                spotsToProcess++;
            }
        }

        if (spotsToProcess == 0) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        DEBUG("PROCESS", "Processing %d spots that need updates", spotsToProcess);

        // Process each spot that needs updating
        for (int spotId = 0; spotId < NUM_SPOTS; spotId++) {
            if (!needsProcessing[spotId]) {
                continue;
            }

            bool usReady = false;
            bool tofReady = false;
            DistanceSensorReading usReading, tofReading;

            // Step 1: Get ultrasonic reading for this spot
            TickType_t startTime = xTaskGetTickCount();
            while (!usReady && (xTaskGetTickCount() - startTime) < pdMS_TO_TICKS(2000)) {
                if (xQueueReceive(usQueue, &usReading, pdMS_TO_TICKS(100))) {
                    if (usReading.sensorId == spotId) {
                        usBuffer[spotId] = usReading;
                        usReady = true;
                        DEBUG("PROCESS", "Got US reading for spot %d: %.1f cm", spotId, usReading.distance);
                        break;
                    } else {
                        // Put back reading for different spot
                        xQueueSendToFront(usQueue, &usReading, 0);
                    }
                }
            }

            if (!usReady) {
                DEBUG("PROCESS", "ERROR: Failed to get US reading for spot %d", spotId);
                continue;
            }

            // Step 2: Determine if we need laser reading
            float usDist = usBuffer[spotId].distance;
            bool usOccupied = (usDist > 0 && usDist < ULTRASONIC_THRESHOLD_DISTANCE);
            
            if (usOccupied) {
                // Need laser confirmation - get TOF reading
                startTime = xTaskGetTickCount();
                while (!tofReady && (xTaskGetTickCount() - startTime) < pdMS_TO_TICKS(2000)) {
                    if (xQueueReceive(tofQueue, &tofReading, pdMS_TO_TICKS(100))) {
                        if (tofReading.sensorId == spotId) {
                            tofBuffer[spotId] = tofReading;
                            tofReady = true;
                            DEBUG("PROCESS", "Got TOF reading for spot %d: %.1f cm", spotId, tofReading.distance);
                            break;
                        } else {
                            // Put back reading for different spot
                            xQueueSendToFront(tofQueue, &tofReading, 0);
                        }
                    }
                }
                
                if (!tofReady) {
                    DEBUG("PROCESS", "ERROR: Failed to get TOF reading for spot %d", spotId);
                    // Use fallback - assume vacant if we can't confirm with laser
                    tofBuffer[spotId].distance = MAX_LASER_DISTANCE;
                    tofBuffer[spotId].sensorId = spotId;
                    tofReady = true;
                }
            } else {
                // Clearly vacant from ultrasonic - no need for laser
                tofBuffer[spotId].distance = MAX_LASER_DISTANCE;
                tofBuffer[spotId].sensorId = spotId;
                tofReady = true;
                DEBUG("PROCESS", "Spot %d clearly vacant from US, skipping TOF", spotId);
            }

            // Step 3: Make occupancy decision
            if (usReady && tofReady) {
                float tofDist = tofBuffer[spotId].distance;
                bool tofOccupied = (tofDist > 0 && tofDist < LASER_THRESHOLD_DISTANCE);
                
                // Final occupancy decision: both sensors must agree for occupied
                bool finalOccupied = usOccupied && tofOccupied;
                
                DEBUG("PROCESS", "Spot %d Decision → US: %.1f cm (%s), TOF: %.1f cm (%s), Final: %s",
                      spotId, usDist, usOccupied ? "occupied" : "vacant",
                      tofDist, tofOccupied ? "occupied" : "vacant",
                      finalOccupied ? "OCCUPIED" : "VACANT");

                // Update sample time
                lastSampleTime[spotId] = currentTime;

                // Step 4: Check if we need to send MQTT update
                bool shouldSendMQTT = false;
                if (currentFirstBoot) {
                    shouldSendMQTT = true;
                    DEBUG("PROCESS", "Sending spot %d (first boot)", spotId);
                } else if (finalOccupied != lastOccupied[spotId]) {
                    shouldSendMQTT = true;
                    DEBUG("PROCESS", "Sending spot %d (state changed: %s → %s)", 
                          spotId, lastOccupied[spotId] ? "occupied" : "vacant",
                          finalOccupied ? "occupied" : "vacant");
                }

                if (shouldSendMQTT) {
                    SpotOccupancyData data = {
                        .spotId = spotId,
                        .occupied = finalOccupied,
                        .usDistance = usDist,
                        .tofDistance = tofDist
                    };

                    if (xQueueSend(mqttDataQueue, &data, pdMS_TO_TICKS(1000)) == pdPASS) {
                        DEBUG("PROCESS", "Queued MQTT data for spot %d", spotId);
                        if (currentFirstBoot) {
                            sentFirstRun[spotId] = true;
                        }
                    } else {
                        DEBUG("PROCESS", "ERROR: Failed to queue MQTT data for spot %d", spotId);
                    }
                }

                // Step 5: Update stored state
                if (finalOccupied != lastOccupied[spotId]) {
                    prefs.begin("parking-sys", false);
                    String key = String("spot") + spotId + "_occupied";
                    prefs.putBool(key.c_str(), finalOccupied);
                    prefs.end();
                    lastOccupied[spotId] = finalOccupied;
                    DEBUG("PROCESS", "Updated stored state for spot %d: %s", spotId, finalOccupied ? "occupied" : "vacant");
                }

                // Step 6: Notify system task that this spot is processed
                if (systemTaskHandle != NULL) {
                    xTaskNotifyGive(systemTaskHandle);
                    DEBUG("PROCESS", "Notified system task: spot %d processed", spotId);
                }
            }
        }

        // Check if first boot is complete
        if (currentFirstBoot) {
            bool allSent = true;
            for (int i = 0; i < NUM_SPOTS; i++) {
                if (needsProcessing[i] && !sentFirstRun[i]) {
                    allSent = false;
                    break;
                }
            }
            
            if (allSent) {
                prefs.begin("parking-sys", false);
                prefs.putBool("firstBoot", false);
                prefs.end();
                firstBoot = false;
                DEBUG("PROCESS", "First boot sequence completed - cleared firstBoot flag");
            }
        }

        // Brief delay before next cycle
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}