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

    DistanceSensorReading usBuffer[NUM_SPOTS] = {};
    DistanceSensorReading tofBuffer[NUM_SPOTS] = {};
    bool usReady[NUM_SPOTS] = {false};
    bool tofReady[NUM_SPOTS] = {false};
    bool lastOccupied[NUM_SPOTS] = {false};
    bool sentFirstRun[NUM_SPOTS] = {false};
    bool processedThisCycle[NUM_SPOTS] = {false};

    prefs.begin("parking-sys", true);
    for (int i = 0; i < NUM_SPOTS; i++) {
        String key = String("spot") + i + "_occupied";
        lastOccupied[i] = prefs.getBool(key.c_str(), false);
    }
    prefs.end();

    while (true) {
        uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Reset processed flags
        for (int i = 0; i < NUM_SPOTS; i++) {
            processedThisCycle[i] = false;
        }

        // Process sensor readings
        DistanceSensorReading incoming;
        int spotsToProcess = 0;
        for (int i = 0; i < NUM_SPOTS; i++) {
            uint32_t sampleInterval = lastOccupied[i] ? OCCUPIED_SAMPLE_INTERVAL : VACANT_SAMPLE_INTERVAL;
            if (currentTime >= lastSampleTime[i] + sampleInterval) {
                spotsToProcess++;
            }
        }

        for (int i = 0; i < NUM_SPOTS; i++) {
            uint32_t sampleInterval = lastOccupied[i] ? OCCUPIED_SAMPLE_INTERVAL : VACANT_SAMPLE_INTERVAL;
            if (currentTime < lastSampleTime[i] + sampleInterval) {
                continue;
            }

            usReady[i] = false;
            tofReady[i] = false;

            // Collect ultrasonic readings
            while (xQueueReceive(usQueue, &incoming, pdMS_TO_TICKS(100))) {
                if (incoming.sensorId == i) {
                    usBuffer[i] = incoming;
                    usReady[i] = true;
                    break;
                }
            }

            if (usReady[i]) {
                float usDist = usBuffer[i].distance;
                bool usOccupied = (usDist > 0 && usDist < ULTRASONIC_THRESHOLD_DISTANCE);

                if (usOccupied) {
                    while (xQueueReceive(tofQueue, &incoming, pdMS_TO_TICKS(100))) {
                        if (incoming.sensorId == i) {
                            tofBuffer[i] = incoming;
                            tofReady[i] = true;
                            break;
                        }
                    }
                } else {
                    tofBuffer[i].distance = 0;
                    tofReady[i] = true;
                }

                if (usReady[i] && tofReady[i]) {
                    float tofDist = tofBuffer[i].distance;
                    bool tofOccupied = (tofDist > 0 && tofDist < LASER_THRESHOLD_DISTANCE);
                    bool occupied = usOccupied && (usOccupied ? tofOccupied : false);

                    DEBUG("PROCESS", "Spot %d State → Occupied: %s | US: %.1f cm (Occupied: %s) | TOF: %.1f cm (Occupied: %s)",
                          i, occupied ? "Yes" : "No", usDist, usOccupied ? "Yes" : "No",
                          tofDist, tofOccupied ? "Yes" : "N/A");

                    lastSampleTime[i] = currentTime;

                    if (firstBoot || occupied != lastOccupied[i]) {
                        SpotOccupancyData data = {
                            .spotId = i,
                            .occupied = occupied,
                            .usDistance = usDist,
                            .tofDistance = tofDist
                        };

                        if (xQueueSend(mqttDataQueue, &data, pdMS_TO_TICKS(100)) != pdPASS) {
                            DEBUG("PROCESS", "Failed to send spot %d data to MQTT queue", i);
                        } else {
                            DEBUG("PROCESS", "Sent spot %d data to MQTT queue (FirstRun: %s, StateChanged: %s)",
                                  i, firstBoot ? "Yes" : "No", occupied != lastOccupied[i] ? "Yes" : "No");
                            if (firstBoot) {
                                sentFirstRun[i] = true;
                            }
                        }

                        prefs.begin("parking-sys", false);
                        String key = String("spot") + i + "_occupied";
                        prefs.putBool(key.c_str(), occupied);
                        prefs.end();
                        lastOccupied[i] = occupied;
                    }

                    processedThisCycle[i] = true;
                    xTaskNotifyGive(systemTaskHandle);
                }
            }
        }

        // Check if all due spots are processed
        bool allProcessed = true;
        for (int i = 0; i < NUM_SPOTS; i++) {
            uint32_t sampleInterval = lastOccupied[i] ? OCCUPIED_SAMPLE_INTERVAL : VACANT_SAMPLE_INTERVAL;
            if (currentTime >= lastSampleTime[i] + sampleInterval && !processedThisCycle[i]) {
                allProcessed = false;
                break;
            }
        }

        if (allProcessed && spotsToProcess > 0) {
            if (firstBoot) {
                bool allSent = true;
                for (int i = 0; i < NUM_SPOTS; i++) {
                    if (!sentFirstRun[i]) {
                        allSent = false;
                        break;
                    }
                }
                if (allSent) {
                    prefs.begin("parking-sys", false);
                    prefs.putBool("firstBoot", false);
                    prefs.end();
                    firstBoot = false;
                    DEBUG("PROCESS", "First boot complete, cleared firstBoot flag in NVS");
                }
            }

            // Calculate minSleepTime
            uint32_t minSleepTime = UINT32_MAX;
            for (int i = 0; i < NUM_SPOTS; i++) {
                uint32_t sampleInterval = lastOccupied[i] ? OCCUPIED_SAMPLE_INTERVAL : VACANT_SAMPLE_INTERVAL;
                uint32_t timeSinceLastSample = currentTime - lastSampleTime[i];
                uint32_t timeUntilNextSample = (timeSinceLastSample >= sampleInterval) ? 0 : (sampleInterval - timeSinceLastSample);
                minSleepTime = min(minSleepTime, timeUntilNextSample);
            }

            if (minSleepTime > 1000) {
                DEBUG("PROCESS", "All sampling and MQTT sending completed, entering deep sleep for %lu ms", minSleepTime);
                WiFi.disconnect(true);
                digitalWrite(Vext, HIGH); // Disable external power
                delay(100);
                Serial.flush();
                esp_sleep_enable_timer_wakeup(minSleepTime * 1000000ULL);
                esp_deep_sleep_start();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}