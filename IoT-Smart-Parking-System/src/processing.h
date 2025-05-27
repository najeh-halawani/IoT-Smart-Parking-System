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

    // Load last occupancy states from NVS
    prefs.begin("parking-sys", true);
    for (int i = 0; i < NUM_SPOTS; i++) {
        String key = String("spot") + i + "_occupied";
        lastOccupied[i] = prefs.getBool(key.c_str(), false);
    }
    prefs.end();

    while (true) {
        uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Process sensor readings for spots that need sampling
        DistanceSensorReading incoming;
        for (int i = 0; i < NUM_SPOTS; i++) {
            // Check if sampling is due
            uint32_t sampleInterval = lastOccupied[i] ? OCCUPIED_SAMPLE_INTERVAL : VACANT_SAMPLE_INTERVAL;
            if (currentTime < lastSampleTime[i] + sampleInterval) {
                continue;
            }

            // Reset readiness flags
            usReady[i] = false;
            tofReady[i] = false;

            // Collect ultrasonic readings
            while (!usReady[i] && xQueueReceive(usQueue, &incoming, pdMS_TO_TICKS(100))) {
                if (incoming.sensorId == i) {
                    usBuffer[i] = incoming;
                    usReady[i] = true;
                }
            }

            // Process ultrasonic reading
            if (usReady[i]) {
                float usDist = usBuffer[i].distance;
                bool usOccupied = (usDist > 0 && usDist < ULTRASONIC_THRESHOLD_DISTANCE);

                // Only sample laser if ultrasonic indicates occupancy
                if (usOccupied) {
                    // Collect laser readings
                    while (!tofReady[i] && xQueueReceive(tofQueue, &incoming, pdMS_TO_TICKS(100))) {
                        if (incoming.sensorId == i) {
                            tofBuffer[i] = incoming;
                            tofReady[i] = true;
                        }
                    }
                } else {
                    tofBuffer[i].distance = -1;
                    tofReady[i] = true;
                }

                // Process if both readings are available (or laser skipped)
                if (usReady[i] && tofReady[i]) {
                    float tofDist = tofBuffer[i].distance;
                    bool tofOccupied = (tofDist > 0 && tofDist < LASER_THRESHOLD_DISTANCE);

                    // Spot is occupied only if both sensors detect occupancy
                    bool occupied = usOccupied && (usOccupied ? tofOccupied : false);

                    // Debug print after processing each spot
                    DEBUG("PROCESS", "Spot %d State → Occupied: %s | US: %.1f cm (Occupied: %s) | TOF: %.1f cm (Occupied: %s)",
                          i, occupied ? "Yes" : "No", usDist, usOccupied ? "Yes" : "No",
                          tofDist, tofOccupied ? "Yes" : "N/A");

                    // Update last sample time
                    lastSampleTime[i] = currentTime;

                    // Send data to MQTT if state changed or first run
                    if (occupied != lastOccupied[i] || (firstBoot && !sentFirstRun[i])) {
                        SpotOccupancyData data = {
                            .spotId = i,
                            .occupied = occupied,
                            .usDistance = usDist,
                            .tofDistance = tofDist
                        };

                        // Send to MQTT queue
                        if (xQueueSend(mqttDataQueue, &data, pdMS_TO_TICKS(100)) != pdPASS) {
                            DEBUG("PROCESS", "Failed to send spot %d data to MQTT queue", i);
                        } else {
                            DEBUG("PROCESS", "Sent spot %d data to MQTT queue (FirstRun: %s, StateChanged: %s)",
                                  i, firstBoot ? "Yes" : "No", occupied != lastOccupied[i] ? "Yes" : "No");
                            if (firstBoot) {
                                sentFirstRun[i] = true;
                            }
                        }

                        // Save new state to NVS
                        prefs.begin("parking-sys", false);
                        String key = String("spot") + i + "_occupied";
                        prefs.putBool(key.c_str(), occupied);
                        prefs.end();
                        lastOccupied[i] = occupied;
                    }

                    // Notify system task that processing is complete for this spot
                    xTaskNotifyGive(systemTaskHandle);
                }
            }
        }

        // Clear firstBoot in NVS if all spots have sent data on first run
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

        // Small delay to prevent tight loop
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}