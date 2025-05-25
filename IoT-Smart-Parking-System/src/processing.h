#pragma once
#include <ArduinoJson.h>
#include <esp_sleep.h>
#include <Preferences.h>
#include "config.h"
#include "debug.h"
#include "spot_occupancy_data.h"

// RTC memory for persisting sampling state
RTC_DATA_ATTR uint32_t lastSampleTime[NUM_SPOTS] = {0};

struct ProcessingTaskParams {
    QueueHandle_t usQueue;
    QueueHandle_t tofQueue;
    QueueHandle_t mqttDataQueue;
    QueueHandle_t loraDataQueue;
    TaskHandle_t systemTaskHandle;
    Preferences* prefs; // Added for sensor state
};

void processingTask(void *pv) {
    auto *params = static_cast<ProcessingTaskParams *>(pv);
    QueueHandle_t ultrasonicReadingQueue = params->usQueue;
    QueueHandle_t laserReadingQueue = params->tofQueue;
    QueueHandle_t mqttDataQueue = params->mqttDataQueue;
    QueueHandle_t loraDataQueue = params->loraDataQueue;
    TaskHandle_t systemTaskHandle = params->systemTaskHandle;
    Preferences& prefs = *params->prefs;
    DistanceSensorReading usBuffer[NUM_SPOTS] = {};
    DistanceSensorReading tofBuffer[NUM_SPOTS] = {};
    bool usReady[NUM_SPOTS] = {false};
    bool tofReady[NUM_SPOTS] = {false};
    static bool lastOccupied[NUM_SPOTS] = {false}; // Not in RTC, loaded from Preferences
    const uint32_t VACANT_SAMPLE_INTERVAL = 120000;   // 2 minutes
    const uint32_t OCCUPIED_SAMPLE_INTERVAL = 300000; // 5 minutes

    // Load lastOccupied from Preferences
    prefs.begin("parking-sys", true);
    for (int i = 0; i < NUM_SPOTS; i++) {
        String key = String("spot") + i + "_occupied";
        lastOccupied[i] = prefs.getBool(key.c_str(), false);
    }
    prefs.end();

    DistanceSensorReading incoming;

    while (true) {
        uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Receive sensor data
        if (xQueueReceive(ultrasonicReadingQueue, &incoming, pdMS_TO_TICKS(100))) {
            int id = incoming.sensorId;
            usBuffer[id] = incoming;
            usReady[id] = true;
        }

        if (xQueueReceive(laserReadingQueue, &incoming, pdMS_TO_TICKS(100))) {
            int id = incoming.sensorId;
            tofBuffer[id] = incoming;
            tofReady[id] = true;
        }

        // Process each spot independently
        bool firstBoot = prefs.getBool("firstboot", true);
        for (int i = 0; i < NUM_SPOTS; i++) {
            if (usReady[i] && tofReady[i]) {
                uint32_t sampleInterval = lastOccupied[i] ? OCCUPIED_SAMPLE_INTERVAL : VACANT_SAMPLE_INTERVAL;
                if (firstBoot || (currentTime - lastSampleTime[i] >= sampleInterval)) {
                    float usDist = usBuffer[i].distance;
                    float tofDist = tofBuffer[i].distance;
                    bool occupied = ((usDist > 0 && usDist < ULTRASONIC_THRESHOLD_DISTANCE) ||
                                     (tofDist > 0 && tofDist < LASER_THRESHOLD_DISTANCE));

                    DEBUG("PROCESS", "Spot %d → US: %.1f | TOF: %.1f → %s",
                          i, usDist, tofDist, occupied ? "OCCUPIED" : "VACANT");

                    // Send data on first boot or state change
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
                            DEBUG("PROCESS", "Sent spot %d data to MQTT queue", i);
                        }
                        // Uncomment for LoRa if needed
                        // if (xQueueSend(loraDataQueue, &data, pdMS_TO_TICKS(100)) != pdPASS) {
                        //     DEBUG("PROCESS", "Failed to send spot %d data to LoRa queue", i);
                        // }

                        // Save new state
                        prefs.begin("parking-sys", false);
                        String key = String("spot") + i + "_occupied";
                        prefs.putBool(key.c_str(), occupied);
                        prefs.end();
                        lastOccupied[i] = occupied;
                    }

                    lastSampleTime[i] = currentTime;
                    usReady[i] = false;
                    tofReady[i] = false;

                    // Notify system task immediately
                    xTaskNotifyGive(systemTaskHandle);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}