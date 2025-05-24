#pragma once
#include <ArduinoJson.h>
#include "config.h"
#include "debug.h"
#include "spot_occupancy_data.h"

struct ProcessingTaskParams {
    QueueHandle_t usQueue;
    QueueHandle_t tofQueue;
    QueueHandle_t mqttDataQueue;
    QueueHandle_t loraDataQueue; // Added for LoRa
    TaskHandle_t systemTaskHandle;
};

void processingTask(void* pv) {
    auto* params = static_cast<ProcessingTaskParams*>(pv);
    QueueHandle_t ultrasonicReadingQueue = params->usQueue;
    QueueHandle_t laserReadingQueue = params->tofQueue;
    QueueHandle_t mqttDataQueue = params->mqttDataQueue;
    QueueHandle_t loraDataQueue = params->loraDataQueue; // Added for LoRa
    TaskHandle_t systemTaskHandle = params->systemTaskHandle;
    DistanceSensorReading usBuffer[NUM_SPOTS] = {};
    DistanceSensorReading tofBuffer[NUM_SPOTS] = {};
    bool usReady[NUM_SPOTS] = {false};
    bool tofReady[NUM_SPOTS] = {false};
    bool processed[NUM_SPOTS] = {false};
    static bool lastOccupied[NUM_SPOTS] = {false};
    static bool firstBoot = true; // Track first boot
    static uint32_t lastSampleTime[NUM_SPOTS] = {0}; // Track last sample time per spot
    int processedCount = 0;
    const uint32_t VACANT_SAMPLE_INTERVAL = 5000; // 5 seconds
    const uint32_t OCCUPIED_SAMPLE_INTERVAL = 30000; // 30 seconds

    DistanceSensorReading incoming;

    while (true) {
        uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Wait on either queue
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

        // Check for complete data for each spot
        SpotOccupancyData occupancyData[NUM_SPOTS];

        for (int i = 0; i < NUM_SPOTS; i++) {
            if (usReady[i] && tofReady[i] && !processed[i]) {
                bool shouldSample = false;
                uint32_t sampleInterval = lastOccupied[i] ? OCCUPIED_SAMPLE_INTERVAL : VACANT_SAMPLE_INTERVAL;

                // Check if it's time to sample based on spot status
                if (firstBoot || (currentTime - lastSampleTime[i] >= sampleInterval)) {
                    shouldSample = true;
                    lastSampleTime[i] = currentTime;
                }

                if (shouldSample) {
                    float usDist = usBuffer[i].distance;
                    float tofDist = tofBuffer[i].distance;
                    bool occupied = ((usDist > 0 & usDist < ULTRASONIC_THRESHOLD_DISTANCE) || (tofDist > 0 & tofDist < LASER_THRESHOLD_DISTANCE));

                    DEBUG("PROCESS", "Spot %d → US: %.1f | TOF: %.1f → %s",
                          i, usDist, tofDist, occupied ? "OCCUPIED" : "VACANT");

                    // Send data on first boot or state change
                    if (firstBoot || occupied != lastOccupied[i]) {
                        occupancyData[i].spotId = i;
                        occupancyData[i].occupied = occupied;
                        occupancyData[i].usDistance = usDist;
                        occupancyData[i].tofDistance = tofDist;

                        // Send to MQTT queue
                        if (xQueueSend(mqttDataQueue, &occupancyData[i], pdMS_TO_TICKS(100)) != pdPASS) {
                            DEBUG("PROCESS", "Failed to send spot %d data to MQTT queue", i);
                        } else {
                            DEBUG("PROCESS", "Sent spot %d data to MQTT queue", i);
                        }

                        // // Send to LoRa queue
                        // if (xQueueSend(loraDataQueue, &occupancyData[i], pdMS_TO_TICKS(100)) != pdPASS) {
                        //     DEBUG("PROCESS", "Failed to send spot %d data to LoRa queue", i);
                        // } else {
                        //     DEBUG("PROCESS", "Sent spot %d data to LoRa queue", i);
                        // }

                        // lastOccupied[i] = occupied;
                    }

                    processed[i] = true;
                    processedCount++;
                }
            }
        }

        // When all spots have been processed, notify system task
        if (processedCount == NUM_SPOTS) {
            DEBUG("PROCESS", "All spots processed, notifying system task");
            processedCount = 0;
            memset(usReady, 0, sizeof(usReady));
            memset(tofReady, 0, sizeof(tofReady));
            memset(processed, 0, sizeof(processed));
            firstBoot = false; // Clear first boot flag after processing all spots

            xTaskNotifyGive(systemTaskHandle);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}