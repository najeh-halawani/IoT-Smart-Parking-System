#pragma once
#include <ArduinoJson.h>
#include "config.h"
#include "debug.h"
#include "spot_occupancy_data.h"

struct ProcessingTaskParams {
    QueueHandle_t usQueue;
    QueueHandle_t tofQueue;
    QueueHandle_t mqttDataQueue;
    TaskHandle_t systemTaskHandle;
};

void processingTask(void* pv) {
    auto* params = static_cast<ProcessingTaskParams*>(pv);
    QueueHandle_t ultrasonicReadingQueue = params->usQueue;
    QueueHandle_t laserReadingQueue = params->tofQueue;
    QueueHandle_t mqttDataQueue = params->mqttDataQueue;
    TaskHandle_t systemTaskHandle = params->systemTaskHandle;
    DistanceSensorReading usBuffer[NUM_SPOTS] = {};
    DistanceSensorReading tofBuffer[NUM_SPOTS] = {};
    bool usReady[NUM_SPOTS] = {false};
    bool tofReady[NUM_SPOTS] = {false};
    bool processed[NUM_SPOTS] = {false};
    static bool lastOccupied[NUM_SPOTS] = {false}; // Track last state
    int processedCount = 0;

    DistanceSensorReading incoming;

    while (true) {
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
                float usDist = usBuffer[i].distance;
                float tofDist = tofBuffer[i].distance;
                bool occupied = (usDist < ULTRASONIC_THRESHOLD_DISTANCE || tofDist < LASER_THRESHOLD_DISTANCE);

                DEBUG("PROCESS", "Spot %d → US: %.1f | TOF: %.1f → %s",
                      i, usDist, tofDist, occupied ? "OCCUPIED" : "VACANT");

                // Only send data if state changed
                if (occupied != lastOccupied[i]) {
                    occupancyData[i].spotId = i;
                    occupancyData[i].occupied = occupied;
                    occupancyData[i].usDistance = usDist;
                    occupancyData[i].tofDistance = tofDist;

                    if (xQueueSend(mqttDataQueue, &occupancyData[i], pdMS_TO_TICKS(100)) != pdPASS) {
                        DEBUG("PROCESS", "Failed to send spot %d data to MQTT queue", i);
                    } else {
                        DEBUG("PROCESS", "Sent spot %d data to MQTT queue (state changed)", i);
                    }
                    lastOccupied[i] = occupied;
                }

                processed[i] = true;
                processedCount++;
            }
        }

        // When all spots have been processed, notify system task
        if (processedCount == NUM_SPOTS) {
            DEBUG("PROCESS", "All spots processed, notifying system task");
            processedCount = 0;
            memset(usReady, 0, sizeof(usReady));
            memset(tofReady, 0, sizeof(tofReady));
            memset(processed, 0, sizeof(processed));

            xTaskNotifyGive(systemTaskHandle);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}