#pragma once

struct ProcessingTaskParams {
  QueueHandle_t usQueue;
  QueueHandle_t tofQueue;
  TaskHandle_t systemTaskHandle;
};

void processingTask(void* pv) {
    auto* params = static_cast<ProcessingTaskParams*>(pv);
    QueueHandle_t ultrasonicReadingQueue = params->usQueue;
    QueueHandle_t laserReadingQueue = params->tofQueue;
    TaskHandle_t systemTaskHandle = params->systemTaskHandle;
    DistanceSensorReading usBuffer[NUM_SPOTS] = {};
    DistanceSensorReading tofBuffer[NUM_SPOTS] = {};
    bool usReady[NUM_SPOTS] = {false};
    bool tofReady[NUM_SPOTS] = {false};
    bool processed[NUM_SPOTS] = {false};
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
        for (int i = 0; i < NUM_SPOTS; i++) {
            if (usReady[i] && tofReady[i] && !processed[i]) {
                // If we have both readings, we can process them
                float usDist = usBuffer[i].distance;
                float tofDist = tofBuffer[i].distance;
                bool occupied = (usDist < ULTRASONIC_THRESHOLD_DISTANCE || tofDist < LASER_THRESHOLD_DISTANCE);

                DEBUG("PROCESS", "Spot %d → US: %.1f | TOF: %.1f → %s", 
                    i, usDist, tofDist, occupied ? "OCCUPIED" : "VACANT");

                processed[i] = true;
                processedCount++;
            }
        }

        // When all spots have been processed, notify the system task
        if (processedCount == NUM_SPOTS) {
            DEBUG("PROCESS", "All spots processed, notifying system task");
            processedCount = 0;
            memset(usReady, 0, sizeof(usReady));
            memset(tofReady, 0, sizeof(tofReady));
            memset(processed, 0, sizeof(processed));

            // Notify system task that all processing is done
            xTaskNotifyGive(systemTaskHandle);
        }

        // Sleep for a short duration to avoid busy-waiting
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}