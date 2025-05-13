#pragma once
#include "sensor_ultrasonic.h"
#include "sensor_laser.h"
#include "DistanceSensor.h"
/* General functions definitions */

/**
 * @brief Populates an array of DistanceSensor pointers with the provided ultrasonic and ToF sensors.
 * 
 * @param usSensors Array of UltrasonicSensor pointers.
 * @param tofSensors Array of TimeOfFlightSensor pointers.
 * @param allSensors Array of DistanceSensor pointers to be populated.
 * @return The number of sensors added to the allSensors array.
 */

int buildUnifiedSensorArray(UltrasonicSensor** usSensors, TimeOfFlightSensor** tofSensors, DistanceSensor** allSensors) {
    int idx = 0;

    // Add ultrasonic sensors
    for (int i = 0; i < NUM_ULTRASONIC_SENSORS; i++) {
        allSensors[idx++] = usSensors[i];
    }

    // Add ToF sensors
    for (int i = 0; i < NUM_VL53L0X_SENSORS; i++) {
        allSensors[idx++] = tofSensors[i];
    }

    return idx;
}

/**
 * @brief Displays system information such as version, build date, chip ID, CPU frequency, and free heap memory.
 *        Only outputs information if DEBUG_MODE is enabled.
 */
void displaySystemInfo() {
    #ifdef DEBUG_MODE
        Serial.println("\n\n=== Smart Parking System Debug Mode ===");
        Serial.printf("Version: %d, Build: %s %s\n", CONFIG_VERSION, __DATE__, __TIME__);
        Serial.printf("ESP32 Chip ID: %06X, CPU Freq: %d MHz\n", 
            ESP.getEfuseMac() & 0xFFFFFF, ESP.getCpuFreqMHz());
        Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
    #endif
}

/**
 * @brief Logs an error message to the serial monitor.
 * 
 * @param message The error message to log.
 */
void logError(const char* message) {
    Serial.printf("[ERROR] %s\n", message);
}