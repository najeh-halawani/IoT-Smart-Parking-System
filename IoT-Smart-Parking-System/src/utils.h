#pragma once

/* General functions definitions */

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