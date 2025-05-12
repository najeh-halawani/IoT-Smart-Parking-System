/*
    * @file main.cpp
    * @brief Main file for the ESP32 project.
    * @version 0.1
    * @date 2025-05-12
    * 
    * @details
    
    
    * @author1 Jose Edgar Hernandez Cancino Estrada
    * @author2 Marcelo Enrique Jimenez Da Fonseca
    * @author3 Najeh Halawani
*/

/* Include necessary libraries */
#include <Arduino.h>
#include <esp_task_wdt.h>

/* Include header files */
#include "debug.h"
#include "config.h"
#include "utils.h"
#include "wifi_comms.h"
#include "sensor_laser.h"
#include "sensor_ultrasonic.h"
#include "sleep.h"
#include "tasks.h"
#include "aes.h"

/* Declaration of global variables */

// ------ VL53L0X sensor instances -------
Adafruit_VL53L0X lox[NUM_VL53L0X_SENSORS];
VL53L0X_RangingMeasurementData_t measures[NUM_VL53L0X_SENSORS];

// ------ WiFi variables -----------------
unsigned long lastSuccessfulConnection = 0;
unsigned long connectionAttempts = 0;
WiFiClientSecure espClient;

// ------ MQTT variables -----------------
PubSubClient client(espClient);


void setup() {
    // Initialize serial communication
    Serial.begin(115200);               delay(300);
    displaySystemInfo();           

    // Initialize I2C communication
    Wire.begin(I2C_SDA, I2C_SCL);       delay(100);
    
    // Initialize VL53L0X sensors
    initializeVL53LOXArray(NUM_VL53L0X_SENSORS, loxAddresses, shutdownPins, lox);

    // Initialize Watchdog timer
    esp_task_wdt_init(WDT_TIMEOUT_S, true);

    randomSeed(esp_random());
}

void loop() {
  testLaserSensors();
}