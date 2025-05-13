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
#include <NTPClient.h>
#include <WiFiUdp.h>

/* Include header files */
#include "debug.h"
#include "config.h"
#include "utils.h"
#include "wifi_comms.h"
#include "sensor_laser.h"
#include "sensor_ultrasonic.h"
#include "deep_sleep.h"
#include "system_task.h"
#include "processing.h"
#include "aes.h"

/* Declaration of global variables */

// ------ Ultrasonic sensor instances ----
UltrasonicSensor* usSensors[NUM_ULTRASONIC_SENSORS];

// ------ VL53L0X sensor instances -------
TimeOfFlightSensor* tofSensors[NUM_VL53L0X_SENSORS];

// ------ All sensors array --------------
DistanceSensor* allSensors[NUM_ULTRASONIC_SENSORS + NUM_VL53L0X_SENSORS];

// ------ WiFi variables -----------------
unsigned long lastSuccessfulConnection = 0;
unsigned long connectionAttempts = 0;
WiFiClientSecure espClient;

// ------ MQTT variables -----------------
PubSubClient client(espClient);

// ------ NVS variables ------------------
bool firstBoot = true;
Preferences preferences;

// ------ FreeRTOS variables -------------
// Handles
TaskHandle_t systemTaskHandle;
TaskHandle_t processingHandle;
TaskHandle_t laserSensingHandle;
TaskHandle_t ultrasonicSensingHandle;
// Queues
QueueHandle_t laserReadingQueue;
QueueHandle_t ultrasonicReadingQueue;

void setup() {

    // Initialize NVS
    preferences.begin("parking-sys", false);
    firstBoot = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER ? false : true;
    // Initialize serial communication
    Serial.begin(115200);               delay(300);
    displaySystemInfo();           
    // Initialize I2C communication
    Wire.begin(I2C_SDA, I2C_SCL);       delay(100);
    // Initialize ultrasonic sensors
    initializeUltrasonicArray(usSensors, NUM_ULTRASONIC_SENSORS);
    // Initialize VL53L0X sensors
    initializeVL53LOXArray(tofSensors, NUM_VL53L0X_SENSORS);
    // Populate allSensors array
    int sensorCount = buildUnifiedSensorArray(usSensors, tofSensors, allSensors);
    // Initialize WiFi connection
    connectWiFi(lastSuccessfulConnection, connectionAttempts);

    // Check if it's the first boot. If so, initialize the system.
    if (firstBoot) {
        DEBUG("SYSTEM", "First boot detected, initializing system.");
        // Setup internal RTC using NTP
        syncInternalRTC(lastSuccessfulConnection, connectionAttempts);
        // Initialize random seed
        randomSeed(esp_random());
    } else {
        DEBUG("SYSTEM", "Not the first boot, loading previous state.");
        // Load sensor states from NVS
        for (int i = 0; i < sensorCount; i++)
           allSensors[i]->loadState(preferences);
    }
    
    // Initialize FreeRTOS objects
    laserReadingQueue = xQueueCreate(NUM_VL53L0X_SENSORS, sizeof(DistanceSensorReading));
    ultrasonicReadingQueue = xQueueCreate(NUM_ULTRASONIC_SENSORS, sizeof(DistanceSensorReading));

    // ---------- Initialization of FreeRTOS tasks -----------

    // ----- Sensing Tasks: Ultrasonic (Core 1)
    static SensorTaskParams usParams = {.sensors = reinterpret_cast<DistanceSensor**>(usSensors), .readingQueue = ultrasonicReadingQueue};
    xTaskCreatePinnedToCore(usSensorTask, "usSensorTask", 4096, &usParams, 1, &ultrasonicSensingHandle, 1);
    
    // ----- Sensing Tasks: Laser (Core 1)
    static SensorTaskParams tofParams = {.sensors = reinterpret_cast<DistanceSensor**>(tofSensors), .readingQueue = laserReadingQueue};
    xTaskCreatePinnedToCore(laserTask, "laserTask", 4096, &tofParams, 1, &laserSensingHandle, 1);
    
    // ----- System Task (Core 0)
    static SystemTaskParams sysParams = {.laserHandle = laserSensingHandle, .usHandle = ultrasonicSensingHandle};
    xTaskCreatePinnedToCore(systemTask, "systemTask", 4096, &sysParams, 1, &systemTaskHandle, 0);

    // ----- Processing Task (Core 0)
    static ProcessingTaskParams processingParams = {.usQueue = ultrasonicReadingQueue, .tofQueue = laserReadingQueue, .systemTaskHandle = systemTaskHandle};
    xTaskCreatePinnedToCore(processingTask, "processingTask", 4096, &processingParams, 1, &processingHandle, 0);

    // ----- Deep Sleep Task (Core 0)
    static SleepTaskParams sleepParams = {.prefs = &preferences, .sensors = allSensors, .sensorCount = sensorCount};
    xTaskCreatePinnedToCore(deepSleepTask, "sleepTask", 4096, &sleepParams, 1, NULL, 0);

    // Initialize Watchdog timer
    esp_task_wdt_init(WDT_TIMEOUT_S, true);
}

void loop() {
    // delay(2500);
    // testUltrasonicSensors(usSensors, NUM_ULTRASONIC_SENSORS);
    // testVL53LOXSensors(tofSensors, NUM_VL53L0X_SENSORS);
}