/* Include necessary libraries */
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "config.h"
#include "debug.h"
#include "utils.h"
#include "wifi_comms.h"
#include "sensor_laser.h"
#include "sensor_ultrasonic.h"
#include "deep_sleep.h"
#include "system_task.h"
#include "processing.h"
#include "aes.h"

// Define lastSampleTime
RTC_DATA_ATTR uint32_t lastSampleTime[NUM_SPOTS] = {0};

// Ultrasonic sensor instances
UltrasonicSensor *usSensors[NUM_ULTRASONIC_SENSORS];
TimeOfFlightSensor *tofSensors[NUM_VL53L0X_SENSORS];
DistanceSensor *allSensors[NUM_ULTRASONIC_SENSORS + NUM_VL53L0X_SENSORS];

// WiFi and MQTT variables
unsigned long lastSuccessfulConnection = 0;
unsigned long connectionAttempts = 0;
WiFiClient espClient;
PubSubClient client(espClient);

// NVS variables
bool firstBoot = true;
Preferences preferences;

// FreeRTOS variables
TaskHandle_t systemTaskHandle;
TaskHandle_t processingHandle;
TaskHandle_t laserSensingHandle;
TaskHandle_t ultrasonicSensingHandle;
TaskHandle_t wifiMQTTHandle;

// Queues
QueueHandle_t laserReadingQueue;
QueueHandle_t ultrasonicReadingQueue;
QueueHandle_t mqttDataQueue;
QueueHandle_t loraDataQueue;
QueueHandle_t usOccupancyQueue;

void setup() {
    // Initialize NVS
    preferences.begin("parking-sys", false);
    firstBoot = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER ? false : true;
    if (firstBoot) {
        preferences.putBool("firstboot", true);
    }
    preferences.end();

    // Initialize serial communication
    Serial.begin(115200);
    delay(300);
    displaySystemInfo();

    // Initialize I2C communication
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(100);

    // Initialize sensors
    initializeUltrasonicArray(usSensors, NUM_ULTRASONIC_SENSORS);
    initializeVL53LOXArray(tofSensors, NUM_VL53L0X_SENSORS);
    int sensorCount = buildUnifiedSensorArray(usSensors, tofSensors, allSensors);

    // Initialize WiFi
    
    connectWiFi(lastSuccessfulConnection, connectionAttempts);
    WiFi.setSleep(true);

    // Initialize MQTT
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setKeepAlive(MAIN_TASK_RATE * 1.5);

    // Initialize FreeRTOS objects
    laserReadingQueue = xQueueCreate(NUM_VL53L0X_SENSORS, sizeof(DistanceSensorReading));
    ultrasonicReadingQueue = xQueueCreate(NUM_ULTRASONIC_SENSORS, sizeof(DistanceSensorReading));
    mqttDataQueue = xQueueCreate(NUM_SPOTS, sizeof(SpotOccupancyData));
    loraDataQueue = xQueueCreate(NUM_SPOTS, sizeof(SpotOccupancyData));
    usOccupancyQueue = xQueueCreate(NUM_ULTRASONIC_SENSORS, sizeof(DistanceSensorReading));

    // Initialize FreeRTOS tasks
    static SensorTaskParams usParams = {
        .sensors = reinterpret_cast<DistanceSensor**>(usSensors),
        .readingQueue = ultrasonicReadingQueue,
        .usOccupancyQueue = usOccupancyQueue
    };
    xTaskCreatePinnedToCore(usSensorTask, "usSensorTask", 4096, &usParams, 1, &ultrasonicSensingHandle, 1);

    static SensorTaskParams tofParams = {
        .sensors = reinterpret_cast<DistanceSensor**>(tofSensors),
        .readingQueue = laserReadingQueue,
        .usOccupancyQueue = nullptr
    };
    xTaskCreatePinnedToCore(laserTask, "laserTask", 4096, &tofParams, 1, &laserSensingHandle, 1);

    static SystemTaskParams sysParams = {
        .laserHandle = laserSensingHandle,
        .usHandle = ultrasonicSensingHandle,
        .usSensors = reinterpret_cast<DistanceSensor**>(usSensors),
        .tofSensors = reinterpret_cast<DistanceSensor**>(tofSensors),
        .prefs = &preferences,
        .usOccupancyQueue = usOccupancyQueue
    };
    xTaskCreatePinnedToCore(systemTask, "systemTask", 4096, &sysParams, 1, &systemTaskHandle, 0);

    static ProcessingTaskParams processingParams = {
        .usQueue = ultrasonicReadingQueue,
        .tofQueue = laserReadingQueue,
        .mqttDataQueue = mqttDataQueue,
        .loraDataQueue = loraDataQueue,
        .systemTaskHandle = systemTaskHandle,
        .prefs = &preferences,
        .firstBoot = firstBoot
    };
    xTaskCreatePinnedToCore(processingTask, "processingTask", 4096, &processingParams, 1, &processingHandle, 0);

    static SleepTaskParams sleepParams = {
        .prefs = &preferences,
        .sensors = allSensors,
        .sensorCount = sensorCount
    };
    xTaskCreatePinnedToCore(deepSleepTask, "sleepTask", 4096, &sleepParams, 1, NULL, 0);

    xTaskCreatePinnedToCore(wifiMQTTTask, "WiFiMQTT Task", 8192, &mqttDataQueue, 1, &wifiMQTTHandle, 1);
}

void loop() {
}