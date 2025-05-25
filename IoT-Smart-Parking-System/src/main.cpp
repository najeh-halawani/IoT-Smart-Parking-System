/* Include necessary libraries */
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <RadioLib.h>
#include <LoRaWAN_ESP32.h>
#include "config.h"
#include <RadioBoards.h>

/* Include header files */
#include "debug.h"
#include "utils.h"
#include "wifi_comms.h"
#include "lora_comms.h"
#include "sensor_laser.h"
#include "sensor_ultrasonic.h"
#include "deep_sleep.h"
#include "system_task.h"
#include "processing.h"
#include "aes.h"

/* Declaration of global variables */

// Radio and LoRaWAN node
Radio radio = new RadioModule();
LoRaWANNode *node;

// Ultrasonic sensor instances
UltrasonicSensor *usSensors[NUM_ULTRASONIC_SENSORS];

// VL53L0X sensor instances
TimeOfFlightSensor *tofSensors[NUM_VL53L0X_SENSORS];

// All sensors array
DistanceSensor *allSensors[NUM_ULTRASONIC_SENSORS + NUM_VL53L0X_SENSORS];

// WiFi variables
unsigned long lastSuccessfulConnection = 0;
unsigned long connectionAttempts = 0;
WiFiClient espClient;

// MQTT variables
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
TaskHandle_t loraTaskHandle;

// Queues
QueueHandle_t laserReadingQueue;
QueueHandle_t ultrasonicReadingQueue;
QueueHandle_t mqttDataQueue;
QueueHandle_t loraDataQueue;

void setup()
{
    // Initialize NVS
    preferences.begin("parking-sys", false);
    firstBoot = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER ? false : true;
    if (firstBoot)
    {
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

    // Initialize ultrasonic sensors
    initializeUltrasonicArray(usSensors, NUM_ULTRASONIC_SENSORS);

    // Initialize VL53L0X sensors
    initializeVL53LOXArray(tofSensors, NUM_VL53L0X_SENSORS);

    // Populate allSensors array
    int sensorCount = buildUnifiedSensorArray(usSensors, tofSensors, allSensors);

    // Initialize WiFi connection
    // connectWiFi(lastSuccessfulConnection, connectionAttempts);
    WiFi.setSleep(true);

    // Initialize MQTT
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setKeepAlive(MAIN_TASK_RATE * 1.5);
    // connectMQTT(client, espClient, MQTT_SERVER, MQTT_PORT, MQTT_CLIENT_ID);

    // Initialize radio
    int16_t state = radio.begin();
    if (state != RADIOLIB_ERR_NONE)
    {
        DEBUG("LORAWAN", "Radio initialization failed with error %d.", state);
        // goToDeepSleep(sensorCount); // Enter deep sleep on failure
    }

    // Initialize LoRaWAN provisioning
    if (!persist.isProvisioned())
    {
        DEBUG("LORAWAN", "No provisioning data found. Writing hardcoded config to NVS.");
        if (!persist.provision(LORAWAN_BAND, LORAWAN_SUBBAND, LORAWAN_JOINEUI, LORAWAN_DEVEUI, LORAWAN_APPKEY, LORAWAN_NWKKEY))
        {
            DEBUG("LORAWAN", "Failed to save provisioning data. Entering deep sleep.");
            // goToDeepSleep(sensorCount);
        }
    }

    // Initialize LoRaWAN node
    node = persist.manage(&radio, true); // Auto-join enabled
    if (!node->isActivated())
    {
        DEBUG("LORAWAN", "Could not join network. Entering deep sleep.");
        // goToDeepSleep(sensorCount);
    }

    // Set duty cycle for TTN Fair Use Policy
    node->setDutyCycle(true, DUTY_CYCLE_MS);

    // Check if it's the first boot
    if (firstBoot)
    {
        DEBUG("SYSTEM", "First boot detected, initializing system.");
        syncInternalRTC(lastSuccessfulConnection, connectionAttempts);
        randomSeed(esp_random());
    }
    else
    {
        DEBUG("SYSTEM", "Not the first boot, loading previous state.");
        for (int i = 0; i < sensorCount; i++)
            allSensors[i]->loadState(preferences);
    }

    // Initialize FreeRTOS objects
    laserReadingQueue = xQueueCreate(NUM_VL53L0X_SENSORS, sizeof(DistanceSensorReading));
    ultrasonicReadingQueue = xQueueCreate(NUM_ULTRASONIC_SENSORS, sizeof(DistanceSensorReading));
    mqttDataQueue = xQueueCreate(NUM_SPOTS, sizeof(SpotOccupancyData));
    loraDataQueue = xQueueCreate(NUM_SPOTS, sizeof(SpotOccupancyData));

    // Initialize FreeRTOS tasks
    static SensorTaskParams usParams = {.sensors = reinterpret_cast<DistanceSensor **>(usSensors), .readingQueue = ultrasonicReadingQueue};
    xTaskCreatePinnedToCore(usSensorTask, "usSensorTask", 4096, &usParams, 1, &ultrasonicSensingHandle, 1);

    static SensorTaskParams tofParams = {.sensors = reinterpret_cast<DistanceSensor **>(tofSensors), .readingQueue = laserReadingQueue};
    xTaskCreatePinnedToCore(laserTask, "laserTask", 4096, &tofParams, 1, &laserSensingHandle, 1);

    static SystemTaskParams sysParams = {
        .laserHandle = laserSensingHandle,
        .usHandle = ultrasonicSensingHandle,
        .usSensors = reinterpret_cast<DistanceSensor **>(usSensors),
        .tofSensors = reinterpret_cast<DistanceSensor **>(tofSensors),
        .prefs = &preferences};
    xTaskCreatePinnedToCore(systemTask, "systemTask", 4096, &sysParams, 1, &systemTaskHandle, 0);

    static ProcessingTaskParams processingParams = {
        .usQueue = ultrasonicReadingQueue,
        .tofQueue = laserReadingQueue,
        .mqttDataQueue = mqttDataQueue,
        .loraDataQueue = loraDataQueue,
        .systemTaskHandle = systemTaskHandle,
        .prefs = &preferences};
    xTaskCreatePinnedToCore(processingTask, "processingTask", 4096, &processingParams, 1, &processingHandle, 0);

    static SleepTaskParams sleepParams = {
        .prefs = &preferences,
        .sensors = allSensors,
        .sensorCount = sensorCount,
        .loraNode = node};
    xTaskCreatePinnedToCore(deepSleepTask, "sleepTask", 4096, &sleepParams, 1, NULL, 0);

    // static LoRaTaskParams loraParams = {.loraNode = node, .loraDataQueue = loraDataQueue};
    // xTaskCreatePinnedToCore(loraTask, "LoRaTask", 8192, &loraParams, 1, &loraTaskHandle, 1);

    xTaskCreatePinnedToCore(wifiMQTTTask, "WiFiMQTT Task", 8192, &mqttDataQueue, 1, &wifiMQTTHandle, 1);

    // Initialize Watchdog timer
    esp_task_wdt_init(WDT_TIMEOUT_S, true);
}

void loop()
{
}
