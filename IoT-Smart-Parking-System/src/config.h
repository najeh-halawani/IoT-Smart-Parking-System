#pragma once

#include <stdint.h>

/* System Configuration File */

// ------ System Information ------------------------------
#define CONFIG_VERSION 1
// --------------------------------------------------------

// ------ FREE_RTOS Configuration -------------------------
#define WDT_TIMEOUT_S 10
// --------------------------------------------------------

// ------ WiFi Configuration ------------------------------
#define WIFI_SSID "Najeh's S25 Ultra"
#define WIFI_PASSWORD "white1xx"
// --------------------------------------------------------

// ------ MQTT Configuration ------------------------------
// #define MQTT_SERVER "broker.hivemq.com"
#define MQTT_SERVER "test.mosquitto.org"
#define MQTT_PORT 1883
#define MQTT_TOPIC "SmartParkingSystem"
#define MQTT_CLIENT_ID "ESP32_Parking_Sensor"
// --------------------------------------------------------

// ------ I2C Configuration -------------------------------
#define I2C_SDA 41
#define I2C_SCL 42
// ---------------------------------------------------------

// ------ DEEP/LIGHT Sleep Configuration -------------------
#define DEEP_SLEEP_START_HOUR 0
#define DEEP_SLEEP_END_HOUR 1
#define DEEP_SLEEP_END_MINUTE 19
#define NTP_OFFSET 7200 // in seconds
// ---------------------------------------------------------

// ------ General TASKS Configuration ----------------------
// #define MAIN_TASK_RATE 30 // in seconds, will be deep sleep duration
#define MAIN_TASK_RATE 30 // in seconds, will be deep sleep duration

// ------ AES Encryption Configuration ---------------------
uint8_t aes_key[32] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F
};
// ---------------------------------------------------------

// ------ General Distance Sensor Configuration -------------
#define HYSTERESIS 5
#define NUM_SPOTS 2

#define NUM_SPOTS 2 // Number of parking spots
#define OCCUPANCY_THRESHOLD 50.0 // cm, distance below which spot is considered occupied
#define SAMPLING_RATE_UNOCCUPIED 5000 // ms, sampling interval when unoccupied
#define SAMPLING_RATE_OCCUPIED 30000 // ms, sampling interval when occupied
#define ULTRASONIC_WEIGHT 0.4 // Weight for ultrasonic sensor in fusion
#define TOF_WEIGHT 0.6 // Weight for ToF sensor in fusion
#define MAX_DISTANCE 400.0 // cm, maximum valid sensor distance
#define CONSISTENCY_THRESHOLD 0.2 // 20% max difference between sensor readings
#define DATA_FRESHNESS_TIMEOUT 5000 // ms, max age of sensor data
#define SINGLE_SENSOR_THRESHOLD 60.0 
// ----------------------------------------------------------

// ------ VL53L0X Time-Of-Flight Sensor Configuration -------
#define NUM_VL53L0X_SENSORS NUM_SPOTS
#define LASER_SAMPLES 5
#define LASER_THRESHOLD_DISTANCE 50
#define MIN_LASER_DISTANCE 2
#define MAX_LASER_DISTANCE 100000000
const uint8_t loxAddresses[NUM_VL53L0X_SENSORS] = { 0x30, 0x31 };
const uint8_t loxShutdownPins[NUM_VL53L0X_SENSORS] = { 48, 47 };
// ----------------------------------------------------------

// ------ Ultrasonic Sensor Configuration -------------------
#define NUM_ULTRASONIC_SENSORS NUM_SPOTS
#define ULTRASONIC_SAMPLES 5
#define ULTRASONIC_THRESHOLD_DISTANCE 50
#define MIN_ULTRASONIC_DISTANCE 2
#define MAX_ULTRASONIC_DISTANCE 400
const int trigPins[NUM_ULTRASONIC_SENSORS] = { 19, 33 };
const int echoPins[NUM_ULTRASONIC_SENSORS] = { 20, 34 };
// ----------------------------------------------------------

// LoRaWAN configurations
#define RADIO_BOARD_WIFI_LORA32_V3 
const char* LORAWAN_BAND = "EU868"; // Adjust for your region (e.g., US915, AS923)
const uint8_t LORAWAN_SUBBAND = 0;  // Set to 0 if no sub-band
const uint8_t LORAWAN_PORT = 1;
const uint64_t LORAWAN_JOINEUI = 0x0000000000000000; 
const uint64_t LORAWAN_DEVEUI = 0x123456789ABCDEF0;  
const uint8_t LORAWAN_APPKEY[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10}; 
const uint8_t LORAWAN_NWKKEY[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10}; 
#define MINIMUM_DELAY 900 // Minimum delay in seconds (per TTN Fair Use Policy)
#define DUTY_CYCLE_MS 1250 // Duty cycle interval in ms