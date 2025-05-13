#pragma once

/* System Configuration File */

// ------ System Information ------------------------------
#define CONFIG_VERSION 1
// --------------------------------------------------------

// ------ FREE_RTOS Configuration -------------------------
#define WDT_TIMEOUT_S 10
// --------------------------------------------------------

// ------ WiFi Configuration ------------------------------
#define WIFI_SSID "FRITZ!Box 7530 LP"
#define WIFI_PASSWORD "70403295595551907386"
// --------------------------------------------------------

// ------ MQTT Configuration ------------------------------
#define MQTT_SERVER " 192.168.178.59"
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
#define LIGHT_SLEEP_DURATION 20 // in seconds
#define NTP_OFFSET 7200 // in seconds
// ---------------------------------------------------------

// ------ Geneal TASKS Configuration ----------------------
#define LIGHT_SLEEP_TASK_RATE 10 // in seconds

// ------ AES Encryption Configuration ---------------------
uint8_t aes_key[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
uint8_t aes_iv[16];
// ---------------------------------------------------------

// ------ General Distance Sensor Configuration -------------
#define HYSTERESIS 5
// ----------------------------------------------------------

// ------ VL53L0X Time-Of-Flight Sensor Configuration -------
#define NUM_VL53L0X_SENSORS 2
#define LASER_SAMPLES 5
#define LASER_THRESHOLD_DISTANCE 50
#define MIN_LASER_DISTANCE 2
#define MAX_LASER_DISTANCE 1500
const uint8_t loxAddresses[NUM_VL53L0X_SENSORS] = {0x30, 0x31};
const uint8_t loxShutdownPins[NUM_VL53L0X_SENSORS] = {48, 47};
// ----------------------------------------------------------

// ------ Ultrasonic Sensor Configuration -------------------
#define NUM_ULTRASONIC_SENSORS 1
#define ULTRASONIC_SAMPLES 5
#define ULTRASONIC_THRESHOLD_DISTANCE 50
#define MIN_ULTRASONIC_DISTANCE 2
#define MAX_ULTRASONIC_DISTANCE 400
const int trigPins[NUM_ULTRASONIC_SENSORS] = { 19 };
const int echoPins[NUM_ULTRASONIC_SENSORS] = { 20 };
// ----------------------------------------------------------
