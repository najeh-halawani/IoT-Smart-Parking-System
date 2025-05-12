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
#define MQTT_SERVER "192.168.178.50"
#define MQTT_PORT 1883
#define MQTT_TOPIC ".../..."
#define MQTT_CLIENT_ID "..."
// --------------------------------------------------------

// ------ I2C Configuration -------------------------------
#define I2C_SDA 41
#define I2C_SCL 42
// ---------------------------------------------------------

// ------ DEEP/LIGHT Sleep Configuration -------------------
#define DEEP_SLEEP_START_HOUR 2
#define DEEP_SLEEP_END_HOUR 5
#define DEEP_SLEEP_END_MINUTE 30
#define LIGHT_SLEEP_DURATION 60 // in seconds
// ---------------------------------------------------------

// ------ AES Encryption Configuration ---------------------
uint8_t aes_key[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
uint8_t aes_iv[16];
// ---------------------------------------------------------

// ------ VL53L0X Time-Of-Flight Sensor Configuration -------
#define NUM_VL53L0X_SENSORS 2
#define LASER_SAMPLES 5
const uint8_t loxAddresses[NUM_VL53L0X_SENSORS] = {0x30, 0x31};
const uint8_t shutdownPins[NUM_VL53L0X_SENSORS] = {48, 47};
// ----------------------------------------------------------

// ------ Ultrasonic Sensor Configuration -------------------
#define NUM_ULTRASONIC_SENSORS 1
#define NUM_SENSORS 1
#define ULTRASONIC_SAMPLES 5

const int trigPins[NUM_SENSORS] = { 19 };
const int echoPins[NUM_SENSORS] = { 20 };
// ----------------------------------------------------------
