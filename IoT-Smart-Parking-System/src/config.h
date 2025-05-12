#pragma once

/* System Configuration File */

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

// ------ I2C Configuration --------------------------------
#define I2C_SDA 41
#define I2C_SCL 42
// ----------------------------------------------------------

// ------ VL53L0X Time-Of-Flight Sensor Configuration -------
#define NUM_VL53L0X_SENSORS 2
const uint8_t loxAddresses[NUM_VL53L0X_SENSORS] = {0x30, 0x31};
const uint8_t shutdownPins[NUM_VL53L0X_SENSORS] = {48, 47};
// ----------------------------------------------------------

// ------ Ultrasonic Sensor Configuration -------------------
#define NUM_ULTRASONIC_SENSORS 2
#define NUM_SENSORS 1
#define SENSOR_SAMPLES 5

const int trigPins[NUM_SENSORS] = { 19 };
const int echoPins[NUM_SENSORS] = { 20 };

// ----------------------------------------------------------
