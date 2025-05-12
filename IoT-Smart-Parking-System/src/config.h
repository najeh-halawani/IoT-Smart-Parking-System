#pragma once
#include <stdint.h>
#include <Arduino.h>

/* System Configuration File */

// ------ --Debug macros for more consistent output
#define DEBUG_PRINT(tag, format, ...) Serial.printf("[%s] " format "\n", tag, ##__VA_ARGS__)
#define DEBUG_ERROR(format, ...) DEBUG_PRINT("ERROR", format, ##__VA_ARGS__)
#define DEBUG_SENSOR(format, ...) DEBUG_PRINT("SENSOR", format, ##__VA_ARGS__)

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
