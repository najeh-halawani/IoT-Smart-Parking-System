#pragma once
#include <stdint.h>

/* System Configuration File */

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



// ----------------------------------------------------------
