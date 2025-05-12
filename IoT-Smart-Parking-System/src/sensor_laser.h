#pragma once
#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include "config.h"

extern Adafruit_VL53L0X lox[NUM_VL53L0X_SENSORS];
extern VL53L0X_RangingMeasurementData_t measures[NUM_VL53L0X_SENSORS];

void initializeVL53LOXArray(uint8_t sensorCount, const uint8_t *sensorAddresses, const uint8_t *shutdownPins, Adafruit_VL53L0X *lox) {
  /* 
    1. Reset all sensors by setting XSHUT to LOW
    2. Activate one sensor at a time by setting XSHUT to HIGH
    3. Initialize the sensor with the corresponding I2C address
    4. Repeat for all sensors
  */

  // Reset all sensors
  DEBUG(VL53L0X, "Resetting all sensors");
  for (uint8_t i = 0; i < sensorCount; i++) {
    pinMode(shutdownPins[i], OUTPUT);
    digitalWrite(shutdownPins[i], LOW);
  }
  delay(10);

  // Activate one sensor at a time
  for (uint8_t i = 0; i < sensorCount; i++) {
    digitalWrite(shutdownPins[i], HIGH);
    delay(10);

    if (!lox[i].begin(sensorAddresses[i])) {
      DEBUG(VL53L0X, "Failed to initialize VL53L0X at index %d", i);
      while (1);
    } else {
      DEBUG(VL53L0X, "VL53L0X %d initialized at address 0x%02X", i, sensorAddresses[i]);
    }
  }
  
}

// void readSensors() {
//   for (int i = 0; i < NUM_SENSORS; i++) {
//     lox[i].rangingTest(&measures[i], false);

//     Serial.print(i + 1);
//     Serial.print(F(": "));
//     if (measures[i].RangeStatus != 4) {
//       Serial.print(measures[i].RangeMilliMeter);
//     } else {
//       Serial.print(F("Out of range"));
//     }
//     Serial.print(F("  "));
//   }
//   Serial.println();
// }


/**
 * @brief Reads multiple distance samples from a VL53L0X laser sensor, filters invalid readings, 
 *        and calculates the median distance.
 * 
 * @param sensorIdx The index of the sensor to read from (0 to NUM_VL53L0X_SENSORS - 1).
 * @param samples The number of distance samples to take (default is LASER_SAMPLES).
 * @return float The median distance in centimeters. Returns -1 if no valid readings are obtained 
 *               or if the sensor index is invalid.
 */
float getMedianLaserDistance(int sensorIdx, int samples = LASER_SAMPLES) {
  // Check if the sensor index is valid
  if (sensorIdx >= NUM_VL53L0X_SENSORS) {
    DEBUG(ERROR, "Invalid sensor index"); // Log an error if the index is invalid
    return -1; // Return -1 to indicate an error
  }

  // Debug: Start of sensor reading
  DEBUG(SENSOR_LS, "Reading sensor %d with %d samples", sensorIdx, samples);

  // Array to store distance readings
  float readings[samples];
  int validReadings = 0; // Counter for valid readings

  // Loop to take multiple samples
  for (int i = 0; i < samples; i++) {
    // Trigger the laser sensor and perform a ranging test
    lox[sensorIdx].rangingTest(&measures[sensorIdx], false); // Pass 'true' for debug data

    // Check if the measurement is valid
    if (measures[sensorIdx].RangeStatus != 4) { // 4 means "Out of range"
      float distance = measures[sensorIdx].RangeMilliMeter / 10.0; // Convert mm to cm
      readings[validReadings++] = distance; // Store the valid reading
      DEBUG(SENSOR_LS, "Sensor %d reading %d: %.1f cm", sensorIdx, i, distance);
    } else {
      DEBUG(SENSOR_LS, "Sensor %d reading %d out of range", sensorIdx, i);
    }

    delay(50); // Short delay between samples
  }

  // Check if there were any valid readings
  if (validReadings == 0) {
    DEBUG(SENSOR_LS, "Sensor %d: No valid readings", sensorIdx); // Log no valid readings
    return -1; // Return -1 to indicate an error
  }

  // Sort the valid readings to calculate the median
  for (int i = 0; i < validReadings - 1; i++) {
    for (int j = 0; j < validReadings - i - 1; j++) {
      if (readings[j] > readings[j + 1]) {
        float temp = readings[j];
        readings[j] = readings[j + 1];
        readings[j + 1] = temp;
      }
    }
  }

  // Calculate the median of the valid readings
  float median = readings[validReadings / 2];
  DEBUG(SENSOR_LS, "Sensor %d median: %.1f cm (valid readings: %d/%d)", 
        sensorIdx, median, validReadings, samples);
  return median; // Return the median distance
}



void testLaserSensors() {
  initializeVL53LOXArray(NUM_VL53L0X_SENSORS, loxAddresses, shutdownPins, lox);
      delay(100); // Short delay to ensure sensors are ready
    // Test the ultrasonic sensor
    for (int i = 0; i < NUM_VL53L0X_SENSORS; i++) {
        // Get the distance measurement
        float distance = getMedianLaserDistance(i, LASER_SAMPLES);
        // Print the result to the serial monitor
        if (distance == -1) {
            Serial.printf("Laser Sensor %d: No valid readings\n", i);
        } else if (distance > 400) {
            Serial.printf("Laser Sensor %d: Out of range (%.1f cm)\n", i, distance);
        } else {
            Serial.printf("Laser Sensor %d: Distance = %.1f cm\n", i, distance);
        }
    }

    // Wait before the next measurement
    delay(1000); // 1 second delay
}
