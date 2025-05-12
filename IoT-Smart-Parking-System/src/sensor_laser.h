#pragma once
#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include "config.h"

void initializeVL53LOXArray(uint8_t sensorCount, const uint8_t *sensorAddresses, const uint8_t *shutdownPins, Adafruit_VL53L0X *lox) {
  /* 
    1. Reset all sensors by setting XSHUT to LOW
    2. Activate one sensor at a time by setting XSHUT to HIGH
    3. Initialize the sensor with the corresponding I2C address
    4. Repeat for all sensors
  */

  // Reset all sensors
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
      Serial.print(F("Failed to initialize VL53L0X at index "));
      Serial.println(i);
      while (1);
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
