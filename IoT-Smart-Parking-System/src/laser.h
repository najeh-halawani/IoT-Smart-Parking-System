#include <Wire.h>
#include "Adafruit_VL53L0X.h"

#define NUM_SENSORS 2  // Change to 4 if you use 4 sensors

// I2C addresses to assign
const uint8_t loxAddresses[NUM_SENSORS] = {0x30, 0x31}; 

// Corresponding shutdown pins for each sensor
const uint8_t shutdownPins[NUM_SENSORS] = {48, 47};  // Adjust pins accordingly

// Sensor instances and measurement data
Adafruit_VL53L0X lox[NUM_SENSORS];
VL53L0X_RangingMeasurementData_t measures[NUM_SENSORS];

void setSensorIDs() {
  // Reset all sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(shutdownPins[i], OUTPUT);
    digitalWrite(shutdownPins[i], LOW);
  }
  delay(10);

  for (int i = 0; i < NUM_SENSORS; i++) {
    // Activate one sensor at a time
    digitalWrite(shutdownPins[i], HIGH);
    delay(10);

    if (!lox[i].begin(loxAddresses[i])) {
      Serial.print(F("Failed to initialize VL53L0X at index "));
      Serial.println(i);
      while (1);
    }
  }
}

void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    lox[i].rangingTest(&measures[i], false);

    Serial.print(i + 1);
    Serial.print(F(": "));
    if (measures[i].RangeStatus != 4) {
      Serial.print(measures[i].RangeMilliMeter);
    } else {
      Serial.print(F("Out of range"));
    }
    Serial.print(F("  "));
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(41, 42);  // Use appropriate SDA/SCL pins
  delay(100);

  Serial.println(F("Initializing shutdown pins..."));
  setSensorIDs();
}

void loop() {
  readSensors();
  delay(200);
}