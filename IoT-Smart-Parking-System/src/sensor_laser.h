#pragma once
#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include "config.h"
#include "DistanceSensor.h"

class TimeOfFlightSensor : public DistanceSensor {
  Adafruit_VL53L0X sensor;
  VL53L0X_RangingMeasurementData_t measure;

public:
  TimeOfFlightSensor(const String& id)
  : DistanceSensor(id, LASER_SAMPLES, LASER_THRESHOLD_DISTANCE) {}

  float minValidRange() const override { return MIN_LASER_DISTANCE; }
  float maxValidRange() const override { return MAX_LASER_DISTANCE; }

  bool restart(uint8_t shutdownPin, uint8_t address) {
    // Reset the sensor:
    // 1. Set the XSHUT pin to LOW
    // 2. Set the XSHUT pin to HIGH
    // 3. Initialize the sensor with the corresponding I2C address
    
    pinMode(shutdownPin, OUTPUT);
    // 1.
    digitalWrite(shutdownPin, LOW);
    delay(10);
    // 2.
    digitalWrite(shutdownPin, HIGH);
    delay(10);
    // 3.
    if (!sensor.begin(address)) {
      DEBUG("SYSTEM", "Failed to initialize VL53L0X at address 0x%02X", address);
      // while (1);
      return false;
    }
    DEBUG("SYSTEM", "VL53L0X initialized at address 0x%02X", address);
    return true;
  }

  float sampleOnce() override {
    sensor.rangingTest(&measure, false);
    if (measure.RangeStatus == 4) return -1;
    float distance = measure.RangeMilliMeter / 10.0;
    return (distance >= minValidRange() && distance <= maxValidRange()) ? distance : -1;
  }

  String debugTag() const override {
    return "SENSOR-" + id;
  }

  void PowerOn(uint8_t sensorIndex) {
  if (sensorIndex >= NUM_VL53L0X_SENSORS) {
    DEBUG("SYSTEM", "Invalid sensor index: %d", sensorIndex);
    return;
  }
  uint8_t shutdownPin = loxShutdownPins[sensorIndex];
  restart(shutdownPin, loxAddresses[sensorIndex]);
  DEBUG("SYSTEM", "Sensor %d powered on", sensorIndex);
}

void PowerOff(uint8_t sensorIndex) {
  if (sensorIndex >= NUM_VL53L0X_SENSORS) {
    DEBUG("SYSTEM", "Invalid sensor index: %d", sensorIndex);
    return;
  }
  uint8_t shutdownPin = loxShutdownPins[sensorIndex];
  pinMode(shutdownPin, OUTPUT);
  digitalWrite(shutdownPin, LOW);
  delay(10);
  DEBUG("SYSTEM", "Sensor %d powered off", sensorIndex);
}

};

void initializeVL53LOXArray(TimeOfFlightSensor** sensors, int count) {
  DEBUG("SYSTEM", "Initializing VL53L0X sensors");
  for (int i = 0; i < count; i++) {
    const uint8_t address = loxAddresses[i];
    const uint8_t shutdownPin = loxShutdownPins[i];
    sensors[i] = new TimeOfFlightSensor("TOF-"+String(i));
    if (!sensors[i]->restart(shutdownPin, address)){
        DEBUG("SYSTEM", "Sensor %d failed to initialize", i);
        continue;
    }
  }
  DEBUG("SYSTEM", "VL53L0X sensors initialized");
}

void testVL53LOXSensors(TimeOfFlightSensor** sensors, int count) {
  for (int i = 0; i < count; i++) {
    sensors[i]->test();
  }
}

