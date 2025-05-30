#pragma once
#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include "config.h"
#include "DistanceSensor.h"

class TimeOfFlightSensor : public DistanceSensor {
  Adafruit_VL53L0X sensor;
  VL53L0X_RangingMeasurementData_t measure;
  uint8_t shutdownPin;
  uint8_t address;

public:
  TimeOfFlightSensor(const String& id, uint8_t shutdownPin, uint8_t address)
    : DistanceSensor(id, LASER_SAMPLES, LASER_THRESHOLD_DISTANCE) {
    this->shutdownPin = shutdownPin;
    this->address = address;
  }

  float minValidRange() const override { return MIN_LASER_DISTANCE; }
  float maxValidRange() const override { return MAX_LASER_DISTANCE; }

  bool restart() {
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

  void powerOn() override {
    restart();
    delay(10);
    DEBUG("SYSTEM", "Sensor %s powered on", id.c_str());
  }

  void powerOff() override {
    pinMode(shutdownPin, OUTPUT);
    digitalWrite(shutdownPin, LOW);
    delay(10);
    DEBUG("SYSTEM", "Sensor %s powered off", id.c_str());
  }

};

void initializeVL53LOXArray(TimeOfFlightSensor** sensors, int count) {
  DEBUG("SYSTEM", "Initializing VL53L0X sensors");
  for (int i = 0; i < count; i++) {
    const uint8_t address = loxAddresses[i];
    const uint8_t shutdownPin = loxShutdownPins[i];
    sensors[i] = new TimeOfFlightSensor("TOF-"+String(i), shutdownPin, address);
  }
  DEBUG("SYSTEM", "VL53L0X sensors initialized");
}

void testVL53LOXSensors(TimeOfFlightSensor** sensors, int count) {
  for (int i = 0; i < count; i++) {
    sensors[i]->test();
  }
}

