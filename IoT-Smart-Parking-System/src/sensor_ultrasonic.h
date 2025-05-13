#pragma once
#include "config.h"
#include "DistanceSensor.h"

class UltrasonicSensor : public DistanceSensor {
  int trigPin, echoPin;

public:
  UltrasonicSensor(const String& id, int trig, int echo)
    : DistanceSensor(id, ULTRASONIC_SAMPLES, ULTRASONIC_THRESHOLD_DISTANCE), trigPin(trig), echoPin(echo) {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);
    DEBUG("SYSTEM", "Ultrasonic sensor %s initialized: TRIG pin %d, ECHO pin %d", id.c_str(), trigPin, echoPin);
  }

  float minValidRange() const override { return MIN_ULTRASONIC_DISTANCE; }
  float maxValidRange() const override { return MAX_ULTRASONIC_DISTANCE; }

  float sampleOnce() override {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration == 0) return -1;

    float distance = duration * 0.034 / 2.0;
  
    return (distance >= minValidRange() && distance <= maxValidRange()) ? distance : -1;
  }

  String debugTag() const override {
    return "SENSOR_US_" + id;
  }

};

void initializeUltrasonicArray(UltrasonicSensor** sensors, int count) {
  DEBUG("SYSTEM", "Initializing ultrasonic sensors");
  for (int i = 0; i < count; i++) {
    sensors[i] = new UltrasonicSensor("US-"+String(i), trigPins[i], echoPins[i]);
  }
}

void testUltrasonicSensors(UltrasonicSensor** sensors, int count) {
  for (int i = 0; i < count; i++) {
    sensors[i]->test();
  }
}