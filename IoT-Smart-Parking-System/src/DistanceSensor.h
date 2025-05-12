#pragma once
#include <Preferences.h>
#include "debug.h"
class DistanceSensor {
protected:
  String id;
  bool lastState = false;

public:
  DistanceSensor(const String& sensorId) : id(sensorId) {}
  virtual ~DistanceSensor() {}

  virtual void updateState() = 0;
  virtual String debugTag() const = 0;

protected:
  virtual float sampleOnce() = 0;

public:
  float getMedianDistance(int samples) {
    float readings[samples];
    int validReadings = 0;

    DEBUG(debugTag().c_str(), "Reading %d samples", samples);

    for (int i = 0; i < samples; i++) {
      float distance = sampleOnce();

      if (distance < 0) {
        DEBUG(debugTag().c_str(), "Reading %d invalid", i);
        continue;
      }

      readings[validReadings++] = distance;
      DEBUG(debugTag().c_str(), "Reading %d: %.1f cm", i, distance);
      delay(50);
    }

    if (validReadings == 0) {
      DEBUG(debugTag().c_str(), "No valid readings");
      return -1;
    }

    // Sort readings to compute median
    for (int i = 0; i < validReadings - 1; i++) {
      for (int j = 0; j < validReadings - i - 1; j++) {
        if (readings[j] > readings[j + 1]) {
          float temp = readings[j];
          readings[j] = readings[j + 1];
          readings[j + 1] = temp;
        }
      }
    }

    float median = readings[validReadings / 2];
    DEBUG(debugTag().c_str(), "Median: %.1f cm (valid: %d/%d)", id.c_str(), median, validReadings, samples);
    return median;
  }

  void loadState(Preferences& prefs) {
    lastState = prefs.getBool(id.c_str(), false);
    DEBUG(debugTag().c_str(), "Loaded state: %s", lastState ? "OCCUPIED" : "VACANT");
  }

  void saveState(Preferences& prefs) const {
    prefs.putBool(id.c_str(), lastState);
    DEBUG(debugTag().c_str(), "Saved state: %s", lastState ? "OCCUPIED" : "VACANT");
  }

  bool isOccupied() const { return lastState; }
  const String& getId() const { return id; }
  void setState(bool state) { lastState = state; }

  public:
  virtual float minValidRange() const = 0;
  virtual float maxValidRange() const = 0;

  void test(int samples = 5) { 
    float distance = getMedianDistance(samples);

    String fullTag = debugTag() + "-TEST";

    if (distance < 0) { 
      DEBUG(fullTag.c_str(), "No valid readings");
    } else if (distance < minValidRange() || distance > maxValidRange()) {
      DEBUG(fullTag.c_str(), "Out of range: %.1f cm", distance);
    } else {
      DEBUG(fullTag.c_str(), "Sensor %s: Distance = %.1f cm", id.c_str(), distance, samples);
    }
  }
};
