#pragma once
#include <Preferences.h>
#include "debug.h"
class DistanceSensor {
protected:
  String id;
  int nSamples;
  int thresholdDistance;
  bool isVacant = true;
  
public:
  DistanceSensor(const String& sensorId, int samples, int threshold)
    : id(sensorId), nSamples(samples), thresholdDistance(threshold) {}

  virtual ~DistanceSensor() {}
  virtual String debugTag() const = 0;  

  bool updateState(float distance) {
    bool vacant;
    if (distance < thresholdDistance - HYSTERESIS) {
      vacant = false;
    } else if (distance > thresholdDistance + HYSTERESIS) {
      vacant = true;
    } else {
      vacant = isVacant;
    }
    setState(vacant);
    return vacant;
  }

protected:
  virtual float sampleOnce() = 0;

public:
  float getMedianDistance() {
    float readings[nSamples];
    int validReadings = 0;

    DEBUG(debugTag().c_str(), "Reading %d samples", nSamples);

    for (int i = 0; i < nSamples; i++) {
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
    DEBUG(debugTag().c_str(), "Median: %.1f cm (valid: %d/%d)", id.c_str(), median, validReadings, nSamples);
    return median;
  }

  void loadState(Preferences& prefs) {
    isVacant = prefs.getBool(id.c_str(), false);
    setState(isVacant);
    DEBUG(debugTag().c_str(), "Loaded state: %s", isVacant ? "VACANT" : "OCCUPIED");
  }

  void saveState(Preferences& prefs) const {
    prefs.putBool(id.c_str(), isVacant);
    DEBUG(debugTag().c_str(), "Saved state: %s", isVacant ? "VACANT" : "OCCUPIED");
  }

  bool isOccupied() const { return isVacant; }
  const String& getId() const { return id; }
  void setState(bool state) { isVacant = state; }

public:
  virtual float minValidRange() const = 0;
  virtual float maxValidRange() const = 0;

  void test() { 
    float distance = getMedianDistance();

    String fullTag = debugTag() + "-TEST";

    if (distance < 0) { 
      DEBUG(fullTag.c_str(), "No valid readings");
    } else if (distance < minValidRange() || distance > maxValidRange()) {
      DEBUG(fullTag.c_str(), "Out of range: %.1f cm", distance);
    } else {
      DEBUG(fullTag.c_str(), "Sensor %s: Distance = %.1f cm", id.c_str(), distance, nSamples);
    }
  }
};

struct SensorTaskParams {
  int sensorCount;
  int samplesPerReading;
  DistanceSensor** sensors;
};

void sensorTask(void* pvParameters)  {
  // Sensor task implementation
  // This function will be called in a separate FreeRTOS task
  // to handle sensor readings.
  auto* params = static_cast<SensorTaskParams*>(pvParameters);
  DistanceSensor** sensors = params->sensors;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  DEBUG("SENSOR", "Sensor task started on core %d", xPortGetCoreID());

  while(true) {
    for (int i = 0; i < params->sensorCount; i++) {
      DEBUG("SENSOR", "Reading sensor %d", sensors[i]->getId().c_str());
      float distance = sensors[i]->getMedianDistance();

      if (distance < 0) {
        DEBUG("SENSOR", "Invalid reading from sensor %d", sensors[i]->getId().c_str());
      } else {
        DEBUG("SENSOR", "Sensor %d: Distance = %.1f cm", sensors[i]->getId().c_str(), distance);

      }
    }
  }
}