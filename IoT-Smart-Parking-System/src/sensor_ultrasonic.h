#pragma once
#include "config.h"

struct SensorData {
  int sensorId;
  float distance;
  unsigned long timestamp;
};


// Initialize the ultrasonic sensor pins
void setupUltrasonicSensors() {
  DEBUG(PRINT,"SYSTEM", "Initializing ultrasonic sensors");
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
    DEBUG(PRINT,"SYSTEM", "Sensor %d: TRIG pin %d, ECHO pin %d", i, trigPins[i], echoPins[i]);
  }
} 

/**
 * @brief Reads multiple distance samples from an ultrasonic sensor, filters invalid readings, 
 *        and calculates the median distance.
 * 
 * @param sensorIdx The index of the sensor to read from (0 to NUM_SENSORS - 1).
 * @param samples The number of distance samples to take (default is SENSOR_SAMPLES).
 * @return float The median distance in centimeters. Returns -1 if no valid readings are obtained 
 *               or if the sensor index is invalid.
 */
float getMedianDistance(int sensorIdx, int samples = SENSOR_SAMPLES) {
  // Check if the sensor index is valid
  if (sensorIdx >= NUM_SENSORS) {
    DEBUG(ERROR,"Invalid sensor index"); // Log an error if the index is invalid
    return -1; // Return -1 to indicate an error
  }

  // Debug: Start of sensor reading
   DEBUG(SENSOR_US,"Reading sensor %d with %d samples", sensorIdx, samples);

  // Array to store distance readings
  float readings[samples];
  int validReadings = 0; // Counter for valid readings

  // Loop to take multiple samples
  for (int i = 0; i < samples; i++) {
    // Trigger the ultrasonic sensor
    digitalWrite(trigPins[sensorIdx], LOW);
    delayMicroseconds(2); // Ensure a clean LOW pulse
    digitalWrite(trigPins[sensorIdx], HIGH);
    delayMicroseconds(10); // Send a 10-microsecond HIGH pulse
    digitalWrite(trigPins[sensorIdx], LOW);

    // Measure the duration of the echo signal
    long duration = pulseIn(echoPins[sensorIdx], HIGH, 30000); // Timeout after 30ms

    // Check if the echo signal was received
    if (duration == 0) {
       DEBUG(SENSOR_US,"Sensor %d reading %d timeout", sensorIdx, i); // Log a timeout
      continue; // Skip to the next iteration
    }

    // Calculate the distance in cm (0.0343 cm/us is the speed of sound and we divide by 2 for the round trip)	
    float distance = duration * 0.034 / 2;

    // Check if the distance is within the valid range
    if (distance >= 2 && distance <= 400) {
      readings[validReadings++] = distance; // Store the valid reading
       DEBUG(SENSOR_US,"Sensor %d reading %d: %.1f cm (duration: %ld μs)", 
                   sensorIdx, i, distance, duration);
    } else {
       DEBUG(SENSOR_US,"Sensor %d reading %d out of range: %.1f cm", sensorIdx, i, distance);
    }

    delay(50); // Short delay between samples
  }

  // Check if there were any valid readings
  if (validReadings == 0) {
     DEBUG(SENSOR_US,"Sensor %d: No valid readings", sensorIdx); // Log no valid readings
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
   DEBUG(SENSOR_US,"Sensor %d median: %.1f cm (valid readings: %d/%d)", 
               sensorIdx, median, validReadings, samples);
  return median; // Return the median distance
}

/**
 * @brief Test function for ultrasonic sensors.
 * 
 * This function tests the ultrasonic sensors by taking multiple distance measurements and 
 * printing the results to the serial monitor. It checks for valid readings and handles out-of-range 
 * values.
 */
void testUltrasonicSensors(){
    setupUltrasonicSensors(); // Initialize the ultrasonic sensors
    delay(100); // Short delay to ensure sensors are ready
    // Test the ultrasonic sensor
    for (int i = 0; i < NUM_SENSORS; i++) {
        // Get the distance measurement
        float distance = getMedianDistance(i, SENSOR_SAMPLES);
        // Print the result to the serial monitor
        if (distance == -1) {
            Serial.printf("Sensor %d: No valid readings\n", i);
        } else if (distance < 2 || distance > 400) {
            Serial.printf("Sensor %d: Out of range (%.1f cm)\n", i, distance);
        } else {
            Serial.printf("Sensor %d: Distance = %.1f cm\n", i, distance);
        }
    }

    // Wait before the next measurement
    delay(1000); // 1 second delay
}
