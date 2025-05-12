/*
    * @file main.cpp
    * @brief Main file for the ESP32 project.
    * @version 0.1
    * @date 2025-05-12
    * 
    * @details
    
    
    * @author1 Jose Edgar Hernandez Cancino Estrada
    * @author2 Marcelo Enrique Jimenez Da Fonseca
    * @author3 Najeh Halawani
*/

// Include necessary libraries
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Include header files
#include "config.h"
#include "utils.h"
#include "wifi_comms.h"
#include "sensor_laser.h"
#include "sensor_ultrasonic.h"
#include "tasks.h"
#include "aes.h"

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
}

void loop() {
}