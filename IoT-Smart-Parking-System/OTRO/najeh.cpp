#include <heltec.h>
#include <AESLib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <Preferences.h>
#include <time.h>  

#define HYSTERESIS 5

#define SENSOR_READ_INTERVAL 2000
#define SENSOR_SAMPLES 5

const char* nodeId = "NODE_001";




bool lastState[NUM_SENSORS] = { false };
float lastDistance[NUM_SENSORS] = { 0 };
bool firstBoot = true;
SemaphoreHandle_t stateMutex;
QueueHandle_t distanceQueue;
TaskHandle_t mqttTaskHandle;



void generateRandomIV() {
  DEBUG_PRINT("CRYPTO", "Generating new random IV");
  for (int i = 0; i < 16; i++) {
    aes_iv[i] = random(256);
  }
  preferences.putBytes("aes_iv", aes_iv, 16);
  DEBUG_PRINT("CRYPTO", "New IV saved to preferences");
}

void padInput(const char* input, uint8_t* padded, size_t input_len) {
  uint8_t padValue = 16 - (input_len % 16);
  memcpy(padded, input, input_len);
  for (size_t i = input_len; i < input_len + padValue; i++) {
    padded[i] = padValue;
  }
  DEBUG_PRINT("CRYPTO", "Padded input from %d to %d bytes", input_len, input_len + padValue);
}

void sensorTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  DEBUG_SENSOR("Sensor task started on core %d", xPortGetCoreID());

  while (1) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      DEBUG_SENSOR("Reading sensor %d", i);
      float distance = getMedianDistance(i);

      if (distance > 0) {
        SensorData data;
        data.sensorId = i;
        data.distance = distance;
        data.timestamp = millis();

        if (xQueueSend(distanceQueue, &data, pdMS_TO_TICKS(100)) != pdPASS) {
          logError("Failed to send to queue");
        } else {
          DEBUG_SENSOR("Sent distance %.1f cm to queue", distance);
        }
      } else {
        DEBUG_SENSOR("Skipping invalid reading");
      }

      vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Print free heap memory occasionally for debugging
    static uint32_t lastHeapCheck = 0;
    if (millis() - lastHeapCheck > 10000) {  // Every 10 seconds
      DEBUG_MEMORY("Free heap: %u bytes", ESP.getFreeHeap());
      lastHeapCheck = millis();
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_READ_INTERVAL));
  }
}

void processingTask(void* pvParameters) {
  SensorData data;
  DEBUG_PROCESS("Processing task started on core %d", xPortGetCoreID());

  while (1) {
    if (xQueueReceive(distanceQueue, &data, portMAX_DELAY)) {
      DEBUG_PROCESS("Received data from sensor %d: %.1f cm", data.sensorId, data.distance);
      bool stateChanged = false;
      bool currentState = false;

      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        DEBUG_PROCESS("Previous state for sensor %d: %s, distance: %.1f cm", 
                      data.sensorId, 
                      lastState[data.sensorId] ? "OCCUPIED" : "VACANT",
                      lastDistance[data.sensorId]);
        
        // Determine new state with hysteresis
        if (data.distance < THRESHOLD_DISTANCE - HYSTERESIS) {
          currentState = true;
          DEBUG_PROCESS("Distance %.1f < threshold %d - hysteresis %d → OCCUPIED", 
                       data.distance, THRESHOLD_DISTANCE, HYSTERESIS);
        } else if (data.distance > THRESHOLD_DISTANCE + HYSTERESIS) {
          currentState = false;
          DEBUG_PROCESS("Distance %.1f > threshold %d + hysteresis %d → VACANT", 
                       data.distance, THRESHOLD_DISTANCE, HYSTERESIS);
        } else {
          currentState = lastState[data.sensorId];
          DEBUG_PROCESS("Distance %.1f in hysteresis range, maintaining state: %s", 
                       data.distance, currentState ? "OCCUPIED" : "VACANT");
        }

        lastDistance[data.sensorId] = data.distance;

        if (currentState != lastState[data.sensorId]) {
          lastState[data.sensorId] = currentState;
          stateChanged = true;
          DEBUG_PROCESS("Sensor %d state changed to %s (%.1f cm)",
                        data.sensorId,
                        currentState ? "OCCUPIED" : "VACANT",
                        data.distance);
        } else {
          DEBUG_PROCESS("No state change for sensor %d", data.sensorId);
        }

        xSemaphoreGive(stateMutex);
      } else {
        DEBUG_ERROR("Failed to take mutex in processing task");
      }

      if (stateChanged || firstBoot) {
        DEBUG_PROCESS("Notifying MQTT task due to %s", 
                     firstBoot ? "first boot" : "state change");
        xTaskNotifyGive(mqttTaskHandle);
        if (firstBoot) {
          DEBUG_PROCESS("First boot flag cleared");
          firstBoot = false;
        }
      }
    }
  }
}

void mqttTask(void* pvParameters) {
  DEBUG_MQTT("MQTT task started on core %d", xPortGetCoreID());
  
  while (1) {
    DEBUG_MQTT("Waiting for notification...");
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    DEBUG_MQTT("Notification received, preparing data transmission");

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      // Build JSON data
      String jsonData = "{\"nodeId\":\"" + String(nodeId) + "\",\"timestamp\":" + String(millis()) + ",\"spots\":[";
      for (int i = 0; i < NUM_SENSORS; i++) {
        jsonData += (lastState[i] ? "1" : "0");
        if (i < NUM_SENSORS - 1) jsonData += ",";
      }
      jsonData += "],\"distances\":[";
      for (int i = 0; i < NUM_SENSORS; i++) {
        jsonData += String(lastDistance[i], 1);
        if (i < NUM_SENSORS - 1) jsonData += ",";
      }
      jsonData += "]}";

      xSemaphoreGive(stateMutex);
      
      DEBUG_MQTT("JSON data prepared: %s", jsonData.c_str());

      size_t inputLen = strlen(jsonData.c_str());
      size_t paddedSize = ((inputLen / 16) + 1) * 16;

      DEBUG_MQTT("Allocating memory: input=%d bytes, padded=%d bytes", inputLen, paddedSize);
      uint8_t* padded = (uint8_t*)malloc(paddedSize);
      uint8_t* encrypted = (uint8_t*)malloc(paddedSize);
      uint8_t* message = (uint8_t*)malloc(16 + paddedSize);

      if (!padded || !encrypted || !message) {
        DEBUG_ERROR("Memory allocation failed: padded=%p, encrypted=%p, message=%p", 
                    padded, encrypted, message);
        free(padded);
        free(encrypted);
        free(message);
        continue;
      }

      DEBUG_MQTT("Encrypting data");
      padInput(jsonData.c_str(), padded, inputLen);
      AESLib aesLib;
      aesLib.encrypt(padded, paddedSize, encrypted, aes_key, 128, aes_iv);

      memcpy(message, aes_iv, 16);
      memcpy(message + 16, encrypted, paddedSize);
      DEBUG_MQTT("Message prepared with IV + encrypted data: %d bytes total", 16 + paddedSize);

      if (WiFi.status() != WL_CONNECTED) {
        DEBUG_MQTT("WiFi not connected, reconnecting");
        connectWiFi();
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        if (!client.connected()) {
          DEBUG_MQTT("MQTT client not connected, reconnecting");
          connectMQTT();
        }
        
        if (client.connected()) {
          DEBUG_MQTT("Publishing to topic: %s (%d bytes)", mqttTopic, 16 + paddedSize);
          if (client.publish(mqttTopic, message, 16 + paddedSize)) {
            DEBUG_MQTT("Publish successful: %s", jsonData.c_str());
          } else {
            DEBUG_ERROR("MQTT publish failed, state: %d", client.state());
          }
        } else {
          DEBUG_ERROR("Failed to connect to MQTT for publishing");
        }
      } else {
        DEBUG_ERROR("Failed to connect to WiFi for MQTT publishing");
      }

      free(padded);
      free(encrypted);
      free(message);
      DEBUG_MQTT("Memory freed");
    } else {
      logError("Failed to take mutex in MQTT task");
    }
  }
}

void setup() {

  if (!firstBoot) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      lastState[i] = preferences.getBool(String("state" + String(i)).c_str(), false);
      DEBUG_PRINT("SYSTEM", "Loaded sensor %d state from NVS: %s", 
                 i, lastState[i] ? "OCCUPIED" : "VACANT");
    }
  }

  if (firstBoot || !preferences.isKey("aes_iv")) {
    DEBUG_PRINT("SYSTEM", "First boot or missing IV, generating new one");
    generateRandomIV();
  } else {
    preferences.getBytes("aes_iv", aes_iv, 16);
    DEBUG_PRINT("CRYPTO", "Loaded IV from preferences");
  }

  DEBUG_PRINT("SYSTEM", "Initializing pins for %d sensors", NUM_SENSORS);
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
    DEBUG_PRINT("SYSTEM", "Sensor %d: TRIG pin %d, ECHO pin %d", i, trigPins[i], echoPins[i]);
  }

  DEBUG_PRINT("SYSTEM", "Creating FreeRTOS objects");
  stateMutex = xSemaphoreCreateMutex();
  distanceQueue = xQueueCreate(NUM_SENSORS * 2, sizeof(SensorData));

  if (!stateMutex || !distanceQueue) {
    logError("Failed to create FreeRTOS objects");
    while (1) { delay(1000); }
  }

  BaseType_t xReturned;

  DEBUG_PRINT("SYSTEM", "Creating tasks");
  xReturned = xTaskCreatePinnedToCore(sensorTask,"sensorTask",3072,NULL,1,NULL,0);
  if (xReturned != pdPASS) logError("Failed to create sensor task");

  xReturned = xTaskCreatePinnedToCore(processingTask,"processingTask",3072,NULL,2,NULL,1);
  if (xReturned != pdPASS) logError("Failed to create processing task");

  xReturned = xTaskCreatePinnedToCore(,"mqttTask",8192,NULL,3,&mqttTaskHandle,1);
  if (xReturned != pdPASS) logError("Failed to create MQTT task");

  xReturned = xTaskCreatePinnedToCore(sleepTask,"sleepTask",3072,NULL,0,NULL,0);
  if (xReturned != pdPASS) logError("Failed to create sleep task");

  DEBUG_PRINT("SYSTEM", "Setup complete, all tasks created successfully");
}