#include <heltec.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <AESLib.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <time.h>  

#define CONFIG_VERSION 1
#define NUM_SENSORS 1
#define THRESHOLD_DISTANCE 50
#define HYSTERESIS 5
#define DEEP_SLEEP_START_HOUR 2
#define DEEP_SLEEP_END_HOUR 5
#define DEEP_SLEEP_END_MINUTE 30
#define SENSOR_READ_INTERVAL 2000
#define SENSOR_SAMPLES 5
#define WDT_TIMEOUT_S 10

const int trigPins[NUM_SENSORS] = { 7 };
const int echoPins[NUM_SENSORS] = { 6 };

const char* ssid = "whitex";
const char* password = "whitewhite";
const char* mqttServer = "test.mosquitto.org";
const int mqttPort = 8883;
const char* mqttTopic = "parking/status";
const char* mqttClientId = "ESP32_Parking_Sensor";

uint8_t aes_key[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
uint8_t aes_iv[16];

const char* nodeId = "NODE_001";
Preferences preferences;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);


WiFiClientSecure espClient;
PubSubClient client(espClient);

bool lastState[NUM_SENSORS] = { false };
float lastDistance[NUM_SENSORS] = { 0 };
bool firstBoot = true;
SemaphoreHandle_t stateMutex;
QueueHandle_t distanceQueue;
TaskHandle_t mqttTaskHandle;
unsigned long lastSuccessfulConnection = 0;
unsigned long connectionAttempts = 0;

struct SensorData {
  int sensorId;
  float distance;
  unsigned long timestamp;
};

void connectWiFi();
void connectMQTT();
bool isTimeInRange(int currentHour, int currentMinute);
void generateRandomIV();
void syncInternalRTC();

void logError(const char* message) {
  Serial.print("ERROR: ");
  Serial.println(message);
}

float getMedianDistance(int sensorIdx, int samples = SENSOR_SAMPLES) {
  if (sensorIdx >= NUM_SENSORS) {
    logError("Invalid sensor index");
    return -1;
  }

  float readings[samples];
  int validReadings = 0;

  for (int i = 0; i < samples; i++) {
    digitalWrite(trigPins[sensorIdx], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPins[sensorIdx], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[sensorIdx], LOW);

    long duration = pulseIn(echoPins[sensorIdx], HIGH, 30000);

    if (duration == 0) {
      Serial.printf("Sensor %d reading timeout\n", sensorIdx);
      continue;
    }

    float distance = duration * 0.034 / 2;

    if (distance >= 2 && distance <= 400) {
      readings[validReadings++] = distance;
    }

    delay(50);
  }

  if (validReadings == 0) {
    return -1;
  }

  for (int i = 0; i < validReadings - 1; i++) {
    for (int j = 0; j < validReadings - i - 1; j++) {
      if (readings[j] > readings[j + 1]) {
        float temp = readings[j];
        readings[j] = readings[j + 1];
        readings[j + 1] = temp;
      }
    }
  }

  return readings[validReadings / 2];
}

void connectWiFi() {
  Serial.print("Connecting to WiFi");

  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
    lastSuccessfulConnection = millis();
    connectionAttempts = 0;
  } else {
    logError("WiFi connection failed");
    connectionAttempts++;
    WiFi.disconnect();
  }
}

void generateRandomIV() {
  for (int i = 0; i < 16; i++) {
    aes_iv[i] = random(256);
  }
  preferences.putBytes("aes_iv", aes_iv, 16);
}

void connectMQTT() {
  espClient.setInsecure();
  client.setServer(mqttServer, mqttPort);
  Serial.println("Connecting to MQTT...");

  String clientId = String(mqttClientId) + "-" + String(random(0xffff), HEX);

  int attempts = 0;
  while (!client.connected() && attempts < 5) {
    if (client.connect(clientId.c_str())) {
      Serial.println("MQTT connected");
    } else {
      Serial.print("MQTT failed: ");
      Serial.println(client.state());
      delay(1000);
      attempts++;
    }
  }

  if (!client.connected()) {
    logError("MQTT connection failed");
  }
}

void padInput(const char* input, uint8_t* padded, size_t input_len) {
  uint8_t padValue = 16 - (input_len % 16);
  memcpy(padded, input, input_len);
  for (size_t i = input_len; i < input_len + padValue; i++) {
    padded[i] = padValue;
  }
}

void syncInternalRTC() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  if (WiFi.status() == WL_CONNECTED) {
    timeClient.begin();
    if (timeClient.update()) {
      time_t now = timeClient.getEpochTime();
      struct timeval tv = { .tv_sec = now, .tv_usec = 0 };
      settimeofday(&tv, NULL);
      Serial.println("Internal RTC synchronized with NTP");
    } else {
      logError("NTP update failed");
    }
    WiFi.disconnect(true);
    delay(100);
  }
}

void sensorTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      float distance = getMedianDistance(i);

      if (distance > 0) {
        SensorData data;
        data.sensorId = i;
        data.distance = distance;
        data.timestamp = millis();

        if (xQueueSend(distanceQueue, &data, pdMS_TO_TICKS(100)) != pdPASS) {
          logError("Failed to send to queue");
        }
      }

      vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_READ_INTERVAL));
  }
}

void processingTask(void* pvParameters) {
  SensorData data;

  while (1) {
    if (xQueueReceive(distanceQueue, &data, portMAX_DELAY)) {
      bool stateChanged = false;
      bool currentState = false;

      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (data.distance < THRESHOLD_DISTANCE - HYSTERESIS) {
          currentState = true;
        } else if (data.distance > THRESHOLD_DISTANCE + HYSTERESIS) {
          currentState = false;
        } else {
          currentState = lastState[data.sensorId];
        }

        lastDistance[data.sensorId] = data.distance;

        if (currentState != lastState[data.sensorId]) {
          lastState[data.sensorId] = currentState;
          stateChanged = true;
          Serial.printf("Sensor %d state changed to %s (%.1f cm)\n",
                        data.sensorId,
                        currentState ? "OCCUPIED" : "VACANT",
                        data.distance);
        }

        xSemaphoreGive(stateMutex);
      }

      if (stateChanged || firstBoot) {
        xTaskNotifyGive(mqttTaskHandle);
        if (firstBoot) firstBoot = false;
      }
    }
  }
}

void mqttTask(void* pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
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

      size_t inputLen = strlen(jsonData.c_str());
      size_t paddedSize = ((inputLen / 16) + 1) * 16;

      uint8_t* padded = (uint8_t*)malloc(paddedSize);
      uint8_t* encrypted = (uint8_t*)malloc(paddedSize);
      uint8_t* message = (uint8_t*)malloc(16 + paddedSize);

      if (!padded || !encrypted || !message) {
        logError("Memory allocation failed");
        free(padded);
        free(encrypted);
        free(message);
        continue;
      }

      padInput(jsonData.c_str(), padded, inputLen);
      AESLib aesLib;
      aesLib.encrypt(padded, paddedSize, encrypted, aes_key, 128, aes_iv);

      memcpy(message, aes_iv, 16);
      memcpy(message + 16, encrypted, paddedSize);

      if (WiFi.status() != WL_CONNECTED) connectWiFi();
      if (WiFi.status() == WL_CONNECTED) {
        if (!client.connected()) connectMQTT();
        if (client.connected()) {
          if (client.publish(mqttTopic, message, 16 + paddedSize)) {
            Serial.println("Published: " + String(jsonData));
          } else {
            logError("MQTT publish failed");
          }
        }
      }

      free(padded);
      free(encrypted);
      free(message);
    } else {
      logError("Failed to take mutex in MQTT task");
    }
  }
}
void sleepTask(void* pvParameters) {
  preferences.begin("sleep-config", false);
  int wakeHour = preferences.getInt("wake-hour", DEEP_SLEEP_END_HOUR);
  int wakeMinute = preferences.getInt("wake-min", DEEP_SLEEP_END_MINUTE);

  while (1) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    int hour = timeinfo.tm_hour;
    int minute = timeinfo.tm_min;

    bool isDeepSleepTime = isTimeInRange(hour, minute);

    if (isDeepSleepTime) {
      struct tm wakeupTime = timeinfo;
      wakeupTime.tm_hour = wakeHour;
      wakeupTime.tm_min = wakeMinute;
      wakeupTime.tm_sec = 0;

      if (hour > wakeHour || (hour == wakeHour && minute >= wakeMinute)) {
        wakeupTime.tm_mday += 1;
      }

      time_t wakeupEpoch = mktime(&wakeupTime);
      int64_t sleepSeconds = wakeupEpoch - now;

      if (sleepSeconds > 0 && sleepSeconds < 24 * 60 * 60) {
        Serial.printf("Entering deep sleep for %lld seconds until %02d:%02d\n",
                      sleepSeconds, wakeHour, wakeMinute);

        preferences.putBool("firstboot", false);
        for (int i = 0; i < NUM_SENSORS; i++) {
          preferences.putBool(String("state" + String(i)).c_str(), lastState[i]);
        }
        preferences.end();

        esp_sleep_enable_timer_wakeup(sleepSeconds * 1000000ULL);
        esp_deep_sleep_start();
      } else {
        logError("Invalid sleep duration calculated");
      }
    } else {
      esp_sleep_enable_timer_wakeup(1 * 1000000ULL);
      esp_light_sleep_start();
    }

    vTaskDelay(pdMS_TO_TICKS(60000));
  }
}

bool isTimeInRange(int currentHour, int currentMinute) {
  if (currentHour < DEEP_SLEEP_START_HOUR || currentHour > DEEP_SLEEP_END_HOUR) {
    return false;
  }

  if (currentHour == DEEP_SLEEP_END_HOUR && currentMinute >= DEEP_SLEEP_END_MINUTE) {
    return false;
  }

  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Smart Parking System");

  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_err_t wdt_err = esp_task_wdt_init(&wdt_config);
  if (wdt_err != ESP_OK) {
    logError("Failed to initialize Watchdog Timer");
  }

  preferences.begin("parking-sys", false);
  firstBoot = preferences.getBool("firstboot", true);

  if (!firstBoot) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      lastState[i] = preferences.getBool(String("state" + String(i)).c_str(), false);
    }
  }

  if (firstBoot || !preferences.isKey("aes_iv")) {
    generateRandomIV();
  } else {
    preferences.getBytes("aes_iv", aes_iv, 16);
  }

  randomSeed(esp_random());

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
  }

  if (firstBoot) {
    syncInternalRTC();
  }

  stateMutex = xSemaphoreCreateMutex();
  distanceQueue = xQueueCreate(NUM_SENSORS * 2, sizeof(SensorData));

  if (!stateMutex || !distanceQueue) {
    logError("Failed to create FreeRTOS objects");
    while (1) { delay(1000); }
  }

  BaseType_t xReturned;

  xReturned = xTaskCreatePinnedToCore(
    sensorTask,
    "sensorTask",
    3072,
    NULL,
    1,
    NULL,
    0
  );
  if (xReturned != pdPASS) logError("Failed to create sensor task");

  xReturned = xTaskCreatePinnedToCore(
    processingTask,
    "processingTask",
    3072,
    NULL,
    2,
    NULL,
    1
  );
  if (xReturned != pdPASS) logError("Failed to create processing task");

  xReturned = xTaskCreatePinnedToCore(
    mqttTask,
    "mqttTask",
    8192,
    NULL,
    3,
    &mqttTaskHandle,
    1
  );
  if (xReturned != pdPASS) logError("Failed to create MQTT task");

  xReturned = xTaskCreatePinnedToCore(
    sleepTask,
    "sleepTask",
    3072,
    NULL,
    0,
    NULL,
    0
  );
  if (xReturned != pdPASS) logError("Failed to create sleep task");

  Serial.println("All tasks created successfully");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}