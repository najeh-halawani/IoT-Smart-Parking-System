#pragma once
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

void connectWiFi(unsigned long lastSuccessfulConnection, unsigned long connectionAttempts) {
    DEBUG("WIFI", "Connecting to WiFi SSID: %s", WIFI_SSID);
  
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
  
    if (WiFi.status() == WL_CONNECTED) {
        DEBUG("WIFI", "Connected with IP: %s (RSSI: %d dBm)", WiFi.localIP().toString().c_str(), WiFi.RSSI());
      lastSuccessfulConnection = millis();
      connectionAttempts = 0;
    } else {
      DEBUG("WIFI", "Connection failed after %d attempts. Status: %d", attempts, WiFi.status());
      connectionAttempts++;
      WiFi.disconnect();
    }
  }

void connectMQTT(PubSubClient &client, WiFiClientSecure &espClient, const char* mqttServer, uint16_t mqttPort, const char* mqttClientId) {
    espClient.setInsecure();  // Use insecure TLS (no certificate validation)
    client.setServer(mqttServer, mqttPort);

    DEBUG("MQTT", "Connecting to MQTT server: %s:%d", mqttServer, mqttPort);
    String clientId = String(mqttClientId) + "-" + String(random(0xffff), HEX);
    DEBUG("MQTT", "Using client ID: %s", clientId.c_str());

    int attempts = 0;
    while (!client.connected() && attempts < 5) {
        if (client.connect(clientId.c_str())) {
            DEBUG("MQTT", "Connected successfully");
        } else {
            DEBUG("MQTT", "Connection failed, state: %d", client.state());
            delay(1000);
            attempts++;
        }
    }
    if (!client.connected()) {
        logError("MQTT connection failed after multiple attempts");
    }
}