#pragma once
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

void connectWiFi(unsigned long lastSuccessfulConnection, unsigned long connectionAttempts) {
    DEBUG(WIFI, "Connecting to WiFi SSID: %s", WIFI_SSID);
  
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
  
    if (WiFi.status() == WL_CONNECTED) {
        DEBUG(WIFI, "Connected with IP: %s (RSSI: %d dBm)", WiFi.localIP().toString().c_str(), WiFi.RSSI());
      lastSuccessfulConnection = millis();
      connectionAttempts = 0;
    } else {
      DEBUG(WIFI, "Connection failed after %d attempts. Status: %d", attempts, WiFi.status());
      connectionAttempts++;
      WiFi.disconnect();
    }
  }