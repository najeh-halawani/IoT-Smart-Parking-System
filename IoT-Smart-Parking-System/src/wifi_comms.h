#pragma once

/* To connect to WiFi and MQTT broker, follow these steps:  
 * 1. Set up your WiFi credentials (SSID and password) in the code.
 *  -   Both your PC and ESP32 should be connected to the same WiFi network.
 * 2. Set up your MQTT broker address, config and port. 
 *  - C:\Program Files\mosquitto\mosquitto.conf should have the following uncomented lines:
 *      - listener 1883
 *      - allow_anonymous true
 *  - mqtt_server = Use Win + r, open command prompt using cmd, and type ipconfig to find your local IP address 192.168.XXX.XX
 *  - mqtt_port = 1883; This is default port for MQTT. You could change it in the file mosquitto.conf
 *  - mqtt_topic = This is the topic for publishing data. Can be any name you want.
 * 3. To start the MQTT broker open a terminal and paste: mosquitto -c "C:\Program Files\mosquitto\mosquitto.conf" -v .
 *  - netstat -ano | findstr :1883  to show the port is open 
 * 

 * 4. Open another terminal and paste: mosquitto_sub -h <mqtt_server> -t "<mqtt_topic>" -v to subscribe to the topic.
 * 
 * 5. Upload the code to your ESP32 board.
 * 6. Open the Serial Monitor in Arduino IDE to see the output.
 * 7. You can use a the python script to plot the data received from the ESP32.
 */
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <config.h>

extern WiFiClientSecure espClient;
extern PubSubClient client; 
extern unsigned long lastSuccessfulConnection;
extern unsigned long connectionAttempts;

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

void testWiFiAndMQTT() {
    // Check if WiFi is connected
    if (WiFi.status() != WL_CONNECTED) {
        DEBUG("WIFI", "WiFi not connected. Attempting to connect...");
        connectWiFi(lastSuccessfulConnection, connectionAttempts);
        if (WiFi.status() == WL_CONNECTED)
        {
          DEBUG("WIFI", "WiFi connection successful. IP: %s", WiFi.localIP().toString().c_str());
        } else {
        DEBUG("WIFI", "WiFi connection failed.");
        return; // Exit the test if WiFi connection fails
    }

    } else {
        DEBUG("WIFI", "WiFi already connected. IP: %s", WiFi.localIP().toString().c_str());
    }

    // Test MQTT connection
    if (!client.connected()) {
        DEBUG("MQTT", "MQTT not connected. Attempting to connect...");
    connectMQTT(client, espClient, MQTT_SERVER, MQTT_PORT, MQTT_CLIENT_ID);

    if (client.connected()) {
        DEBUG("MQTT", "MQTT connection successful.");

        // Publish a test message to the MQTT topic
        const char* testMessage = "TEST: ESP32 connected to MQTT broker successfully!";
        if (client.publish(MQTT_TOPIC, testMessage)) {
            DEBUG("MQTT", "Test message published to topic '%s': %s", MQTT_TOPIC, testMessage);
        } else {
            DEBUG("MQTT", "Failed to publish test message to topic '%s'", MQTT_TOPIC);
        }
    } else {
        DEBUG("MQTT", "MQTT connection failed.");
    }
    
    } else {
        DEBUG("MQTT", "MQTT already connected.");
    }


}