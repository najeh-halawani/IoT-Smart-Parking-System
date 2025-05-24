#pragma once
#include <LoRaWAN_ESP32.h>
#include <ArduinoJson.h>
#include <time.h>
#include "config.h"
#include "debug.h"
#include "spot_occupancy_data.h"
#include "aes.h"

struct LoRaTaskParams
{
    LoRaWANNode *loraNode;
    QueueHandle_t loraDataQueue;
};

void loraTask(void *pv)
{
    auto *params = static_cast<LoRaTaskParams *>(pv);
    LoRaWANNode *node = params->loraNode;
    QueueHandle_t loraDataQueue = params->loraDataQueue;
    SpotOccupancyData data;

    AES256 aes(aes_key);

    while (true)
    {
        // Check if node is activated
        if (!node->isActivated())
        {
            DEBUG("LORAWAN", "Not activated, attempting OTAA join");
            int16_t state = node->activateOTAA();
            if (state == RADIOLIB_LORAWAN_SESSION_RESTORED)
            {
                DEBUG("LORAWAN", "Session restored");
            }
            else if (state == RADIOLIB_LORAWAN_NEW_SESSION)
            {
                DEBUG("LORAWAN", "New session activated");
            }
            else
            {
                DEBUG("LORAWAN", "Failed to join network with error %d", state);
                vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5s before retrying
                continue;
            }
            persist.saveSession(node); // Save session after successful join
        }

        // Create JSON document
        StaticJsonDocument<512> doc;
        JsonArray spots = doc["spots"].to<JsonArray>();

        // Collect up to MAX_SPOTS_PER_TX to keep payload size manageable
        const int MAX_SPOTS_PER_TX = 3; // Adjust based on LoRaWAN payload limits (e.g., 51 bytes for DR0)
        int spotCount = 0;

        // Collect data from queue
        while (spotCount < MAX_SPOTS_PER_TX && xQueueReceive(loraDataQueue, &data, pdMS_TO_TICKS(100)))
        {
            DEBUG("LORAWAN", "Received spot %d data: occupied=%d, us=%.1f, tof=%.1f",
                  data.spotId, data.occupied, data.usDistance, data.tofDistance);
            JsonObject spot = spots.add<JsonObject>();
            spot["id"] = data.spotId;
            spot["occupied"] = data.occupied;
            spot["us_distance"] = data.usDistance;
            spot["tof_distance"] = data.tofDistance;
            spotCount++;
        }

        // Proceed if data was collected
        if (spotCount > 0)
        {
            // Add UTC timestamp
            time_t now = time(nullptr);
            char timeStr[25];
            strftime(timeStr, sizeof(timeStr), "%Y-%m-%dT%H:%M:%SZ", gmtime(&now));
            doc["timestamp"] = timeStr;

            // Serialize JSON to string
            char jsonBuffer[512];
            size_t jsonLength = serializeJson(doc, jsonBuffer);
            DEBUG("LORAWAN", "Serialized JSON length: %d bytes", jsonLength);

            // Check payload size (LoRaWAN limit: ~51-222 bytes depending on data rate)
            if (jsonLength > 200) // Conservative limit to account for encryption and IV
            {
                DEBUG("LORAWAN", "JSON payload too large (%d bytes). Reducing spots or splitting.", jsonLength);
                vTaskDelay(pdMS_TO_TICKS(1000)); // Wait before retrying
                continue;
            }

            // Pad JSON to multiple of 16 bytes for AES
            uint8_t paddedInput[512];
            size_t paddedLength = (jsonLength + 15) & ~15; // Round up to next 16-byte block
            memcpy(paddedInput, jsonBuffer, jsonLength);
            for (size_t i = jsonLength; i < paddedLength; i++)
            {
                paddedInput[i] = 16 - (jsonLength % 16); // PKCS7 padding
            }

            // Generate unique IV
            uint8_t iv[16];
            generate_iv(iv); 

            // Encrypt the padded JSON
            uint8_t encryptedOutput[512];
            aes.encryptCBC(paddedInput, encryptedOutput, paddedLength, iv);

            // Prepend IV to encrypted data
            uint8_t finalOutput[512 + 16];
            memcpy(finalOutput, iv, 16);
            memcpy(finalOutput + 16, encryptedOutput, paddedLength);

            // Send via LoRaWAN
            uint8_t downlinkData[256];
            size_t lenDown = sizeof(downlinkData);
            size_t totalLength = paddedLength + 16;
            if (totalLength > 222) // Max payload for higher data rates in EU868
            {
                DEBUG("LORAWAN", "Encrypted payload too large (%d bytes). Reducing spots or splitting.", totalLength);
                vTaskDelay(pdMS_TO_TICKS(1000)); // Wait before retrying
                continue;
            }

            int16_t state = node->sendReceive(finalOutput, totalLength, LORAWAN_PORT, downlinkData, &lenDown);
            if (state == RADIOLIB_ERR_NONE)
            {
                DEBUG("LORAWAN", "Published IV + encrypted data (%d bytes), no downlink received", totalLength);
            }
            else if (state > 0)
            {
                DEBUG("LORAWAN", "Published IV + encrypted data (%d bytes), downlink received", totalLength);
            }
            else
            {
                DEBUG("LORAWAN", "Send failed with error %d", state);
                xQueueReset(loraDataQueue); 
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}