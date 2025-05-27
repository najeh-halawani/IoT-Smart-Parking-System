// #pragma once
// #include <Arduino.h>
// #include <heltec.h>
// #include "LoRaWan_APP.h"
// #include <ArduinoJson.h>
// #include "config.h"
// #include "debug.h"
// #include "spot_occupancy_data.h"
// #include "aes.h"

// // LoRaWAN Parameters
// extern uint32_t license[4];
// extern uint8_t devEui[];
// extern uint8_t appEui[];
// extern uint8_t appKey[];
// extern uint8_t nwkSKey[];
// extern uint8_t appSKey[];
// extern uint32_t devAddr;
// extern uint16_t userChannelsMask[6];
// extern LoRaMacRegion_t loraWanRegion;
// extern DeviceClass_t loraWanClass;
// extern bool overTheAirActivation;
// extern bool loraWanAdr;
// extern bool isTxConfirmed;
// extern uint8_t appPort;
// extern uint8_t confirmedNbTrials;
// extern uint32_t appTxDutyCycle;

// struct LoRaTaskParams {
//     QueueHandle_t loraDataQueue;
// };

// // Prepare LoRaWAN frame with encrypted data
// static void prepareTxFrame(uint8_t port, SpotOccupancyData* data, AES256* aes) {
//     // Create JSON document for data
//     StaticJsonDocument<256> doc;
//     doc["id"] = data->spotId;
//     doc["occ"] = data->occupied;
//     doc["us"] = data->usDistance;
//     doc["tof"] = data->tofDistance;
//     doc["ts"] = (uint32_t)time(nullptr);

//     // Serialize to JSON
//     char jsonBuffer[256];
//     size_t jsonLength = serializeJson(doc, jsonBuffer);
//     DEBUG("LORAWAN", "JSON data: %s", jsonBuffer);

//     // Pad data for AES encryption (16-byte blocks)
//     uint8_t paddedData[256];
//     size_t paddedLength = (jsonLength + 15) & ~15;
//     memcpy(paddedData, jsonBuffer, jsonLength);
    
//     // Add PKCS7 padding
//     uint8_t paddingValue = paddedLength - jsonLength;
//     for (size_t i = jsonLength; i < paddedLength; i++) {
//         paddedData[i] = paddingValue;
//     }

//     // Generate IV
//     uint8_t iv[16];
//     generate_iv(iv);

//     // Encrypt data
//     uint8_t encryptedData[256];
//     aes->encryptCBC(paddedData, encryptedData, paddedLength, iv);

//     // Prepare final payload: IV + encrypted data
//     // Make sure it doesn't exceed maximum payload size
//     size_t maxPayloadSize = 51; // Maximum for DR0 in EU868
//     size_t totalLength = 16 + paddedLength; // IV + encrypted data
    
//     if (totalLength > maxPayloadSize) {
//         DEBUG("LORAWAN", "Payload too large (%d bytes), truncating", totalLength);
//         totalLength = maxPayloadSize;
//         paddedLength = maxPayloadSize - 16;
//     }

//     // Copy IV and encrypted data to appData
//     appDataSize = totalLength;
//     memcpy(appData, iv, 16);
//     memcpy(appData + 16, encryptedData, paddedLength);

//     DEBUG("LORAWAN", "Prepared frame: %d bytes (IV: 16, Data: %d)", appDataSize, paddedLength);
// }

// void loraTask(void* pvParameters) {
//     auto* params = static_cast<LoRaTaskParams*>(pvParameters);
//     QueueHandle_t loraDataQueue = params->loraDataQueue;
    
//     // Initialize AES
//     AES256 aes(aes_key);
    
//     DEBUG("LORAWAN", "Starting LoRa task on core %d", xPortGetCoreID());

//     // Initialize Heltec board

//     // Initialize LoRaWAN
// // #if (LORAWAN_DEVEUI_AUTO)
// //     LoRaWAN.generateDeveuiByChipID();
// // #endif
//     LoRaWAN.init(loraWanClass, loraWanRegion);
//     LoRaWAN.setDefaultDR(3);
//     deviceState = DEVICE_STATE_JOIN;

//     SpotOccupancyData data;
//     uint32_t txDutyCycleTime = 0;
//     TickType_t lastWakeTime = xTaskGetTickCount();

//     while (true) {
//         switch (deviceState) {
//             case DEVICE_STATE_INIT:
//                 DEBUG("LORAWAN", "Initializing...");
//                 deviceState = DEVICE_STATE_JOIN;
//                 break;

//             case DEVICE_STATE_JOIN:
//                 DEBUG("LORAWAN", "Joining network...");
//                 LoRaWAN.join();
//                 break;

//             case DEVICE_STATE_SEND:
//                 if (xQueueReceive(loraDataQueue, &data, 0) == pdTRUE) {
//                     DEBUG("LORAWAN", "Sending data for spot %d", data.spotId);
//                     prepareTxFrame(appPort, &data, &aes);
//                     LoRaWAN.send();
//                 }
//                 deviceState = DEVICE_STATE_CYCLE;
//                 break;

//             case DEVICE_STATE_CYCLE:
//                 txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
//                 DEBUG("LORAWAN", "Cycle: waiting %lu ms", txDutyCycleTime);
//                 LoRaWAN.cycle(txDutyCycleTime);
//                 deviceState = DEVICE_STATE_SLEEP;
//                 break;

//             case DEVICE_STATE_SLEEP:
//                 DEBUG("LORAWAN", "Entering sleep mode");
//                 LoRaWAN.sleep(loraWanClass);
//                 break;

//             default:
//                 deviceState = DEVICE_STATE_INIT;
//                 break;
//         }

//         // Allow other tasks to run and respect duty cycle
//         vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100));
//     }
// }