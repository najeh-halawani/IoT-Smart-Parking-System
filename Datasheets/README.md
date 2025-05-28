# IoT Smart Parking System 🚗📡

Final delivery of our IoT group project at Sapienza University of Rome. This project aims to detect parking spot occupancy using a hybrid sensor approach (ultrasonic + ToF), powered by low-energy microcontrollers, and transmitting data securely via LoRa and Wi-Fi/MQTT.

## 👥 Team
- Najeh Alhalawani  
- José Edgar Hernández  
- Marcelo Jiménez

---

## 🧠 Project Overview

We designed and deployed a smart sensing platform to:
- Detect parking spot occupancy with ≥90% accuracy.
- Transmit data wirelessly in near real-time.
- Visualize availability and collect user-validated reports via mobile crowdsensing.

Prototype tested outside the DIAG building using a single node mounted on the curb.

---

## ✅ Functional Requirements

| Code | Description |
|------|-------------|
| FR-1 | Detect vehicle presence with ≥90% accuracy |
| FR-2 | Update cloud within 5 minutes + 5 seconds after occupancy change |
| FR-3 | Secure data transmission and server connection |

---

## ⚙️ Non-Functional Requirements

| Code | Description |
|------|-------------|
| NFR-1 | ≥180 days autonomy with a 3.7 V 10,000 mAh LiPo battery |
| NFR-2 | BOM ≤ €15 per node (at 100 units) |
| NFR-3 | Support ≥4 sensors per MCU |
| NFR-4 | Wireless packet loss < 5% in 24h |

---

## 🛠️ Hardware Components

| Component | Description | Price (est.) |
|----------|-------------|--------------|
| ESP32 LoRa Heltec V3 | MCU with LoRa SX1262 | €17.50 |
| VL53L1X | ToF Laser sensor | €4.18 |
| HC-SR04 | Ultrasonic sensor | €1.59 |
| LiPo 3.7V 10,000 mAh | Battery | €9.39 |
| IP65 Enclosure | Weatherproof housing | €2.07 |

[🔗 Verify BOM Pricing on LCSC](https://lcsc.com)

---

## 🧰 System Architecture

- **Dual-core ESP32 task separation:**
  - Core 0: system state, logic, deep sleep
  - Core 1: sensor polling, MQTT, LoRa
- **Sensor Fusion Logic:**
  - Ultrasonic → triggers
  - Laser → confirms
- **Security:** AES-256 encryption over MQTT with TLS
- **Crowdsensing:** App with QR code validation + incentives

---

## 🔐 Security Pipeline

1. Serialize sensor reading as JSON  
2. Apply PKCS7 padding  
3. Encrypt using AES-256  
4. Prepend IV  
5. Publish over MQTT TLS  
6. Cloud decrypts and stores  

---

## 🧪 Field Test Results

| Metric        | Value            |
|---------------|------------------|
| Accuracy      | ≈91.7% (33/36)   |
| Confidence CI | ±8% @ 95% level  |
| Test Duration | 6 minutes        |
| Repetitions   | 36 transitions   |

📊 [Confusion Matrix](#)

---

## 📱 Frontend

- Displays real-time parking availability
- Enables user ground-truth validation via photo
- Rewards validation with temporary app features

---

## 🔋 Battery Projection

With deep sleep cycles and efficient polling, our system can run for over **180 days** without charging or maintenance.

---

## 🧪 Requirement Verification

| Requirement | Met | Verification |
|-------------|-----|--------------|
| FR-1        | ✅  | Field tests  |
| FR-2        | ✅  | RTOS logic   |
| FR-3        | ✅  | Encrypted MQTT |
| NFR-1       | ✅  | Battery simulation |
| NFR-2       | ✅  | BOM check |
| NFR-3       | ✅  | I²C Multiplexing |

---

## 📁 Repository Structure

```bash
📦smart-parking-iot
 ┣ 📂firmware
 ┃ ┣ 📜main.cpp
 ┃ ┣ 📜tasks.cpp
 ┃ ┗ 📜config.h
 ┣ 📂frontend
 ┃ ┗ 📜app_mockup.png
 ┣ 📂docs
 ┃ ┗ 📜IoT_Presentation.pdf
 ┣ 📜README.md
 ┗ 📜LICENSE
```

---

## 📄 License

MIT License. See `LICENSE` file.