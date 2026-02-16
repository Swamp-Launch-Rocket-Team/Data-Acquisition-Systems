#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_NAU7802.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// --- CONFIGURATION ---

// Custom I2C Pins for Sensor B
#define SDA_CUSTOM 33
#define SCL_CUSTOM 32

// SD Card Chip Select
#define SD_CS 5 

// Log File Name
#define LOG_FILENAME "/datalog.csv"

// --- OBJECTS ---

// "Wire" is the default I2C instance (Pins 21/22)
// We create a second instance for the custom pins
TwoWire I2C_Custom = TwoWire(1);

Adafruit_NAU7802 nauA; // Sensor on Default I2C (A1, A2)
Adafruit_NAU7802 nauB; // Sensor on Custom I2C  (B1, B2)

// Tare values
int offsets[] = {0,0,0,0};

// WIFI Connectivity
const char* ssid = "FBI WebService";
const char* password = "smoothstar803";

WebSocketsServer webSocket = WebSocketsServer(81);

#define LED_PIN 2

unsigned long lastSend = 0;
bool tareNext = false;

// --- HELPER FUNCTIONS ---

void appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (!file.print(message)) {
    Serial.println("Append failed");
  }
  file.close();
}

int32_t readChannel(Adafruit_NAU7802 &sensor, uint8_t channel) {
  sensor.setChannel(channel);
  // Flush 5 readings to let the ADC settle after channel switch
  for (uint8_t i=0; i<5; i++) {
    while (!sensor.available()) delay(1);
    sensor.read();
  }
  // Take the actual reading
  while (!sensor.available()) delay(1);
  return sensor.read();
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_TEXT:
      if (strcmp((char*)payload, "TARE") == 0) {
        tareNext = true;
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected");
  Serial.println(WiFi.localIP());

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Serial.println("\n--- 4-Channel NAU7802 Logger ---");

  // 2. Initialize I2C Buses
  Wire.begin(); // Default I2C (21 SDA, 22 SCL)
  I2C_Custom.begin(SDA_CUSTOM, SCL_CUSTOM); // Custom I2C (33 SDA, 32 SCL)

  // 3. Initialize Sensor A (Default I2C)
  if (!nauA.begin(&Wire)) {
    Serial.println("Failed to find NAU7802 'A' (Default Pins 21/22)");
    while (1) delay(10);
  }
  nauA.setLDO(NAU7802_3V0);
  nauA.setGain(NAU7802_GAIN_128);
  nauA.setRate(NAU7802_RATE_320SPS);
  Serial.println("Sensor A configured.");

  // 4. Initialize Sensor B (Custom I2C)
  if (!nauB.begin(&I2C_Custom)) {
    Serial.println("Failed to find NAU7802 'B' (Custom Pins 33/32)");
    while (1) delay(10);
  }
  nauB.setLDO(NAU7802_3V0);
  nauB.setGain(NAU7802_GAIN_128);
  nauB.setRate(NAU7802_RATE_320SPS);
  Serial.println("Sensor B configured.");

  Serial.println("Logging started...");
}

void loop() {
  webSocket.loop();

  // Send sensor data every 100ms
  if (millis() - lastSend > 100) {
    int32_t valA1, valA2, valB1, valB2;
    unsigned long timestamp = millis();

    // --- Read Sensor A (Default I2C) ---
    valA1 = readChannel(nauA, 0); // Channel 0 -> Label A1
    valA2 = readChannel(nauA, 1); // Channel 1 -> Label A2

    // --- Read Sensor B (Custom I2C) ---
    valB1 = readChannel(nauB, 0); // Channel 0 -> Label B1
    valB2 = readChannel(nauB, 1); // Channel 1 -> Label B2

    if (tareNext) {
      offsets[0] = valA1;
      offsets[1] = valA2;
      offsets[2] = valB1;
      offsets[3] = valB2;
      Serial.println("TARED");
      tareNext = false;
    }

    valA1 -= offsets[0];
    valA2 -= offsets[1];
    valB1 -= offsets[2];
    valB2 -= offsets[3];


    StaticJsonDocument<200> doc;
    doc["time"] = timestamp;
    doc["A1"] = valA1;
    doc["A2"] = valA2;
    doc["B1"] = valB1;
    doc["B2"] = valB2;

    String jsonString;
    serializeJson(doc, jsonString);
    webSocket.broadcastTXT(jsonString);

    lastSend = millis();
  }
}