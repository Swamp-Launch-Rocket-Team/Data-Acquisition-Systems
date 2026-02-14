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

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- 4-Channel NAU7802 Logger ---");

  // 1. Initialize SD Card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card Mount Failed! Check wiring.");
    return;
  }
  Serial.println("SD Card Initialized.");

  // Write CSV Header (only if file is new)
  if (!SD.exists(LOG_FILENAME)) {
    // Header matches your labels: Time, A1, A2, B1, B2
    appendFile(SD, LOG_FILENAME, "Time_ms,A1,A2,B1,B2\n");
  }

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
  int32_t valA1, valA2, valB1, valB2;
  unsigned long timestamp = millis();

  // --- Read Sensor A (Default I2C) ---
  valA1 = readChannel(nauA, 0); // Channel 0 -> Label A1
  valA2 = readChannel(nauA, 1); // Channel 1 -> Label A2

  // --- Read Sensor B (Custom I2C) ---
  valB1 = readChannel(nauB, 0); // Channel 0 -> Label B1
  valB2 = readChannel(nauB, 1); // Channel 1 -> Label B2

  // --- Format Data for CSV ---
  String dataString = String(timestamp) + "," + 
                      String(valA1) + "," + 
                      String(valA2) + "," + 
                      String(valB1) + "," + 
                      String(valB2) + "\n";

  // --- Log to SD ---
  appendFile(SD, LOG_FILENAME, dataString.c_str());

  // --- Print to Serial ---
  Serial.print("Time: "); Serial.print(timestamp);
  Serial.print(" | A1: "); Serial.print(valA1);
  Serial.print(" | A2: "); Serial.print(valA2);
  Serial.print(" | B1: "); Serial.print(valB1);
  Serial.print(" | B2: "); Serial.println(valB2);
}
