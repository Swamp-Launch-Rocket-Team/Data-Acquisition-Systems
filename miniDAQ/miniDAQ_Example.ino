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

// Appends a string to the SD card file
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

// Reads a channel and flushes stale data
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

// Robust Initialization Function with Retries
bool initSensorWithRetry(Adafruit_NAU7802 &sensor, TwoWire *bus, const char* sensorName, int maxRetries = 5) {
  int attempts = 0;
  
  while (attempts < maxRetries) {
    if (sensor.begin(bus)) {
      // If successful, configure the sensor
      sensor.setLDO(NAU7802_3V0);
      sensor.setGain(NAU7802_GAIN_128);
      sensor.setRate(NAU7802_RATE_320SPS);
      Serial.print("Sensor "); Serial.print(sensorName); Serial.println(" configured successfully.");
      return true; // Success!
    }
    
    attempts++;
    Serial.print("Failed to find Sensor "); Serial.print(sensorName);
    Serial.print(". Attempt "); Serial.print(attempts); Serial.print(" of "); Serial.println(maxRetries);
    
    // Wait a bit before trying again
    delay(500); 
  }
  
  return false; // Failed after all retries
}

void setup() {
  Serial.begin(115200);
  
  // Wait to let sensors time-out of any hanging I2C transactions
  // This helps prevent "stuck bus" issues after pressing the EN button
  delay(1000); 

  Serial.println("\n--- 4-Channel NAU7802 SD Logger ---");

  // 1. Initialize SD Card
  if (!SD.begin(SD_CS)) {
    Serial.println("CRITICAL: SD Card Mount Failed! Check wiring.");
    while (1) delay(10); // Halt if SD fails
  }
  Serial.println("SD Card Initialized.");

  // Write CSV Header (only if file is new)
  if (!SD.exists(LOG_FILENAME)) {
    appendFile(SD, LOG_FILENAME, "Time_ms,A1,A2,B1,B2\n");
  }

  // 2. Initialize I2C Buses
  Wire.begin(); // Default I2C (21 SDA, 22 SCL)
  I2C_Custom.begin(SDA_CUSTOM, SCL_CUSTOM); // Custom I2C (33 SDA, 32 SCL)

  // 3. Initialize Sensor A (Default I2C) with Retries
  if (!initSensorWithRetry(nauA, &Wire, "A")) {
    Serial.println("CRITICAL: Sensor A could not be initialized. Halting.");
    while (1) delay(10);
  }

  // 4. Initialize Sensor B (Custom I2C) with Retries
  if (!initSensorWithRetry(nauB, &I2C_Custom, "B")) {
    Serial.println("CRITICAL: Sensor B could not be initialized. Halting.");
    while (1) delay(10);
  }

  Serial.println("Logging started...");
}

void loop() {
  int32_t valA1, valA2, valB1, valB2;
  unsigned long timestamp = millis();

  // --- Read Sensor A (Default I2C) ---
  valA1 = readChannel(nauA, 0); 
  valA2 = readChannel(nauA, 1); 

  // --- Read Sensor B (Custom I2C) ---
  valB1 = readChannel(nauB, 0); 
  valB2 = readChannel(nauB, 1); 

  // --- Format Data for CSV ---
  String dataString = String(timestamp) + "," + 
                      String(valA1) + "," + 
                      String(valA2) + "," + 
                      String(valB1) + "," + 
                      String(valB2) + "\n";

  // --- Log to SD ---
  appendFile(SD, LOG_FILENAME, dataString.c_str());

  // --- Print to Serial ---
  Serial.print("Logged -> Time: "); Serial.print(timestamp);
  Serial.print(" | A1: "); Serial.print(valA1);
  Serial.print(" | A2: "); Serial.print(valA2);
  Serial.print(" | B1: "); Serial.print(valB1);
  Serial.print(" | B2: "); Serial.println(valB2);
}
