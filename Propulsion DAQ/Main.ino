/*
  Robust DAQ with machine-friendly Serial Protocol
  - Teensy 4.1
  - SD.h for SD card
  - Sequential filenames DATA1.csv, DATA2.csv...
  - LED_PIN = 2, SWITCH_PIN = 41 (INPUT_PULLUP)
  - Commands via Serial (USB). Replies are single-line JSON objects.
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>
#include "Adafruit_MCP9601.h"
#include <EEPROM.h>

// ---------- Pin configuration (user requested) ----------
const int PRESSURE_PIN = 22;
const int SWITCH_PIN = 41;    // INPUT_PULLUP
const int LED_PIN = 2;        // LED on pin 2
const int SD_CHIP_SELECT = BUILTIN_SDCARD; // if not defined replace appropriately

// ---------- I2C addresses ----------
const uint8_t MCP9601_I2C_ADDR = 0x67;

// ---------- Constants ----------
const int LED_BRIGHTNESS_STANDBY = 1;
const int LED_BRIGHTNESS_ACTIVE = 50;
const int LED_BRIGHTNESS_ERROR = 150;

const float PRESSURE_MAX = 2500.0;
const float VOLTAGE_MIN = 0.3;
const float VOLTAGE_MAX = 3.0;
const float VOLTAGE_RANGE = VOLTAGE_MAX - VOLTAGE_MIN;

const int BUFFER_SIZE = 4096;
static char dataBuffer[BUFFER_SIZE];
static int bufferPos = 0;
unsigned long lastFlushTime = 0;
const unsigned long FLUSH_INTERVAL_MS = 500;

File dataFile;
String filename;
bool isRecording = false;

// Calibration
float zeroOffset = 0.0;
float calibrationFactor = 1.0;
bool isCalibrated = false;

// EEPROM layout
#pragma pack(push,1)
struct CalData {
  uint32_t magic;
  float zeroOffset;
  float calibFactor;
  uint16_t crc16;
};
#pragma pack(pop)
const uint32_t EEPROM_MAGIC = 0xDA7A5CA1UL;
const int EEPROM_ADDR_CALSTRUCT = 0;      // start for CalData
const int EEPROM_ADDR_FILE_COUNTER = 128; // fileCounter store

// file counter
uint32_t fileCounter = 0; // persisted

// Peripherals
NAU7802 scaleHandler;
Adafruit_MCP9601 thermocouple;
bool thermocoupleWorking = false;

// Switch state
bool initialSwitchState = false;
bool previousSwitchState = false;

// Timeouts / retries
const unsigned long SCALE_WAIT_TIMEOUT_MS = 500;
const unsigned long SERIAL_WAIT_TIMEOUT_MS = 5000;
const int I2C_BEGIN_RETRIES = 3;
const unsigned long I2C_RETRY_DELAY_MS = 100;

// Serial parser
const int CMD_BUF_SIZE = 256;
char cmdBuf[CMD_BUF_SIZE];
int cmdPos = 0;

// Current LED brightness
int currentLedBrightness = LED_BRIGHTNESS_STANDBY;

// Firmware version string
const char* FW_VERSION = "DAQ-v1.1";

// Prototypes
void setup();
void loop();
void processSerialCommand(const char* line);
String mkJson(bool ok, const char* cmd, const String &msg = "", const String &data = "");
void addToBuffer(const char* data);
void flushBuffer();
bool openDataFile(const char* fname);
void createFilename(); // sequential using fileCounter
void startRecording();
void stopRecording();
void collectAndLogData();
float calculatePressure(float voltage);
bool waitForScaleAvailable(unsigned long timeoutMs);
bool getAvgScaleReading(long &outReading, int samples=5, unsigned long timeoutPerSample=200);
void saveCalibrationData();
void loadCalibrationData();
void saveFileCounter();
void loadFileCounter();
void tryReinitializePeripherals();
uint16_t crc16_ccitt(const uint8_t *buf, size_t len);
uint16_t crc16_ccitt_update(uint16_t crc, uint8_t a);
void errorStateBlink(int errorCode);
String listFilesOnSD();

// ---------- CRC16 ----------
uint16_t crc16_ccitt_update(uint16_t crc, uint8_t a) {
  crc ^= (uint16_t)a << 8;
  for (int i=0;i<8;i++) {
    if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
    else crc <<= 1;
  }
  return crc;
}
uint16_t crc16_ccitt(const uint8_t *buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i=0;i<len;i++) crc = crc16_ccitt_update(crc, buf[i]);
  return crc;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("{\"ok\":true,\"cmd\":\"BOOT\",\"msg\":\"Booting...\"}");

  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  analogWriteResolution(8);
  analogWrite(LED_PIN, 0);
  analogWrite(LED_PIN, LED_BRIGHTNESS_STANDBY);
  currentLedBrightness = LED_BRIGHTNESS_STANDBY;

  delay(10);
  initialSwitchState = digitalRead(SWITCH_PIN);
  previousSwitchState = initialSwitchState;

  // I2C init
  Wire.begin();
  Wire.setClock(100000);

  // Initialize scale
  if (!scaleHandler.begin()) {
    bool ok = false;
    for (int i=0;i<I2C_BEGIN_RETRIES;i++) {
      delay(I2C_RETRY_DELAY_MS);
      if (scaleHandler.begin()) { ok = true; break; }
    }
    if (!ok) {
      Serial.println(mkJson(false,"INIT","Scale init failed after retries"));
      errorStateBlink(2);
    }
  }
  scaleHandler.setGain(NAU7802_GAIN_64);
  scaleHandler.setSampleRate(NAU7802_SPS_320);
  scaleHandler.calibrateAFE();

  // Load cal data and file counter
  loadCalibrationData();
  loadFileCounter();

  // Thermocouple begin, multiple times as required
  bool thermoOk = false;
  for (int i=0;i<I2C_BEGIN_RETRIES;i++) {
    if (thermocouple.begin(MCP9601_I2C_ADDR)) { thermoOk = true; break; }
    delay(I2C_RETRY_DELAY_MS);
  }
  if (thermoOk) {
    thermocouple.setADCresolution(MCP9600_ADCRESOLUTION_14);
    thermocouple.setAmbientResolution(RES_ZERO_POINT_25);
    thermocouple.setFilterCoefficient(0);
    thermocouple.setThermocoupleType(MCP9600_TYPE_K);
    thermocouple.enable(true);
    thermocoupleWorking = true;
  } else {
    thermocoupleWorking = false;
    Serial.println(mkJson(false,"INIT","Thermocouple not detected"));
    for (int i=0;i<3;i++) { analogWrite(LED_PIN, LED_BRIGHTNESS_ERROR); delay(100); analogWrite(LED_PIN,0); delay(100); }
  }

  Wire.setClock(400000);
  analogReadResolution(12);

  // SD init (SD.h)
  if (!SD.begin(SD_CHIP_SELECT)) {
    Serial.println(mkJson(false,"INIT","SD init failed"));
    errorStateBlink(4);
  } else {
    Serial.println(mkJson(true,"INIT","SD initialized"));
  }

  // Small ready message (JSON)
  Serial.println(mkJson(true,"READY","System ready"));
}

// ---------- Main loop ----------
void loop() {
  // Read serial (non-blocking)
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue; // ignore CR
    if (c == '\n') {
      // terminate and process
      cmdBuf[cmdPos] = 0;
      if (cmdPos > 0) {
        processSerialCommand(cmdBuf);
      }
      cmdPos = 0;
    } else {
      if (cmdPos < CMD_BUF_SIZE - 1) {
        cmdBuf[cmdPos++] = c;
      } else {
        // overflow: clear and warn
        cmdPos = 0;
        Serial.println(mkJson(false,"PARSE","Command buffer overflow"));
      }
    }
  }

  // Handle switch-based start/stop (unchanged behavior)
  bool currentSwitchState = digitalRead(SWITCH_PIN);
  if (currentSwitchState != previousSwitchState) {
    delay(50);
    currentSwitchState = digitalRead(SWITCH_PIN);
    if (currentSwitchState != previousSwitchState) {
      if (currentSwitchState != initialSwitchState) {
        if (!isRecording) {
          startRecording();
          Serial.println(mkJson(true,"START","Recording started", String("{\"filename\":\""+filename+"\"}")));
        } else {
          stopRecording();
          Serial.println(mkJson(true,"STOP","Recording stopped"));
        }
      } else {
        if (isRecording) {
          stopRecording();
          Serial.println(mkJson(true,"STOP","Recording stopped (switch returned)"));
        }
      }
      previousSwitchState = currentSwitchState;
    }
  }

  // If recording, collect and flush periodically
  if (isRecording) {
    collectAndLogData();
    unsigned long now = millis();
    if ((now - lastFlushTime >= FLUSH_INTERVAL_MS) || (bufferPos > BUFFER_SIZE - 200)) {
      flushBuffer();
      lastFlushTime = now;
    }
  }
}

// ---------- Serial command processor ----------
void processSerialCommand(const char* line) {
  // parse command and args (space-separated)
  // allow case-insensitive commands
  String s = String(line);
  s.trim();
  if (s.length() == 0) {
    Serial.println(mkJson(false,"PARSE","Empty command"));
    return;
  }

  // Split at spaces: first token is command
  int sp = s.indexOf(' ');
  String cmd = (sp == -1) ? s : s.substring(0, sp);
  String rest = (sp == -1) ? "" : s.substring(sp + 1);
  cmd.toUpperCase();

  // Handle commands
  if (cmd == "PING") {
    String data = String("{\"fw\":\"") + FW_VERSION + String("\"}");
    Serial.println(mkJson(true,"PING","pong", data));
    return;
  }

  if (cmd == "STATUS") {
    bool sdPresent = SD.begin(SD_CHIP_SELECT); // cheap check
    String data = "{";
    data += "\"recording\":";
    data += isRecording ? "true" : "false";
    data += ",\"isCalibrated\":";
    data += isCalibrated ? "true" : "false";
    data += ",\"thermocoupleWorking\":";
    data += thermocoupleWorking ? "true":"false";
    data += ",\"fileCounter\":";
    data += String(fileCounter);
    data += ",\"currentFilename\":\"";
    data += filename;
    data += "\",\"sdPresent\":";
    data += sdPresent ? "true":"false";
    data += ",\"switchState\":";
    data += String(digitalRead(SWITCH_PIN));
    data += "}";
    Serial.println(mkJson(true,"STATUS","", data));
    return;
  }

  if (cmd == "START") {
    if (!isRecording) {
      startRecording();
      String data = String("{\"filename\":\"") + filename + String("\"}");
      Serial.println(mkJson(true,"START","Recording started", data));
    } else {
      Serial.println(mkJson(false,"START","Already recording"));
    }
    return;
  }

  if (cmd == "STOP") {
    if (isRecording) {
      stopRecording();
      Serial.println(mkJson(true,"STOP","Recording stopped"));
    } else {
      Serial.println(mkJson(false,"STOP","Not recording"));
    }
    return;
  }

  if (cmd == "TARE") {
    // Tare non-interactive
    long zeroSum = 0;
    int samples = 10;
    for (int i=0;i<samples;i++) {
      long single;
      if (!getAvgScaleReading(single,1,SCALE_WAIT_TIMEOUT_MS)) {
        Serial.println(mkJson(false,"TARE","Timeout reading scale"));
        return;
      }
      zeroSum += single;
      delay(50);
    }
    zeroOffset = (float)zeroSum / samples;
    saveCalibrationData(); // saves and sets isCalibrated
    String data = String("{\"zeroOffset\":") + String(zeroOffset,2) + String("}");
    Serial.println(mkJson(true,"TARE","Tare complete", data));
    return;
  }

  if (cmd == "CALIBRATE") {
    // Expect argument: known weight in grams. e.g. "CALIBRATE 100.0"
    rest.trim();
    if (rest.length() == 0) {
      Serial.println(mkJson(false,"CALIBRATE","Missing weight argument. Use: CALIBRATE <grams>"));
      return;
    }
    float knownWeight = rest.toFloat();
    if (knownWeight <= 0.0f) {
      Serial.println(mkJson(false,"CALIBRATE","Invalid weight argument"));
      return;
    }

    // Step 1: zero readings
    long zeroSum = 0;
    int samples = 10;
    for (int i=0;i<samples;i++) {
      long single;
      if (!getAvgScaleReading(single,1,SCALE_WAIT_TIMEOUT_MS)) {
        Serial.println(mkJson(false,"CALIBRATE","Timeout reading zero from scale"));
        return;
      }
      zeroSum += single;
      delay(50);
    }
    zeroOffset = (float)zeroSum / samples;

    // Step 2: known weight readings
    long weightSum = 0;
    for (int i=0;i<samples;i++) {
      long single;
      if (!getAvgScaleReading(single,1,SCALE_WAIT_TIMEOUT_MS)) {
        Serial.println(mkJson(false,"CALIBRATE","Timeout reading weight from scale"));
        return;
      }
      weightSum += single;
      delay(50);
    }
    float weightReading = (float)weightSum / samples;
    float diff = weightReading - zeroOffset;
    if (diff <= 0.0f) {
      Serial.println(mkJson(false,"CALIBRATE","Weight reading <= zero reading; check weight"));
      return;
    }
    calibrationFactor = knownWeight / diff;
    isCalibrated = true;
    saveCalibrationData();
    String data = String("{\"zeroOffset\":") + String(zeroOffset,2) + String(",\"calibrationFactor\":") + String(calibrationFactor,6) + String("}");
    Serial.println(mkJson(true,"CALIBRATE","Calibration complete", data));
    return;
  }

  if (cmd == "READ") {
    // Single sample - does not affect recording state
    unsigned long timestamp = micros();

    long rawReading = 0;
    float thrust = 0.0f;
    if (scaleHandler.available()) {
      if (waitForScaleAvailable(SCALE_WAIT_TIMEOUT_MS)) {
        rawReading = scaleHandler.getReading();
      }
    }
    thrust = calibrationFactor * ((float)rawReading - zeroOffset);

    float temperature = -999.99f;
    if (thermocoupleWorking) {
      thermocouple.begin(MCP9601_I2C_ADDR);
      temperature = thermocouple.readThermocouple();
    }

    int rawPressure = analogRead(PRESSURE_PIN);
    float voltage = (rawPressure * 3.3f) / 4095.0f;
    float pressure = calculatePressure(voltage);

    String data = "{";
    data += "\"timestamp\":";
    data += String(timestamp);
    data += ",\"thrust\":";
    data += String(thrust,2);
    data += ",\"temperature\":";
    data += String(temperature,2);
    data += ",\"pressure\":";
    data += String(pressure,2);
    data += "}";
    Serial.println(mkJson(true,"READ","", data));
    return;
  }

  if (cmd == "TEST_TEMP") {
    if (!thermocoupleWorking) {
      Serial.println(mkJson(false,"TEST_TEMP","Thermocouple not available"));
      return;
    }
    thermocouple.begin(MCP9601_I2C_ADDR);
    float hot = thermocouple.readThermocouple();
    float cold = thermocouple.readAmbient();
    String data = "{";
    data += "\"hot\":";
    data += String(hot,2);
    data += ",\"cold\":";
    data += String(cold,2);
    data += "}";
    Serial.println(mkJson(true,"TEST_TEMP","", data));
    return;
  }

  if (cmd == "TEST_PRESSURE") {
    int raw = analogRead(PRESSURE_PIN);
    float v = (raw * 3.3f) / 4095.0f;
    float p = calculatePressure(v);
    String data = "{";
    data += "\"raw\":";
    data += String(raw);
    data += ",\"voltage\":";
    data += String(v,3);
    data += ",\"pressure\":";
    data += String(p,2);
    data += "}";
    Serial.println(mkJson(true,"TEST_PRESSURE","", data));
    return;
  }

  if (cmd == "RESET_COUNTER") {
    // require explicit "CONFIRM" argument to avoid accidental resets
    rest.trim();
    rest.toUpperCase();
    if (rest != "CONFIRM") {
      Serial.println(mkJson(false,"RESET_COUNTER","Missing CONFIRM argument. Use: RESET_COUNTER CONFIRM"));
      return;
    }
    fileCounter = 0;
    saveFileCounter();
    Serial.println(mkJson(true,"RESET_COUNTER","File counter reset to 0"));
    return;
  }

  if (cmd == "LIST_FILES") {
    String files = listFilesOnSD();
    String data = String("{\"files\":[") + files + String("]}"); // files is CSV of quoted names
    Serial.println(mkJson(true,"LIST_FILES","", data));
    return;
  }

  if (cmd == "SET_LED") {
    rest.trim();
    if (rest.length() == 0) {
      Serial.println(mkJson(false,"SET_LED","Missing brightness argument"));
      return;
    }
    int b = rest.toInt();
    if (b < 0) b = 0;
    if (b > 255) b = 255;
    analogWrite(LED_PIN, b);
    currentLedBrightness = b;
    Serial.println(mkJson(true,"SET_LED","LED set", String("{\"brightness\":" + String(b) + "}")));
    return;
  }

  if (cmd == "GET_LED") {
    Serial.println(mkJson(true,"GET_LED","", String("{\"brightness\":" + String(currentLedBrightness) + "}")));
    return;
  }

  if (cmd == "HELP") {
    String msg = "Commands: PING,STATUS,START,STOP,TARE,CALIBRATE <g>,READ,TEST_TEMP,TEST_PRESSURE,RESET_COUNTER CONFIRM,LIST_FILES,SET_LED <0-255>,GET_LED";
    Serial.println(mkJson(true,"HELP", msg));
    return;
  }

  // Unknown command
  Serial.println(mkJson(false,"UNKNOWN","Unknown command"));
}

// ---------- Helpers ----------
// Build single-line JSON result
String mkJson(bool ok, const char* cmd, const String &msg, const String &data) {
  String s = "{";
  s += "\"ok\":";
  s += (ok ? "true":"false");
  s += ",\"cmd\":\"";
  s += cmd;
  s += "\"";
  if (msg.length() > 0) {
    s += ",\"msg\":\"";
    // escape quotes in msg minimally
    String esc = msg;
    esc.replace("\"", "\\\"");
    s += esc;
    s += "\"";
  }
  if (data.length() > 0) {
    s += ",\"data\":";
    s += data;
  }
  s += "}";
  return s;
}

// ---------- Buffering & SD ----------
void addToBuffer(const char* data) {
  size_t dataLen = strlen(data);
  if (dataLen == 0) return;
  if ((int)dataLen >= BUFFER_SIZE) {
    if (dataFile) {
      size_t written = dataFile.write((const uint8_t*)data, dataLen);
      dataFile.flush();
      if (written != dataLen) {
        Serial.println(mkJson(false,"SD_WRITE","Direct oversized write failed"));
      }
    } else {
      Serial.println(mkJson(false,"SD_WRITE","Oversized data and no file open"));
    }
    return;
  }
  if (bufferPos + (int)dataLen >= BUFFER_SIZE) {
    flushBuffer();
  }
  memcpy(&dataBuffer[bufferPos], data, dataLen);
  bufferPos += dataLen;
}

void flushBuffer() {
  if (bufferPos == 0) return;
  if (!dataFile) {
    bufferPos = 0;
    Serial.println(mkJson(false,"SD_FLUSH","No file open; dropping buffer"));
    return;
  }
  size_t written = dataFile.write((const uint8_t*)dataBuffer, bufferPos);
  dataFile.flush();
  if (written != (size_t)bufferPos) {
    Serial.println(mkJson(false,"SD_FLUSH","Short write occurred"));
    dataFile.close();
    if (!SD.begin(SD_CHIP_SELECT)) {
      Serial.println(mkJson(false,"SD_RECOVER","Remount failed"));
    } else {
      if (!openDataFile(filename.c_str())) {
        Serial.println(mkJson(false,"SD_RECOVER","Reopen failed"));
      }
    }
  }
  bufferPos = 0;
}

bool openDataFile(const char* fname) {
  if (dataFile) dataFile.close();
  dataFile = SD.open(fname, FILE_WRITE);
  if (!dataFile) {
    return false;
  }
  return true;
}

// sequential filename persisted in fileCounter (EEPROM)
void loadFileCounter() {
  uint32_t val = 0;
  EEPROM.get(EEPROM_ADDR_FILE_COUNTER, val);
  if (val > 1000000000UL) val = 0;
  fileCounter = val;
}

void saveFileCounter() {
  EEPROM.put(EEPROM_ADDR_FILE_COUNTER, fileCounter);
  delay(5);
  uint32_t check;
  EEPROM.get(EEPROM_ADDR_FILE_COUNTER, check);
  if (check != fileCounter) {
    Serial.println(mkJson(false,"EEPROM","File counter write verification failed"));
  }
}

void createFilename() {
  uint32_t candidate = fileCounter + 1;
  const int MAX_ATTEMPTS = 100000;
  char candidateName[32];
  int attempts = 0;
  while (attempts < MAX_ATTEMPTS) {
    snprintf(candidateName, sizeof(candidateName), "DATA%lu.csv", (unsigned long)candidate);
    if (!SD.exists(candidateName)) {
      filename = String(candidateName);
      fileCounter = candidate;
      saveFileCounter();
      return;
    }
    candidate++;
    attempts++;
  }
  // fallback
  snprintf(candidateName, sizeof(candidateName), "DATA_%lu.csv", (unsigned long)millis());
  filename = String(candidateName);
}

// ---------- Calibration persistence ----------
void saveCalibrationData() {
  CalData cd;
  cd.magic = EEPROM_MAGIC;
  cd.zeroOffset = zeroOffset;
  cd.calibFactor = calibrationFactor;
  uint8_t* p = (uint8_t*)&cd;
  cd.crc16 = crc16_ccitt(p, sizeof(CalData) - sizeof(cd.crc16));
  EEPROM.put(EEPROM_ADDR_CALSTRUCT, cd);
  delay(5);
  CalData check;
  EEPROM.get(EEPROM_ADDR_CALSTRUCT, check);
  uint16_t verifyCrc = crc16_ccitt((uint8_t*)&check, sizeof(CalData) - sizeof(check.crc16));
  if (check.magic != cd.magic || verifyCrc != check.crc16) {
    isCalibrated = false;
    Serial.println(mkJson(false,"EEPROM","Calibration EEPROM verify failed"));
  } else {
    isCalibrated = true;
  }
}

void loadCalibrationData() {
  CalData cd;
  EEPROM.get(EEPROM_ADDR_CALSTRUCT, cd);
  if (cd.magic != EEPROM_MAGIC) {
    isCalibrated = false;
    zeroOffset = 0.0;
    calibrationFactor = 1.0;
    return;
  }
  uint16_t crc = crc16_ccitt((uint8_t*)&cd, sizeof(CalData) - sizeof(cd.crc16));
  if (crc != cd.crc16) {
    isCalibrated = false;
    zeroOffset = 0.0;
    calibrationFactor = 1.0;
    return;
  }
  zeroOffset = cd.zeroOffset;
  calibrationFactor = cd.calibFactor;
  isCalibrated = true;
}

// ---------- Sensor helpers ----------
bool waitForScaleAvailable(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (!scaleHandler.available()) {
    if (millis() - start >= timeoutMs) return false;
    delay(2);
  }
  return true;
}

bool getAvgScaleReading(long &outReading, int samples, unsigned long timeoutPerSample) {
  long sum = 0;
  for (int i=0;i<samples;i++) {
    if (!waitForScaleAvailable(timeoutPerSample)) return false;
    sum += scaleHandler.getReading();
    delay(10);
  }
  outReading = sum / samples;
  return true;
}

float calculatePressure(float voltage) {
  if (voltage < VOLTAGE_MIN) return 0.0f;
  float normalized = voltage - VOLTAGE_MIN;
  float percent = normalized / VOLTAGE_RANGE;
  return percent * PRESSURE_MAX;
}

// ---------- Recording control ----------
void startRecording() {
  createFilename();
  if (!SD.begin(SD_CHIP_SELECT)) {
    Serial.println(mkJson(false,"START","SD not available"));
    return;
  }
  if (!openDataFile(filename.c_str())) {
    Serial.println(mkJson(false,"START","Failed to open data file"));
    return;
  }
  bufferPos = 0;
  if (thermocoupleWorking) {
    thermocouple.begin(MCP9601_I2C_ADDR);
  }
  addToBuffer("Time(us),Thrust(g),Temperature(C),Pressure(PSI)\r\n");
  for (int i=0;i<3;i++) { analogWrite(LED_PIN, LED_BRIGHTNESS_ERROR); delay(50); analogWrite(LED_PIN,0); delay(50); }
  analogWrite(LED_PIN, LED_BRIGHTNESS_ACTIVE);
  currentLedBrightness = LED_BRIGHTNESS_ACTIVE;
  isRecording = true;
  lastFlushTime = millis();
}

void stopRecording() {
  isRecording = false;
  flushBuffer();
  if (dataFile) dataFile.close();
  analogWrite(LED_PIN, LED_BRIGHTNESS_STANDBY);
  currentLedBrightness = LED_BRIGHTNESS_STANDBY;
}

void collectAndLogData() {
  unsigned long timestamp = micros();
  long rawReading = 0;
  float thrust = 0.0f;
  if (scaleHandler.available()) {
    if (waitForScaleAvailable(SCALE_WAIT_TIMEOUT_MS)) {
      rawReading = scaleHandler.getReading();
    }
  }
  thrust = calibrationFactor * ((float)rawReading - zeroOffset);

  float temperature = -999.99f;
  if (thermocoupleWorking) {
    thermocouple.begin(MCP9601_I2C_ADDR);
    temperature = thermocouple.readThermocouple();
  }

  int rawPressure = analogRead(PRESSURE_PIN);
  float voltage = (rawPressure * 3.3f) / 4095.0f;
  float pressure = calculatePressure(voltage);

  char row[128];
  int len = snprintf(row, sizeof(row), "%lu,%.2f,%.2f,%.2f\r\n", timestamp, thrust, temperature, pressure);
  if (len > 0) addToBuffer(row);
}

// ---------- Recovery & misc ----------
void tryReinitializePeripherals() {
  Wire.end();
  delay(10);
  Wire.begin();
  delay(5);

  for (int i=0;i<I2C_BEGIN_RETRIES;i++) {
    if (thermocouple.begin(MCP9601_I2C_ADDR)) {
      thermocouple.setADCresolution(MCP9600_ADCRESOLUTION_14);
      thermocouple.setAmbientResolution(RES_ZERO_POINT_25);
      thermocouple.setFilterCoefficient(0);
      thermocouple.setThermocoupleType(MCP9600_TYPE_K);
      thermocouple.enable(true);
      thermocoupleWorking = true;
      break;
    }
    delay(I2C_RETRY_DELAY_MS);
  }

  if (!SD.begin(SD_CHIP_SELECT)) {
    // no-op: try later
  }
}

void errorStateBlink(int errorCode) {
  unsigned long start = millis();
  const unsigned long TOTAL = 15000;
  while (millis() - start < TOTAL) {
    for (int i=0;i<errorCode;i++) {
      analogWrite(LED_PIN, LED_BRIGHTNESS_ERROR);
      delay(120);
      analogWrite(LED_PIN, 0);
      delay(120);
    }
    delay(300);
    tryReinitializePeripherals();
    if (thermocoupleWorking && SD.begin(SD_CHIP_SELECT)) return;
  }
}

// ---------- SD file listing helper ----------
String listFilesOnSD() {
  // Build JSON array of quoted filenames, comma-separated
  String out = "";
  File root = SD.open("/");
  if (!root) return out;
  File entry;
  bool first = true;
  entry = root.openNextFile();
  while (entry) {
    if (!entry.isDirectory()) {
      String name = String(entry.name());
      // add quoted name
      if (!first) out += ",";
      out += "\"";
      out += name;
      out += "\"";
      first = false;
    }
    entry.close();
    entry = root.openNextFile();
  }
  root.close();
  return out;
}
