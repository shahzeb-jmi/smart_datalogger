/* ESP32 Data Logger (FreeRTOS)
   - Reader task (Core 1): read sensors every cycle and enqueue JSON
   - Sender task (Core 0): manage WiFi, send to Flask or write to SD if offline
   - SD (SPI) for persistent buffering
   - Low-power deep sleep between cycles (10s active / 10s sleep)
   - Sensors: DHT11, LDR (ADC), MPU6050 (I2C)
   - Avoids dynamic heap fragmentation: uses StaticJsonDocument and static buffers
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>              // SPI SD
#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>
#include <ArduinoJson.h>

// ---------- USER CONFIG ---------- 
#define WIFI_SSID    "Excitel_esp32"
#define WIFI_PASS    "123456789"
#define SERVER_URL   "http://192.168.1.100:5000/data" // Flask endpoint

// Pins (change to your wiring)
#define DHT_PIN      15
#define DHT_TYPE     DHT11
#define LDR_PIN      36     
#define SDA_PIN      21
#define SCL_PIN      22

// SD (SPI) pins (adjust for your board)
#define SD_CS_PIN    5      // typical: 5
const char *BUFFER_FILE = "/buffer_log.txt";

#define ACTIVE_WINDOW_MS   10000UL // 10s active to try WiFi + send
#define SLEEP_MS           10000UL // 10s deep sleep

// Queue and JSON sizing
#define QUEUE_LENGTH  25
#define MAX_JSON_LEN  512
#define JSON_DOC_SIZE 512

// FreeRTOS stack sizes (tweak if needed)
#define READER_STACK  4096 //SENSORS BYTES 
#define SENDER_STACK  8192 //JASON BYTES 

// ---------- Globals ----------
QueueHandle_t jsonQueue = NULL;
SemaphoreHandle_t sdMutex = NULL;

DHT dht(DHT_PIN, DHT_TYPE);

// Minimal MPU6050 wrapper (lightweight)
class MPU6050Simple {
public:
  bool begin() {
    Wire.beginTransmission(0x68);
    if (Wire.endTransmission() != 0) return false;
    // Wake up
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0);
    if (Wire.endTransmission() != 0) return false;
    return true;
  }
  bool read() {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(0x68, (uint8_t)14, (uint8_t)1);
    if (Wire.available() < 14) return false;
    //SINCE ,INTERNAL ARCH OF SENSOR WITH REGISTER.
    int16_t ax = (Wire.read()<<8) | Wire.read();
    int16_t ay = (Wire.read()<<8) | Wire.read();
    int16_t az = (Wire.read()<<8) | Wire.read();
    Wire.read(); Wire.read(); // INBUIT TEMPERATURE SENSOR.
    Wire.read(); Wire.read(); Wire.read(); Wire.read(); Wire.read(); Wire.read(); // gyro raw discard
    accX = (float)ax / 16384.0f;
    accY = (float)ay / 16384.0f;
    accZ = (float)az / 16384.0f;
    return true;
  }
  float getAccX(){ return accX; }
  float getAccY(){ return accY; }
  float getAccZ(){ return accZ; }
private:
  float accX=0, accY=0, accZ=0;
} mpu;

// ---------- Function prototypes ----------
void readerTask(void* pvParameters); // Core 1
void senderTask(void* pvParameters); // Core 0
bool sdInit();
bool appendToSD(const char *line);
bool flushBufferToServer();
bool sendJsonToServer(const char *jsonLine);
String getDeviceId();
void enterDeepSleep();

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("== Data Logger Boot ==");

  // Init I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Init DHT
  dht.begin();

  // Init SD
  if (!sdInit()) {
    Serial.println("SD init failed - buffering disabled (will attempt still)");
  }

  // Create queue and mutex
  jsonQueue = xQueueCreate(QUEUE_LENGTH, MAX_JSON_LEN);//READ UPTO THOUSAND VALUES.
  if (!jsonQueue) {
    Serial.println("Failed to create queue - abort");
    while (1) { delay(1000); }
  }
  sdMutex = xSemaphoreCreateMutex();//FURTHR READ .
  if (!sdMutex) {
    Serial.println("Failed to create SD mutex - abort");
    while (1) { delay(1000); }
  }

  // Init MPU (best-effort)
  if (!mpu.begin()) {
    Serial.println("MPU6050 not responding (check wiring). Continuing.");
  } else {
    Serial.println("MPU6050 OK");
  }

  // Create tasks
  // WE CAN SWITCH PRIORITY OR SOME ADDITIONAL TASKS.
  BaseType_t r = xTaskCreatePinnedToCore(readerTask, "Reader", READER_STACK, NULL, 2, NULL, 1);
  BaseType_t s = xTaskCreatePinnedToCore(senderTask, "Sender", SENDER_STACK, NULL, 1, NULL, 0);
  if (r != pdPASS || s != pdPASS) {
    Serial.println("Task creation failed!");
    while (1) delay(1000);
  }
}

// loop unused (all in tasks)
void loop() {
  vTaskDelay(pdMS_TO_TICKS(2000));//BY INCREASING VALUE, YOU CAN INCREASE BATTERY LIFE BY THE COST OF PROCESSING.
}

// ---------- Reader task (Core 1) ----------
void readerTask(void* pvParameters) {
  (void) pvParameters;
  Serial.printf("Reader running on core %d\n", xPortGetCoreID());

  static StaticJsonDocument<JSON_DOC_SIZE> doc; // reused to avoid dynamic alloc (FRAGMENTATION)
  static char jsonBuf[MAX_JSON_LEN];

  while (true) {
    // Read sensors
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    int ldr = analogRead(LDR_PIN);

    float ax=0, ay=0, az=0;
    if (mpu.read()) {
      ax = mpu.getAccX();
      ay = mpu.getAccY();
      az = mpu.getAccZ();
    } // else zeros

    // Build JSON (static doc to avoid heap fragmentation)
    doc.clear();
    doc["device_id"] = getDeviceId();
    doc["ts_ms"] = (uint64_t)(esp_timer_get_time() / 1000ULL);

    JsonObject s = doc.createNestedObject("sensors");
    if (!isnan(temperature)) s["temperature_c"] = temperature;
    else s["temperature_c"] = nullptr;
    if (!isnan(humidity)) s["humidity_pct"] = humidity;
    else s["humidity_pct"] = nullptr;
    s["ldr_raw"] = ldr;
    JsonObject acc = s.createNestedObject("accel_g");//LOGICAL CONCEPT
    acc["x"] = ax;
    acc["y"] = ay;
    acc["z"] = az;

    size_t n = serializeJson(doc, jsonBuf, sizeof(jsonBuf));
    if (n == 0 || n >= sizeof(jsonBuf)) {
      Serial.println("JSON serialize error");
    } else {
      // Try enqueue (non-blocking)
      if (xQueueSend(jsonQueue, jsonBuf, 0) != pdTRUE) {
        Serial.println("Queue full -> append to SD immediately");
        if (!appendToSD(jsonBuf)) {
          Serial.println("SD append failed - dropping sample");
        }
      } else {
        // queued successfully
        // debug print minimal to reduce UART overhead
        Serial.println("Sample queued");
      }
    }

    // Sensor read cadence while active: we will read more frequently inside active window,
    // but overall hardware DHT11 should not be polled too rapidly; here we sleep 10s between reads
    // *BUT* we want to follow user request: read sensor reading every 10s from wake.
    vTaskDelay(pdMS_TO_TICKS(10000)); // 10s between readings
  }
}

// ---------- Sender task (Core 0) ----------
void senderTask(void* pvParameters) {
  (void) pvParameters;
  Serial.printf("Sender running on core %d\n", xPortGetCoreID());

  static char jsonLine[MAX_JSON_LEN];

  while (true) {
    Serial.println("=== Active cycle started ===");
    uint64_t activeStart = millis();

    // Start WiFi and attempt to connect within ACTIVE_WINDOW_MS
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    bool wifiConnected = false;
    uint64_t connStart = millis();
    while (millis() - connStart < ACTIVE_WINDOW_MS) {
      if (WiFi.status() == WL_CONNECTED) { wifiConnected = true; break; }
      vTaskDelay(pdMS_TO_TICKS(200));
    }

    if (wifiConnected) {
      Serial.print("WiFi connected, IP: ");
      Serial.println(WiFi.localIP());
      // First flush SD buffer
      if (!flushBufferToServer()) {
        Serial.println("Flush buffer partially failed or had unsent items.");
      }
      // Drain queue during remaining active window
      while (millis() - activeStart < ACTIVE_WINDOW_MS) {
        if (xQueueReceive(jsonQueue, jsonLine, pdMS_TO_TICKS(500)) == pdTRUE) {
          if (!sendJsonToServer(jsonLine)) {
            Serial.println("Live send failed -> append to SD");
            appendToSD(jsonLine);
          } else {
            Serial.println("Sent live sample");
          }
        } // else no item available, re-check active window
      }
    } else {
      Serial.println("WiFi NOT connected in active window -> persist queued samples to SD");
      // Drain queue immediately to SD
      while (xQueueReceive(jsonQueue, jsonLine, 0) == pdTRUE) {
        if (!appendToSD(jsonLine)) {
          Serial.println("SD append failed while buffering queued data");
        }
      }
    }

    // Cleanup WiFi
    if (WiFi.status() == WL_CONNECTED) {
      WiFi.disconnect(true, true);
      WiFi.mode(WIFI_OFF);
    }

    Serial.printf("Active done. Going to sleep for %u ms\n", (unsigned)SLEEP_MS);
    delay(50);
    enterDeepSleep();
    // device will restart on wake
  }
}

// ---------- SD helpers ----------
bool sdInit() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD.begin failed");
    return false;
  }
  // Ensure buffer file exists
  File f = SD.open(BUFFER_FILE, FILE_APPEND);
  if (!f) {
    Serial.println("Create buffer file failed");
    return false;
  }
  f.close();
  Serial.println("SD initialized");
  return true;
}

bool appendToSD(const char *line) {
  if (!sdMutex) return false;
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) != pdTRUE) return false;
  bool ok = false;
  File f = SD.open(BUFFER_FILE, FILE_APPEND);
  if (!f) {
    Serial.println("appendToSD: open failed");
    ok = false;
  } else {
    f.println(line);
    f.flush();
    f.close();
    ok = true;
  }
  xSemaphoreGive(sdMutex);
  return ok;
}

// Read buffer file line-by-line and post each. Unsent lines are rewritten back into file.
// Best-effort but safe (uses temp file replacement)
//WORK ON TEMPERATURE ACCURACY,USING A NEW TEMP CSV FILE .
bool flushBufferToServer() {
  if (!SD.exists(BUFFER_FILE)) return true;
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000)) != pdTRUE) return false;

  File f = SD.open(BUFFER_FILE, FILE_READ);
  if (!f) { xSemaphoreGive(sdMutex); return false; }

  const char *tmp = "/buffer_tmp.txt";
  File temp = SD.open(tmp, FILE_WRITE);
  if (!temp) { f.close(); xSemaphoreGive(sdMutex); return false; }

  char line[MAX_JSON_LEN];
  bool overallSuccess = true;

  while (f.available()) {
    size_t len = f.readBytesUntil('\n', line, sizeof(line)-1);
    line[len] = 0;
    if (len == 0) continue;
    if (!sendJsonToServer(line)) {
      temp.println(line); // keep for later
      overallSuccess = false;
    }
  }
  f.close();
  temp.close();

  // Replace original file with tmp (unsent lines). If tmp empty, recreate empty buffer file.
  if (SD.exists(tmp)) {
    File t = SD.open(tmp, FILE_READ);
    if (t && t.size() == 0) {
      t.close();
      SD.remove(tmp);
      SD.remove(BUFFER_FILE);
      File e = SD.open(BUFFER_FILE, FILE_WRITE);
      if (e) e.close();
    } else {
      if (t) t.close();
      SD.remove(BUFFER_FILE);
      SD.rename(tmp, BUFFER_FILE);
    }
  }
//END
  xSemaphoreGive(sdMutex);
  return overallSuccess;
}

// ---------- Networking ----------
bool sendJsonToServer(const char *jsonLine) {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST((uint8_t*)jsonLine, strlen(jsonLine));
  bool ok = false;
  if (code > 0) {
    if (code >= 200 && code < 300) ok = true;
    // optional: read response if needed
    // String resp = http.getString();
  } else {
    Serial.printf("HTTP POST failed: %s\n", http.errorToString(code).c_str());
  }
  http.end();
  return ok;
}

// ---------- Utilities ----------
String getDeviceId() {
  uint64_t mac = ESP.getEfuseMac();
  char buf[25];
  sprintf(buf, "%04X%08X", (uint16_t)(mac>>32), (uint32_t)mac);
  return String(buf);
}

void enterDeepSleep() {
  // disable peripherals gently
  delay(50);
  esp_sleep_enable_timer_wakeup(SLEEP_MS * 1000ULL);
  esp_deep_sleep_start();
}
