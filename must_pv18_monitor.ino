/*
  ============================================================
   MUST PV-18 Inverter Monitor (ESP32)
   Reliable • WDT-safe • Modbus-safe • Buffer-safe
  ============================================================

  ✔ WiFi reconnect без зависань
  ✔ Захист від WDT у довгих операціях
  ✔ Modbus з паузами та ретраями
  ✔ HTTP POST з таймаутами
  ✔ FIFO-буфер (20 пакетів) при втраті мережі
  ✔ JSON формат під твій бекенд

  RS485:
   - RXD2      = GPIO18
   - TXD2      = GPIO19
   - RS485_DE  = GPIO23
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <ModbusMaster.h>
#include "esp_task_wdt.h"


// ============================================================
// 🔧 CONFIGURATION
// ============================================================

const char* WIFI_SSID = "Home36";
const char* WIFI_PASS = "83227423";

const char* API_URL     = "https://gv-traffic.com.ua/api/inverter_push.php";
const char* INVERTER_ID = "inverter";


// ============================================================
// 🔌 RS485 / MODBUS
// ============================================================

#define RXD2      18
#define TXD2      19
#define RS485_DE  23

ModbusMaster node;


// ============================================================
// ⏱ TIMING
// ============================================================

static const uint32_t SEND_INTERVAL_MS = 30000UL;  // 30 сек
static const uint32_t WIFI_DEAD_MS     = 30000UL;  // перезавантаження, якщо WiFi недоступний
static const uint32_t HTTP_TIMEOUT_MS  = 5000UL;   // HTTP timeout
static const uint32_t MODBUS_GAP_MS    = 30UL;     // пауза між Modbus запитами
static const uint8_t  MODBUS_RETRIES   = 2;        // кількість ретраїв

uint32_t lastSend   = 0;
uint32_t wifiLostAt = 0;


// ============================================================
// 🐕 WATCHDOG
// ============================================================

static const uint32_t WDT_TIMEOUT_MS = 30000; // 30 секунд


// ============================================================
// 📦 FIFO BUFFER
// ============================================================

#define BUFFER_SIZE 20

String buffer[BUFFER_SIZE];
int bufferStart = 0;
int bufferCount = 0;


// ============================================================
// 🔁 RS485 DIRECTION CONTROL
// ============================================================

void preTransmission()  { digitalWrite(RS485_DE, HIGH); }
void postTransmission() { digitalWrite(RS485_DE, LOW);  }


// ============================================================
// 🔧 UTILS
// ============================================================

static inline void wdtKick() {
  esp_task_wdt_reset();
  delay(0); // yield
}


// ============================================================
// 📦 BUFFER HANDLING
// ============================================================

void bufferPush(const String& data) {
  if (bufferCount < BUFFER_SIZE) {
    buffer[(bufferStart + bufferCount) % BUFFER_SIZE] = data;
    bufferCount++;
  } else {
    buffer[bufferStart] = data;
    bufferStart = (bufferStart + 1) % BUFFER_SIZE;
  }
  Serial.printf("📦 Buffered (%d/%d)\n", bufferCount, BUFFER_SIZE);
}

bool bufferPeek(String &out) {
  if (bufferCount == 0) return false;
  out = buffer[bufferStart];
  return true;
}

void bufferPop() {
  if (bufferCount == 0) return;
  bufferStart = (bufferStart + 1) % BUFFER_SIZE;
  bufferCount--;
}


// ============================================================
// 📶 WIFI
// ============================================================

void connectWiFiBlocking() {
  Serial.println("📶 Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(200);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    wdtKick();
    delay(250);
    Serial.print(".");
    if (millis() - start > WIFI_DEAD_MS) {
      Serial.println("\n💥 WiFi timeout → reboot");
      ESP.restart();
    }
  }

  Serial.println("\n✅ WiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void checkWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiLostAt = 0;
    return;
  }

  if (wifiLostAt == 0) {
    wifiLostAt = millis();
    Serial.println("⚠️ WiFi lost");
  }

  WiFi.reconnect();
  wdtKick();
  delay(200);

  if (millis() - wifiLostAt > WIFI_DEAD_MS) {
    Serial.println("💥 WiFi dead too long → reboot");
    ESP.restart();
  }
}


// ============================================================
// 📡 MODBUS SAFE READ
// ============================================================

bool modbusReadU16(uint16_t reg, uint16_t &out) {
  for (uint8_t i = 0; i <= MODBUS_RETRIES; i++) {
    wdtKick();
    uint8_t res = node.readHoldingRegisters(reg, 1);
    delay(MODBUS_GAP_MS);
    if (res == node.ku8MBSuccess) {
      out = node.getResponseBuffer(0);
      return true;
    }
    delay(50);
  }
  out = 0;
  return false;
}

bool modbusReadS16(uint16_t reg, int16_t &out) {
  for (uint8_t i = 0; i <= MODBUS_RETRIES; i++) {
    wdtKick();
    uint8_t res = node.readHoldingRegisters(reg, 1);
    delay(MODBUS_GAP_MS);
    if (res == node.ku8MBSuccess) {
      out = (int16_t)node.getResponseBuffer(0);
      return true;
    }
    delay(50);
  }
  out = 0;
  return false;
}


// ============================================================
// 🌐 HTTP SEND
// ============================================================

bool sendHTTP(const String& payload) {
  if (WiFi.status() != WL_CONNECTED) return false;

  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT_MS);

  wdtKick();
  if (!http.begin(API_URL)) return false;

  http.addHeader("Content-Type", "application/json");
  http.addHeader("User-Agent", "esp32-inverter");

  wdtKick();
  int code = http.POST(payload);

  wdtKick();
  http.end();

  Serial.printf("🌐 HTTP %d\n", code);
  return (code >= 200 && code < 300);
}


// ============================================================
// 🚀 SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1200);
  Serial.println("\n✅ MUST PV-18 ESP32 monitor booting...");

  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);

  Serial2.begin(19200, SERIAL_8N1, RXD2, TXD2);
  node.begin(0x04, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  connectWiFiBlocking();

  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT_MS,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  Serial.println("✅ System ready");
}


// ============================================================
// 🔄 LOOP
// ============================================================

void loop() {
  wdtKick();
  checkWiFi();

  // 1️⃣ Відправляємо з буфера по одному пакету
  if (bufferCount > 0 && WiFi.status() == WL_CONNECTED) {
    String packet;
    if (bufferPeek(packet)) {
      if (sendHTTP(packet)) {
        bufferPop();
        delay(150);
      } else {
        delay(200);
      }
    }
  }

  // 2️⃣ Відправка по інтервалу
  if (millis() - lastSend < SEND_INTERVAL_MS) {
    delay(25);
    return;
  }
  lastSend = millis();

  // ============================================================
  // 📊 READ MODBUS REGISTERS
  // ============================================================

  uint16_t u16 = 0;
  int16_t  s16 = 0;

  float pv_power        = modbusReadU16(15208, u16) ? (float)u16 : 0;
  float batt_voltage    = modbusReadU16(25205, u16) ? (float)u16 * 0.1f : 0;
  float batt_power      = modbusReadS16(25273, s16) ? (float)s16 : 0;
  float inverter_power  = modbusReadS16(25213, s16) ? (float)s16 : 0;
  float grid_power      = modbusReadS16(25214, s16) ? (float)s16 : 0;
  float load_power      = modbusReadU16(25215, u16) ? (float)u16 : 0;
  float grid_freq       = modbusReadU16(25226, u16) ? (float)u16 * 0.01f : 0;
  float temperature     = modbusReadU16(25233, u16) ? (float)u16 : 0;

  int rssi = WiFi.RSSI();

  // ============================================================
  // 🧾 JSON PAYLOAD
  // ============================================================

  String json;
  json.reserve(220);
  json  = "{";
  json += "\"inverter_id\":\"" + String(INVERTER_ID) + "\",";
  json += "\"pv_power\":" + String(pv_power, 1) + ",";
  json += "\"battery_voltage\":" + String(batt_voltage, 2) + ",";
  json += "\"battery_power\":" + String(batt_power, 1) + ",";
  json += "\"inverter_power\":" + String(inverter_power, 1) + ",";
  json += "\"load_power\":" + String(load_power, 1) + ",";
  json += "\"grid_power\":" + String(grid_power, 1) + ",";
  json += "\"temperature\":" + String(temperature, 1) + ",";
  json += "\"grid_frequency\":" + String(grid_freq, 2) + ",";
  json += "\"rssi\":" + String(rssi);
  json += "}";

  Serial.println("📤 NEW DATA:");
  Serial.println(json);

  // 3️⃣ Відправляємо або кладемо в буфер
  if (!sendHTTP(json)) {
    bufferPush(json);
  }

  delay(50);
}
