#include <Arduino.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>

// === Конфігурація ===
#define OPEN_SW_NUM         GPIO_NUM_18
#define BAT_LEV_NUM         GPIO_NUM_1
#define RGB_DIN_NUM         GPIO_NUM_20
#define RGB_VDD_NUM         GPIO_NUM_19
#define OPEN_SWITCH_POLARITY 0   // 1 = замкнуто

// === BLE UUID ===
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHAR_PHASE_UUID     "12345678-1234-5678-1234-56789abcdef1"

// === RGB ===
Adafruit_NeoPixel rgb(1, RGB_DIN_NUM, NEO_GRB + NEO_KHZ800);

// === BLE ===
BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
volatile bool g_clientConnected = false;

// === Фази ===
enum Phase { PHASE_RED, PHASE_BLUE, PHASE_GREEN };
RTC_DATA_ATTR Phase currentPhase = PHASE_RED;  // 🧠 зберігається між deep sleep

// === Допоміжні функції ===
bool m_is_closed() { return digitalRead(OPEN_SW_NUM) == OPEN_SWITCH_POLARITY; }
void m_wait_opening() { esp_sleep_enable_ext0_wakeup(OPEN_SW_NUM, !OPEN_SWITCH_POLARITY); }
void m_wait_closing() { esp_sleep_enable_ext0_wakeup(OPEN_SW_NUM, OPEN_SWITCH_POLARITY); }

void showColor(uint8_t r, uint8_t g, uint8_t b) {
  rgb.setPixelColor(0, rgb.Color(r,g,b));
  rgb.show();
}

// === BLE callbacks ===
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    g_clientConnected = true;
    Serial.println("🔗 BLE client connected (callback)");
  }
  void onDisconnect(BLEServer* pServer) override {
    g_clientConnected = false;
    Serial.println("⚠️ BLE client disconnected (callback)");
  }
};

// --- BLE + очікування + deep sleep ---
void sendBLEandSleep(const char* msg, void (*waitFunc)(), uint32_t sleepSec = 0) {
  Serial.printf("📡 Preparing to send: %s\n", msg);

  BLEDevice::startAdvertising();
  Serial.println("📢 BLE advertising started");

  unsigned long start = millis();
  const unsigned long MAX_WAIT = 20000;  // 20 сек
  while (!g_clientConnected && millis() - start < MAX_WAIT) {
    delay(200);
  }

  if (g_clientConnected) {
  Serial.println("🔗 BLE client connected!");
  delay(800);
  for (int i = 0; i < 3; ++i) {
    if (!g_clientConnected) break;
    pCharacteristic->setValue(msg);
    pCharacteristic->notify();
    delay(500);
  }

  // 🕒 даємо телефону 3 сек після відправки, щоб підтвердити notify
  unsigned long extraWait = millis();
  while (g_clientConnected && millis() - extraWait < 3000) {
    delay(100);
  }
} else {
    Serial.println("⚠️ No BLE client connected within timeout.");
  }

  BLEDevice::stopAdvertising();
  Serial.println("🛑 BLE advertising stopped");

  waitFunc(); // wakeup по геркону

  if (sleepSec > 0) {
    esp_sleep_enable_timer_wakeup((uint64_t)sleepSec * 1000000ULL);
    Serial.printf("⏲️ Timer wakeup after %u seconds\n", sleepSec);
  }

  Serial.println("💤 Going to deep sleep...\n");
  Serial.flush();
  delay(300);
  esp_deep_sleep_start();
}

// === Фази ===
void phaseRED() {
  currentPhase = PHASE_RED;
  Serial.println("🔴 Phase RED (магніт відсутній)");
  showColor(255, 0, 0);
  sendBLEandSleep("PHASE_RED", m_wait_closing);  // чекаємо появи магніту → BLUE
}

void phaseBLUE() {
  currentPhase = PHASE_BLUE;
  Serial.println("🔵 Phase BLUE (магніт присутній)");
  showColor(0, 0, 255);
  sendBLEandSleep("PHASE_BLUE", m_wait_opening, 30); // після 30 сек → GREEN
}

void phaseGREEN() {
  currentPhase = PHASE_GREEN;
  Serial.println("🟢 Phase GREEN");
  showColor(0, 255, 0);
  sendBLEandSleep("PHASE_GREEN", [](){
    if (m_is_closed()) m_wait_opening();
    else m_wait_closing();
  });
}

// === SETUP ===
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(OPEN_SW_NUM, INPUT);
  pinMode(RGB_VDD_NUM, OUTPUT);
  digitalWrite(RGB_VDD_NUM, HIGH);

  rgb.begin(); 
  rgb.clear(); 
  rgb.show();

  rtc_gpio_init(OPEN_SW_NUM);
  rtc_gpio_set_direction(OPEN_SW_NUM, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en(OPEN_SW_NUM);
  rtc_gpio_pulldown_dis(OPEN_SW_NUM);

  // --- BLE ініціалізація ---
  BLEDevice::init("ESP");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHAR_PHASE_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setMinInterval(0x00A0);
  adv->setMaxInterval(0x00F0);
  adv->setScanResponse(true);

  BLEAdvertisementData advData;
  advData.setName("ESP");
  adv->setAdvertisementData(advData);
  adv->start();

  Serial.println("📡 Fast BLE advertising started");

  // --- Пробудження ---
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  bool closed = m_is_closed();

  Serial.printf("\nWakeup cause: %d\n", cause);
  Serial.printf("Magnet state: %s\n", closed ? "ON (є)" : "OFF (нема)");
  Serial.printf("Last phase: %d\n", currentPhase);

  // === Основна логіка фаз (зворотна) ===
  switch (currentPhase) {
    case PHASE_RED:
      if (closed) phaseBLUE();      // якщо магніт з’явився
      else phaseRED();              // залишаємося
      break;
    case PHASE_BLUE:
      if (cause == ESP_SLEEP_WAKEUP_TIMER) phaseGREEN();  // таймер → GREEN
      else if (!closed) phaseRED();                      // магніт зник → RED
      else phaseBLUE();
      break;
    case PHASE_GREEN:
      if (!closed) phaseRED();
      else phaseBLUE();
      break;
  }
}

void loop() {}
