#include <Arduino.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// === Конфігурація ===
#define OPEN_SW_NUM         GPIO_NUM_18
#define BAT_LEV_NUM         GPIO_NUM_1
#define RGB_DIN_NUM         GPIO_NUM_20
#define RGB_VDD_NUM         GPIO_NUM_19
#define OPEN_SWITCH_POLARITY 1

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

// --- Надійна функція BLE + очікування підключення + deep sleep
void sendBLEandSleep(const char* msg, void (*waitFunc)(), uint32_t sleepSec=0) {
  Serial.printf("📡 Preparing to send: %s\n", msg);

  // --- Старт реклами після пробудження ---
  BLEDevice::startAdvertising();
  delay(500); // даємо час рекламі запуститися

  // --- Очікуємо підключення лише якщо клієнт ще не підключений ---
  if (!g_clientConnected) {
    unsigned long start = millis();
    const unsigned long WAIT_TIMEOUT = 8000; // 8 секунд
    while (!g_clientConnected && millis() - start < WAIT_TIMEOUT) {
      BLEDevice::startAdvertising();
      delay(200);
    }
  }

  if (g_clientConnected) {
    Serial.println("🔗 BLE client connected, sending notify...");
    for (int i = 0; i < 3; ++i) {
      pCharacteristic->setValue(msg);
      pCharacteristic->notify();
      Serial.printf("📡 BLE notify #%d sent: %s\n", i+1, msg);
      delay(700);
    }
    delay(500); // додатковий запас часу
  } else {
    Serial.println("⚠️ BLE client not connected within timeout, will still sleep");
    delay(200);
  }

  // --- Налаштування пробудження ---
  waitFunc();

  if (sleepSec > 0) {
    esp_sleep_enable_timer_wakeup((uint64_t)sleepSec * 1000000ULL);
    Serial.printf("💤 Will sleep for %u seconds\n", sleepSec);
  }

  Serial.println("💤 Going to deep sleep...");
  Serial.flush();
  delay(50);
  esp_deep_sleep_start();
}

// === Фази роботи ===
void phaseRED() {
  Serial.println("🔴 Phase RED");
  showColor(255, 0, 0);
  sendBLEandSleep("PHASE_RED", m_wait_closing);
}

void phaseBLUE() {
  Serial.println("🔵 Phase BLUE");
  showColor(0, 0, 255);
  sendBLEandSleep("PHASE_BLUE", m_wait_opening, 30);
}

void phaseGREEN() {
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

  rgb.begin(); rgb.clear(); rgb.show();

  rtc_gpio_init(OPEN_SW_NUM);
  rtc_gpio_set_direction(OPEN_SW_NUM, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en(OPEN_SW_NUM);
  rtc_gpio_pulldown_dis(OPEN_SW_NUM);

  // BLE
  BLEDevice::init("ESP");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // --- BLE сервіс ---
  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHAR_PHASE_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEDevice::startAdvertising();

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  Serial.printf("\nWakeup cause: %d\n", cause);

  bool closed = m_is_closed();
  Serial.printf("Magnet state: %s\n", closed ? "ON" : "OFF");

  // Логіка фаз
  if (cause == ESP_SLEEP_WAKEUP_TIMER) {
    phaseGREEN();
  } else {
    if (!closed) phaseRED();
    else phaseBLUE();
  }
}

void loop() {}
