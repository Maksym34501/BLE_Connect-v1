#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_sleep.h"

#define OPEN_SW_NUM 18
#define LED_TOP_NUM 12
#define LED_SIDE_NUM 21
#define RGB_DIN_NUM 20
#define RGB_VDD_NUM 19

Adafruit_NeoPixel pixel(1, RGB_DIN_NUM, NEO_GRB + NEO_KHZ800);

BLEServer* pServer = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// === Колбек сервера BLE ===
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("✅ Телефон підключився!");
  }

  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("❌ Телефон відключився!");
  }
};

// === Ініціалізація BLE ===
void initBLE() {
  Serial.println("🔌 Ініціалізація BLE...");
  BLEDevice::init("MagnetDevice");

  // 🔓 Вимикаємо обов’язкове шифрування та pairing
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_NO_MITM);
  BLESecurity *pSecurity = new BLESecurity();
pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND); // або ESP_LE_AUTH_BOND
pSecurity->setCapability(ESP_IO_CAP_NONE);
pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(BLEUUID((uint16_t)0x180F));
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
      BLEUUID((uint16_t)0x2A19),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("Ready");
  pService->start();

  pServer->getAdvertising()->start();
  Serial.println("📡 BLE запущено, чекаємо підключення...");
}

// === Очікування підключення ===
bool waitForConnection() {
  pixel.setPixelColor(0, pixel.Color(255, 200, 0)); // 🟡 Очікування
  pixel.show();

  unsigned long start = millis();
  while (!deviceConnected && millis() - start < 30000) {
    delay(200);
  }

  if (deviceConnected) {
    pixel.setPixelColor(0, pixel.Color(0, 255, 0)); // 🟩 Зелений — підключено
    pixel.show();
    delay(1000);
    pixel.clear();
    pixel.show();
    return true;
  } else {
    pixel.setPixelColor(0, pixel.Color(255, 0, 0)); // 🔴 Не підключились
    pixel.show();
    delay(2000);
    pixel.clear();
    pixel.show();
    return false;
  }
}

void goToDeepSleepWithTimer(uint64_t seconds) {
  Serial.printf("💤 Йдемо в сон на %llu секунд...\n", seconds);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_timer_wakeup(seconds * 1000000ULL);
  esp_deep_sleep_start();
}

void goToDeepSleepUntilMagnetState(int targetState) {
  Serial.printf("💤 Сон до стану магніта = %d...\n", targetState);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)OPEN_SW_NUM, targetState);
  esp_deep_sleep_start();
}

void activePhase(uint8_t r, uint8_t g, uint8_t b, const char* msg) {
  Serial.println(msg);
  digitalWrite(LED_TOP_NUM, HIGH);
  digitalWrite(LED_SIDE_NUM, HIGH);
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
  delay(5000);
  digitalWrite(LED_TOP_NUM, LOW);
  digitalWrite(LED_SIDE_NUM, LOW);
  pixel.clear();
  pixel.show();
}

void setup() {
  Serial.begin(115200);
  pinMode(OPEN_SW_NUM, INPUT_PULLUP);
  pinMode(LED_TOP_NUM, OUTPUT);
  pinMode(LED_SIDE_NUM, OUTPUT);
  pinMode(RGB_VDD_NUM, OUTPUT);
  digitalWrite(RGB_VDD_NUM, HIGH);
  pixel.begin();
  pixel.clear();
  pixel.show();

  // 🔧 1. Запускаємо BLE
  initBLE();

  // 🔧 2. Очікуємо підключення телефона
  if (!waitForConnection()) {
    Serial.println("❌ Нема підключення — спимо 30 сек");
    goToDeepSleepWithTimer(30);
  }

  // 🔧 3. Після підключення — сценарії з магнітом
  Serial.println("▶ Починаємо сценарії з магнітом...");
  bool magnetPresent = (digitalRead(OPEN_SW_NUM) == LOW);
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
    if (magnetPresent) {
      Serial.println("Магніт присутній → сон до прибирання");
      goToDeepSleepUntilMagnetState(1);
    } else {
      activePhase(255, 0, 0, "🚫 Початково без магніта → червона фаза");
      goToDeepSleepUntilMagnetState(0);
    }
  }
  else if (cause == ESP_SLEEP_WAKEUP_EXT0) {
    if (magnetPresent) {
      activePhase(0, 0, 255, "🔵 Магніт повернувся → синя фаза");
      goToDeepSleepWithTimer(30);
    } else {
      activePhase(255, 0, 0, "🚫 Магніт прибрано → червона фаза");
      goToDeepSleepUntilMagnetState(0);
    }
  }
  else if (cause == ESP_SLEEP_WAKEUP_TIMER) {
    activePhase(0, 255, 0, "🟩 30 сек минуло → зелена фаза");
    goToDeepSleepUntilMagnetState(1);
  }
}

void loop() {
  if (!oldDeviceConnected && deviceConnected) {
    oldDeviceConnected = true;
  }
  if (oldDeviceConnected && !deviceConnected) {
    oldDeviceConnected = false;
    Serial.println("📴 Пристрій втратив зв’язок — перезапуск BLE реклами");
    delay(500);
    pServer->getAdvertising()->start();
  }
}