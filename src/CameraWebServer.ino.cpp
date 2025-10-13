# 1 "C:\\Users\\Maksym\\AppData\\Local\\Temp\\tmppsfycxmq"
#include <Arduino.h>
# 1 "C:/ovul_cam_template/src/CameraWebServer.ino"
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_sleep.h"


#define OPEN_SW_PIN GPIO_NUM_18
#define LED_SIDE_PIN GPIO_NUM_21
#define RGB_DIN_PIN GPIO_NUM_20
#define RGB_VDD_PIN GPIO_NUM_19





#define CONNECT_WAIT_MS 30000UL
#define AFTER_CONNECT_MS 60000UL
#define ADV_CONNECT_TIMEOUT 10000UL
#define DEBOUNCE_MS 50UL

Adafruit_NeoPixel rgb(1, RGB_DIN_PIN, NEO_GRB + NEO_KHZ800);

BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
volatile bool deviceConnected = false;

unsigned long openSince = 0;
unsigned long connectStart = 0;
unsigned long purpleStart = 0;
unsigned long wakeStart = 0;

bool waitingForConnectWindow = false;
bool inPurpleTimer = false;
bool justWokeUp = false;


#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("📱 BLE connected");
  }
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("📴 BLE disconnected");
    pServer->startAdvertising();
  }
};
void setupBLE();
void setRGB(uint8_t r, uint8_t g, uint8_t b);
void sendNotify(const char* msg);
void goToSleepUntilMagnetChange(bool wakeOnHigh);
void indicateBlink(uint8_t r, uint8_t g, uint8_t b, int times, int onMs, int offMs);
void setup();
void loop();
#line 55 "C:/ovul_cam_template/src/CameraWebServer.ino"
void setupBLE() {
  BLEDevice::init("ESP32-DOOR");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("📡 BLE ready & advertising...");
}


void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  digitalWrite(RGB_VDD_PIN, HIGH);
  rgb.setPixelColor(0, rgb.Color(r, g, b));
  rgb.show();
}

void sendNotify(const char* msg) {
  if (deviceConnected && pCharacteristic) {
    pCharacteristic->setValue((uint8_t*)msg, strlen(msg));
    pCharacteristic->notify();
    Serial.print("▶ notify: "); Serial.println(msg);
  } else {
    Serial.println("⚠ cannot notify (not connected)");
  }
}

void goToSleepUntilMagnetChange(bool wakeOnHigh) {
  Serial.print("💤 Going to deep sleep... (wake on ");
  Serial.print(wakeOnHigh ? "HIGH" : "LOW");
  Serial.println(")");
  setRGB(0,0,0);
  delay(50);

  esp_sleep_enable_ext0_wakeup(OPEN_SW_PIN, wakeOnHigh ? 1 : 0);
  esp_deep_sleep_start();
}

void indicateBlink(uint8_t r, uint8_t g, uint8_t b, int times, int onMs, int offMs) {
  for (int i=0;i<times;i++){
    setRGB(r,g,b);
    delay(onMs);
    setRGB(0,0,0);
    delay(offMs);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);


  pinMode(OPEN_SW_PIN, INPUT_PULLUP);
  pinMode(LED_SIDE_PIN, OUTPUT);
  pinMode(RGB_VDD_PIN, OUTPUT);
  digitalWrite(RGB_VDD_PIN, HIGH);
  rgb.begin();

  setupBLE();


  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED) {
    justWokeUp = true;
    wakeStart = millis();
    Serial.println("🔅 Woke from sleep");
  } else {
    justWokeUp = false;
  }


  setRGB(0, 255, 0);
  Serial.println("🚀 Boot complete.");
}

void loop() {
  unsigned long now = millis();
  bool raw = digitalRead(OPEN_SW_PIN);
  bool magnetPresent = (raw == LOW);


  if (justWokeUp) {
    justWokeUp = false;
    Serial.println("🔔 After wake: advertising and waiting for central to connect...");
    setRGB(255, 255, 0);
    wakeStart = now;

    pServer->startAdvertising();

  }

  if (wakeStart > 0 && (now - wakeStart) < ADV_CONNECT_TIMEOUT) {

    if (deviceConnected) {
      sendNotify("123");
      indicateBlink(255, 255, 0, 2, 120, 80);
      wakeStart = 0;
      delay(100);


      goToSleepUntilMagnetChange(false);
    }
  } else if (wakeStart > 0 && (now - wakeStart) >= ADV_CONNECT_TIMEOUT) {

    Serial.println("⏳ No central connected after wake -> sleep until magnet returns");
    wakeStart = 0;
    goToSleepUntilMagnetChange(false);
  }


  static unsigned long lastDebounce = 0;
  static bool lastMagnetPresent = magnetPresent;
  if (magnetPresent != lastMagnetPresent) {
    lastDebounce = now;
    lastMagnetPresent = magnetPresent;
  }
  if ((now - lastDebounce) > DEBOUNCE_MS) {

    if (!magnetPresent) {

      if (openSince == 0) {
        openSince = now;
        Serial.println("🚪 OPEN (магніт знято)");
        setRGB(0, 0, 255);
        waitingForConnectWindow = true;
        connectStart = now;

      }
    } else {

      if (openSince != 0) {
        Serial.println("🔒 CLOSED (магніт повернувся)");
        openSince = 0;
        waitingForConnectWindow = false;
        connectStart = 0;
        inPurpleTimer = false;
        purpleStart = 0;
        if (deviceConnected) sendNotify("CLOSE");
        delay(50);


        goToSleepUntilMagnetChange(true);
      }
    }
  }


  if (waitingForConnectWindow && openSince > 0) {

    if (deviceConnected) {
      Serial.println("✅ Connected during 30s window -> blink green x3");
      indicateBlink(0,255,0,3,120,120);
      waitingForConnectWindow = false;

      inPurpleTimer = true;
      purpleStart = now;
      setRGB(128,0,128);

      sendNotify("CONNECTED_OK");
    } else if ((now - connectStart) >= CONNECT_WAIT_MS) {

      Serial.println("⏱ 30s passed without connect -> RED & deep sleep until magnet");
      setRGB(255,0,0);
      waitingForConnectWindow = false;
      connectStart = 0;
      delay(80);

      goToSleepUntilMagnetChange(false);
    }
  }


  if (inPurpleTimer) {
    if ((now - purpleStart) >= AFTER_CONNECT_MS) {
      Serial.println("⏲ Purple 1min finished -> go to deep sleep, will attempt notify after wake");
      inPurpleTimer = false;
      purpleStart = 0;
      delay(80);


      goToSleepUntilMagnetChange(false);
    }
  }


  static unsigned long lastStatusSend = 0;
  if (deviceConnected && (now - lastStatusSend) > 5000UL) {
    lastStatusSend = now;
    sendNotify("STATUS:OPEN");
  }

  delay(10);
}