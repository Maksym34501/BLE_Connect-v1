#include <Arduino.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"

#include <Adafruit_NeoPixel.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include "camera_pins.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <LiteLED.h>
#include "rgb_led.h"

// Configuration
#define USE_REAL_FLASH true 
#define OPEN_SW_NUM         GPIO_NUM_18
#define BAT_LEV_NUM         GPIO_NUM_1
#define RGB_DIN_NUM         GPIO_NUM_20
#define RGB_VDD_NUM         GPIO_NUM_19
#define OPEN_SWITCH_POLARITY 0
#ifndef CAM_PWR_NUM
#define CAM_PWR_NUM  4
#endif
#ifndef RESET_GPIO_NUM
#define RESET_GPIO_NUM 15
#endif

#define SERVICE_UUID              "6f1e0000-1234-4bcd-8000-abcdef123450"
#define CHAR_PHASE_UUID           "6f1e0001-1234-4bcd-8000-abcdef123450"
#define IMAGE_SERVICE_UUID        "6f1e1000-1234-4bcd-8000-abcdef123450"
#define IMAGE_CHARACTERISTIC_UUID "6f1e1001-1234-4bcd-8000-abcdef123450"
#define CMD_CHARACTERISTIC_UUID   "6f1e1002-1234-4bcd-8000-abcdef123450"
// --- ДОДАТКОВІ ГЛОБАЛЬНІ ПРАПОРЦІ (вгорі файлу) ---
volatile bool g_clientConnected = false;
bool deviceConnected = false;
// зберігаємо обидва стани для діагностики
volatile bool cccd_notify_enabled = false;     // bit 0 (0x0001)
volatile bool cccd_indicate_enabled = false;   // bit 1 (0x0002)
// Зручний хуткий флаг, який означає "клієнт підписався (взагалі)"
volatile bool clientSubscribed = false;
Adafruit_NeoPixel rgb(1, RGB_DIN_NUM, NEO_GRB + NEO_KHZ800);
LiteLED myLED(LED_STRIP_SK6812, true);

BLEServer* pServer = nullptr;
BLECharacteristic* pPhaseCharacteristic = nullptr;
BLECharacteristic* pImageCharacteristic = nullptr;
BLECharacteristic* pCmdCharacteristic = nullptr;

 
 
volatile bool notificationsEnabled = false;
volatile bool sendRequested = false;

enum Phase { PHASE_RED = 0, PHASE_BLUE = 1, PHASE_GREEN = 2 };
RTC_DATA_ATTR Phase currentPhase = PHASE_RED;

void phaseRED();
void phaseBLUE();
void phaseGREEN();
void showColor(uint8_t r, uint8_t g, uint8_t b);
bool cameraInit();
bool waitForNotificationsEnabled(uint32_t timeoutMs);
void sendBufferViaBLE(uint8_t* data, size_t len);
void advertiseAndWaitForClientThenNotify(const char* msg, uint32_t maxWaitMs = 20000, uint32_t postNotifyWaitMs = 3000, int repeats = 3);

bool m_is_closed() { return digitalRead(OPEN_SW_NUM) == OPEN_SWITCH_POLARITY; }
void m_wait_opening() { esp_sleep_enable_ext0_wakeup(OPEN_SW_NUM, !OPEN_SWITCH_POLARITY); }
void m_wait_closing() { esp_sleep_enable_ext0_wakeup(OPEN_SW_NUM, OPEN_SWITCH_POLARITY); }

void showColor(uint8_t r, uint8_t g, uint8_t b) {
rgb.setPixelColor(0, rgb.Color(r,g,b));
rgb.show();
delay(10);
rgb_t pix; pix.red = r; pix.green = g; pix.blue = b;
myLED.setPixel(0, pix, 1);
}
void enableFlashForCapture() {
#if USE_REAL_FLASH
    digitalWrite(LED_TOP_NUM, HIGH);   // вмикаємо реальний білий LED
    digitalWrite(LED_SIDE_NUM, LOW);   // якщо потрібно — можна теж HIGH
#else
    rgb.setPixelColor(0, rgb.Color(255,255,255));
    rgb.show();
    rgb_t pix; pix.red = 255; pix.green = 255; pix.blue = 255;
    myLED.setPixel(0, pix, 1);
#endif
}

void restoreFlashAfterCapture() {
#if USE_REAL_FLASH
    digitalWrite(LED_TOP_NUM, LOW);    // вимикаємо підсвітку
    digitalWrite(LED_SIDE_NUM, LOW);
#else
    rgb.setPixelColor(0, rgb.Color(prev_r, prev_g, prev_b));
    rgb.show();
    rgb_t pix; pix.red = prev_r; pix.green = prev_g; pix.blue = prev_b;
    myLED.setPixel(0, pix, 1);
#endif
}
// BLE callbacks
class PhaseServerCallbacks: public BLEServerCallbacks {
void onConnect(BLEServer* pServer) override {
g_clientConnected = true;
deviceConnected = true;
Serial.println("🔗 BLE client connected");
}
void onDisconnect(BLEServer* pServer) override {
g_clientConnected = false;
deviceConnected = false;
notificationsEnabled = false;
Serial.println("⚠️ BLE client disconnected");
pServer->getAdvertising()->start();
}
};
// --- ОНОВЛЕНА MyDescriptorCallbacks ---
class MyDescriptorCallbacks : public BLEDescriptorCallbacks {
  void onWrite(BLEDescriptor* desc) override {
    uint8_t* buf = desc->getValue();
    size_t len = desc->getLength();
    if (len >= 2) {
      uint16_t val = ((uint16_t)buf[1] << 8) | buf[0];
      // Розпізнаємо обидва біти
      cccd_notify_enabled = (val & 0x0001) != 0;
      cccd_indicate_enabled = (val & 0x0002) != 0;

      // Вважай підписку валідною, якщо встановлено хоча б один з бітів.
      clientSubscribed = cccd_notify_enabled || cccd_indicate_enabled;

      Serial.printf("CCCD written: 0x%04X, notify=%d, indicate=%d, subscribed=%d\n",
                    val, cccd_notify_enabled, cccd_indicate_enabled, clientSubscribed);
    } else {
      clientSubscribed = false;
      cccd_notify_enabled = false;
      cccd_indicate_enabled = false;
      Serial.println("CCCD written: too short -> treat as unsubscribed");
    }
  }
};

bool waitForNotificationsEnabled(uint32_t timeoutMs) {
  uint32_t waited = 0;
  while (!clientSubscribed && waited < timeoutMs) {
    if (!deviceConnected) return false;
    vTaskDelay(pdMS_TO_TICKS(20));
    waited += 20;
  }
  return clientSubscribed;
}

// --- передача буфера зображення без збереження ---
 void sendBufferViaBLE(uint8_t* data, size_t len) {
    if (!deviceConnected || !clientSubscribed) {
        Serial.println("⚠️ BLE client not ready for image (not connected or not subscribed)");
        return;
    }

    const size_t chunkSize = 500; // або 180..500 в залежності від MTU/стабільності
    const char* startMsg = "START";
    const char* endMsg   = "END";

    // Надсилаємо START (ми використовуємо indicate() завжди)
    pImageCharacteristic->setValue((uint8_t*)startMsg, strlen(startMsg));
    pImageCharacteristic->indicate();
    delay(200); // даємо клієнту час обробити START

    size_t sent = 0;
    while (sent < len) {
        size_t toSend = min(chunkSize, len - sent);
        pImageCharacteristic->setValue(data + sent, toSend);
        pImageCharacteristic->indicate(); // НАМЕРТВО: indicate()
        sent += toSend;

        Serial.printf("Sent chunk %u/%u\n", (unsigned)sent, (unsigned)len);

        // невелика пауза для стабільності (можна зменшити, якщо тест показує стабільність)
        delay(5);
    }

    // Надсилаємо END
    pImageCharacteristic->setValue((uint8_t*)endMsg, strlen(endMsg));
    pImageCharacteristic->indicate();
    delay(200);

    Serial.println("✅ Image buffer sent successfully (all via indicate())");
}


void advertiseAndWaitForClientThenNotify(const char* msg, uint32_t maxWaitMs, uint32_t postNotifyWaitMs, int repeats) {
BLEAdvertising* adv = pServer->getAdvertising();
adv->start();
unsigned long start = millis();
while (!deviceConnected && millis() - start < maxWaitMs) delay(200);

if (deviceConnected) {
for (int i = 0; i < repeats; ++i) {
if (!deviceConnected) break;
pPhaseCharacteristic->setValue(msg);
pPhaseCharacteristic->indicate();
delay(500);
}
delay(postNotifyWaitMs);
} else {
Serial.println("⚠️ No BLE client connected within timeout");
}
adv->stop();
}

// --- ФАЗИ ---
void sendBLEandSleep(const char* msg, void (*waitFunc)(), uint32_t sleepSec = 0) {
BLEDevice::startAdvertising();
unsigned long start = millis();
while (!g_clientConnected && millis() - start < 20000) delay(200);

if (g_clientConnected) {
for (int i = 0; i < 3; ++i) {
pPhaseCharacteristic->setValue(msg);
pPhaseCharacteristic->indicate();
delay(500);
}
}

BLEDevice::stopAdvertising();
waitFunc();
if (sleepSec > 0)
esp_sleep_enable_timer_wakeup((uint64_t)sleepSec * 1000000ULL);
esp_deep_sleep_start();
}

void phaseRED() {
currentPhase = PHASE_RED;
Serial.println("🔴 Phase RED (магніт відсутній)");
showColor(255, 0, 0);
// Вилучено відправку фото
sendBLEandSleep("PHASE_RED", m_wait_closing);
}

void phaseBLUE() {
currentPhase = PHASE_BLUE;
Serial.println("🔵 Phase BLUE (магніт присутній)");
showColor(0, 0, 255);
sendBLEandSleep("PHASE_BLUE", m_wait_opening, 30);
}

void phaseGREEN() {
    currentPhase = PHASE_GREEN;
    Serial.println("🟢 Phase GREEN");
    showColor(0, 255, 0);

    pinMode(CAM_PWR_NUM, OUTPUT);
    digitalWrite(CAM_PWR_NUM, HIGH);
    delay(200);

    if (!cameraInit()) {
        Serial.println("❌ Camera init failed");
        digitalWrite(CAM_PWR_NUM, LOW);
        return;
    }

    // прогрів камери
    for (int i = 0; i < 2; i++) {
        camera_fb_t* fb_temp = esp_camera_fb_get();
        if (fb_temp) esp_camera_fb_return(fb_temp);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // --- ПІДСВІТКА: вмикаємо біле для зйомки прямо перед захопленням ---
    enableFlashForCapture();

    camera_fb_t* fb = esp_camera_fb_get();

    // після захоплення відновлюємо підсвітку (перед обробкою буфера)
    restoreFlashAfterCapture();

    if (!fb) {
        Serial.println("❌ Failed to capture photo");
        digitalWrite(CAM_PWR_NUM, LOW);
        return;
    }

    Serial.printf("Captured frame len=%u\n", (unsigned)fb->len);

    // --- чекати підключення клієнта ---
    BLEAdvertising* adv = pServer->getAdvertising();
    adv->start();
    unsigned long start = millis();
    while (!deviceConnected && millis() - start < 20000) delay(50);

    if (!deviceConnected) {
        Serial.println("⚠️ No BLE client connected within timeout");
        esp_camera_fb_return(fb);
        digitalWrite(CAM_PWR_NUM, LOW);
        return;
    }

    // --- чекати індикації ---
    if (!waitForNotificationsEnabled(20000)) {
        Serial.println("⚠️ BLE client not enabled notifications");
        esp_camera_fb_return(fb);
        digitalWrite(CAM_PWR_NUM, LOW);
        return;
    }

    delay(200); // додатковий buffer time

    // --- повідомлення READY_FOR_IMAGE ---
    for (int i = 0; i < 3; i++) {
        pPhaseCharacteristic->setValue("READY_FOR_IMAGE");
        pPhaseCharacteristic->indicate();
        delay(500);
    }

    // --- надсилання буфера ---
    sendBufferViaBLE(fb->buf, fb->len);

    esp_camera_fb_return(fb);
    digitalWrite(CAM_PWR_NUM, LOW);

 
    if (m_is_closed()) m_wait_opening();
    else m_wait_closing();

 

    esp_deep_sleep_start();
}


// --- Ініціалізація камери ---
bool cameraInit () {
camera_config_t config;
config.ledc_channel = LEDC_CHANNEL_0;
config.ledc_timer = LEDC_TIMER_0;
config.pin_d0 = Y2_GPIO_NUM;
config.pin_d1 = Y3_GPIO_NUM;
config.pin_d2 = Y4_GPIO_NUM;
config.pin_d3 = Y5_GPIO_NUM;
config.pin_d4 = Y6_GPIO_NUM;
config.pin_d5 = Y7_GPIO_NUM;
config.pin_d6 = Y8_GPIO_NUM;
config.pin_d7 = Y9_GPIO_NUM;
config.pin_xclk = XCLK_GPIO_NUM;
config.pin_pclk = PCLK_GPIO_NUM;
config.pin_vsync = VSYNC_GPIO_NUM;
config.pin_href = HREF_GPIO_NUM;
config.pin_sccb_sda = SIOD_GPIO_NUM;
config.pin_sccb_scl = SIOC_GPIO_NUM;
config.pin_pwdn = PWDN_GPIO_NUM;
config.pin_reset = RESET_GPIO_NUM;
config.xclk_freq_hz = 10000000;
config.pixel_format = PIXFORMAT_JPEG;
config.frame_size = FRAMESIZE_SXGA;
config.jpeg_quality = 8;
config.fb_count = 2;
config.fb_location = CAMERA_FB_IN_PSRAM;
config.grab_mode = CAMERA_GRAB_LATEST;

esp_err_t err = esp_camera_init(&config);
if (err != ESP_OK) {
Serial.printf("Camera init failed with error 0x%x\n", err);
return false;
}
sensor_t * s = esp_camera_sensor_get();
if (!s) return false;
if (s->id.PID == OV5640_PID) Serial.println("Camera: OV5640 detected");
return true;
}

// --- SETUP ---
void setup() {
pinMode(LED_TOP_NUM, OUTPUT);
pinMode(LED_SIDE_NUM, OUTPUT);
digitalWrite(LED_TOP_NUM, LOW);
digitalWrite(LED_SIDE_NUM, LOW);
WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
Serial.begin(115200);
delay(500);

pinMode(OPEN_SW_NUM, INPUT);
pinMode(RGB_VDD_NUM, OUTPUT);
digitalWrite(RGB_VDD_NUM, HIGH);

rgb.begin();
rgb.clear();
rgb.show();
myLED.begin(RGB_DIN_NUM, 1);
myLED.brightness(255);

rtc_gpio_init(OPEN_SW_NUM);
rtc_gpio_set_direction(OPEN_SW_NUM, RTC_GPIO_MODE_INPUT_ONLY);
rtc_gpio_pullup_en(OPEN_SW_NUM);

BLEDevice::init("ESP_Camera_Phase");
pServer = BLEDevice::createServer();
pServer->setCallbacks(new PhaseServerCallbacks());

BLEService* phaseService = pServer->createService(SERVICE_UUID);
pPhaseCharacteristic = phaseService->createCharacteristic(
CHAR_PHASE_UUID,
BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
pPhaseCharacteristic->addDescriptor(new BLE2902());
phaseService->start();

BLEService* imageService = pServer->createService(IMAGE_SERVICE_UUID);
pImageCharacteristic = imageService->createCharacteristic(
IMAGE_CHARACTERISTIC_UUID,
BLECharacteristic::PROPERTY_INDICATE);
BLE2902* img2902 = new BLE2902();
img2902->setCallbacks(new MyDescriptorCallbacks());
pImageCharacteristic->addDescriptor(img2902);
imageService->start();

BLEAdvertising* adv = pServer->getAdvertising();
adv->addServiceUUID(SERVICE_UUID);
adv->addServiceUUID(IMAGE_SERVICE_UUID);
adv->start();

BLEDevice::setMTU(517);

bool closed = m_is_closed();
if (currentPhase == PHASE_GREEN) {
if (!closed) phaseRED();
else phaseBLUE();
} else if (currentPhase == PHASE_RED) {
if (closed) phaseBLUE();
else phaseRED();
} else {
if (closed) phaseGREEN();
else phaseRED();
}
}

void loop() {}
