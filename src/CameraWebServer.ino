// Повний скетч — з виправленнями для надійної передачі зображення по BLE (notify + 180B chunks)
#include <Arduino.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"

#include <Adafruit_NeoPixel.h>

// Camera + SPIFFS
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include "camera_pins.h"
#include "FS.h"
#include "SPIFFS.h"

// BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Optional LED lib
#include <LiteLED.h>
#include "rgb_led.h"

// Configuration
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

// === BLE UUIDs (з твоїми UUID) ===
#define SERVICE_UUID              "6f1e0000-1234-4bcd-8000-abcdef123450"  // Phase/info service
#define CHAR_PHASE_UUID           "6f1e0001-1234-4bcd-8000-abcdef123450"  // Phase info characteristic

#define IMAGE_SERVICE_UUID        "6f1e1000-1234-4bcd-8000-abcdef123450"  // Image transfer service
#define IMAGE_CHARACTERISTIC_UUID "6f1e1001-1234-4bcd-8000-abcdef123450"  // Image data characteristic
#define CMD_CHARACTERISTIC_UUID   "6f1e1002-1234-4bcd-8000-abcdef123450"  // Command/control characteristic

// RGB
Adafruit_NeoPixel rgb(1, RGB_DIN_NUM, NEO_GRB + NEO_KHZ800);
LiteLED myLED( LED_STRIP_SK6812, true );

// Globals
BLEServer* pServer = nullptr;
BLECharacteristic* pPhaseCharacteristic = nullptr;

BLECharacteristic* pImageCharacteristic = nullptr;
BLEDescriptor* pImageCCCD = nullptr;
BLECharacteristic* pCmdCharacteristic = nullptr;

volatile bool g_clientConnected = false;
bool deviceConnected = false;
volatile bool notificationsEnabled = false;
volatile bool sendRequested = false;

// Phases and persistence
enum Phase { PHASE_RED = 0, PHASE_BLUE = 1, PHASE_GREEN = 2 };
RTC_DATA_ATTR Phase currentPhase = PHASE_RED;
RTC_DATA_ATTR bool hasSavedPhoto = false;

const char * SAVED_PHOTO_PATH = "/last.jpg";
const char * TMP_PHOTO_PATH = "/last.jpg.tmp";

// Forward
void phaseRED();
void phaseBLUE();
void phaseGREEN();
void showColor(uint8_t r, uint8_t g, uint8_t b);
bool cameraInit();
bool saveFrameToSPIFFS_safe(camera_fb_t* fb, const char* finalPath);
void sendFileViaBLE_SPIFFS(const char* path);
bool waitForNotificationsEnabled(uint32_t timeoutMs);
void advertiseAndSendIfPhoto(void (*waitFunc)(), uint32_t sleepSec = 0);

// Magnet helpers
bool m_is_closed() { return digitalRead(OPEN_SW_NUM) == OPEN_SWITCH_POLARITY; }
void m_wait_opening() { esp_sleep_enable_ext0_wakeup(OPEN_SW_NUM, !OPEN_SWITCH_POLARITY); }
void m_wait_closing() { esp_sleep_enable_ext0_wakeup(OPEN_SW_NUM, OPEN_SWITCH_POLARITY); }

// LEDs
void showColor(uint8_t r, uint8_t g, uint8_t b) {
  rgb.setPixelColor(0, rgb.Color(r,g,b));
  rgb.show();
  delay(10);
  rgb_t pix; pix.red = r; pix.green = g; pix.blue = b;
  myLED.setPixel(0, pix, 1);
}

// BLE Callbacks
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

class MyDescriptorCallbacks : public BLEDescriptorCallbacks {
  void onWrite(BLEDescriptor* desc) override {
    uint8_t* buf = desc->getValue();
    size_t len = desc->getLength();
    if(len >= 2) {
        uint16_t val = ((uint16_t)buf[1] << 8) | buf[0];
        // accept notify (0x0001) or indicate (0x0002)
        notificationsEnabled = (val & 0x0002) != 0; // 0x0002 = indicate
        Serial.printf("CCCD written: 0x%04X, indicationsEnabled=%d, millis=%lu\n", val, notificationsEnabled, millis());
    }
  }
};

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    std::string val = pCharacteristic->getValue();
    if (val.length() > 0) {
      sendRequested = true;
      Serial.println("BLE: Capture/send command received");
    }
  }
};

// wait for client to write CCCD (we set notificationsEnabled in descriptor callback)
bool waitForNotificationsEnabled(uint32_t timeoutMs) {
  const uint32_t step = 20;
  uint32_t waited = 0;
  while (!notificationsEnabled && waited < timeoutMs) {
    if (!deviceConnected) return false;
    vTaskDelay(pdMS_TO_TICKS(step));
    waited += step;
  }
  return notificationsEnabled;
}

// --- FIXED: use notify() (not indicate) and smaller chunk size
void sendFileViaBLE_SPIFFS(const char* path) {
  if (!deviceConnected) {
    Serial.println("Client not connected");
    return;
  }
  if (!notificationsEnabled) {
    Serial.println("Client has not enabled indications for image characteristic");
    // return; // можна продовжити, але краще чекати enable через waitForNotificationsEnabled()
  }

  if (!SPIFFS.exists(path)) {
    Serial.println("File not found");
    return;
  }

  File f = SPIFFS.open(path, FILE_READ);
  size_t total = f.size();
  Serial.printf("Total file size: %u bytes\n", (unsigned)total);

  // --- send START ---
  const char* startMsg = "START";
  pImageCharacteristic->setValue((uint8_t*)startMsg, strlen(startMsg));
  pImageCharacteristic->indicate();  // <-- reliable send
  delay(150);

  const size_t chunkSize = 500; // BLE safe chunk size
  uint8_t buf[chunkSize];
  size_t sent = 0;

  while (sent < total) {
    size_t toRead = (total - sent > chunkSize) ? chunkSize : (total - sent);
    size_t actuallyRead = f.read(buf, toRead);
    if (actuallyRead == 0) break;

    pImageCharacteristic->setValue(buf, actuallyRead);
    pImageCharacteristic->indicate(); // <-- wait for client ack
    sent += actuallyRead;

    Serial.printf("Sent chunk: %u / %u\n", (unsigned)sent, (unsigned)total);
    delay(10); // optional small pause
  }

  // --- send END ---
  const char* endMsg = "END";
  pImageCharacteristic->setValue((uint8_t*)endMsg, strlen(endMsg));
  pImageCharacteristic->indicate();
  delay(150);

  Serial.println("✅ File transfer done");
  f.close();

   if (SPIFFS.exists(path)) {
    SPIFFS.remove(path);
    Serial.println("🗑️ Photo deleted from SPIFFS after send");
  }
  hasSavedPhoto = false;
}

// advertise helper (unchanged)
 void advertiseAndWaitForClientThenNotify(const char* msg, uint32_t maxWaitMs = 20000, uint32_t postNotifyWaitMs = 3000, int repeats = 3) {
  Serial.printf("📡 Advertise & notify helper: '%s' (wait %u ms)\n", msg, (unsigned)maxWaitMs);
  BLEAdvertising* adv = pServer->getAdvertising();
  adv->start();

  unsigned long start = millis();
  while (!deviceConnected && millis() - start < maxWaitMs) {
    delay(200);
  }

  if (deviceConnected) {
    Serial.println("🔗 client connected (helper) - sending notify(s)");
    for (int i = 0; i < repeats; ++i) {
      if (!deviceConnected) break;
      if (pPhaseCharacteristic) {
        pPhaseCharacteristic->setValue(msg);
        pPhaseCharacteristic->indicate();
      }
      delay(500);
    }
    unsigned long post = millis();
    while (deviceConnected && millis() - post < postNotifyWaitMs) {
      delay(100);
    }
  } else {
    Serial.println("⚠️ No client connected within helper timeout");
  }
  adv->stop();
  Serial.println("🛑 Advertising stopped (helper)");
}

// advertise and send photo (unchanged logic but will use updated sendFileViaBLE_SPIFFS)
void advertiseAndSendIfPhoto(void (*waitFunc)(), uint32_t sleepSec ) {
  Serial.println("📢 Start advertising for image transfer");
  pServer->getAdvertising()->start();

  unsigned long start = millis();
  const unsigned long MAX_WAIT = 20000;  // 20 sec to connect
  while (!deviceConnected && millis() - start < MAX_WAIT) {
    delay(200);
  }

  if (deviceConnected) {
    Serial.println("🔗 BLE client connected (for image transfer)");
    // wait for client to enable notifications/indications (up to 20s)
    if (waitForNotificationsEnabled(20000)) {
      Serial.println("✅ Client enabled indications/notify — proceeding to send photo");
      delay(500);

      for(int i=0;i<3;i++){
          if(!deviceConnected) break;
          if(pPhaseCharacteristic){
              pPhaseCharacteristic->setValue("READY_FOR_IMAGE");
              pPhaseCharacteristic->indicate();
          }
          delay(500);
      }

      if (hasSavedPhoto) sendFileViaBLE_SPIFFS(SAVED_PHOTO_PATH);
    } else {
      Serial.println("⚠️ Client did not enable indications/notify in time");
    }
  } else {
    Serial.println("⚠️ No client connected within timeout");
  }

  pServer->getAdvertising()->stop();
  Serial.println("🛑 Advertising stopped");

  // prepare wakeup and timer
  waitFunc();
  if (sleepSec > 0) {
    esp_sleep_enable_timer_wakeup((uint64_t)sleepSec * 1000000ULL);
    Serial.printf("⏲️ Timer wakeup after %u seconds\n", sleepSec);
  }

  Serial.println("💤 Going to deep sleep...\n");
  Serial.flush();
  delay(200);
  esp_deep_sleep_start();
}

// Camera helpers (unchanged)
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
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (!s) return false;
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);
  s->set_lenc(s, 1);

  if (s->id.PID == OV5640_PID) Serial.println("Camera: OV5640 detected");
  return true;
}

#include "SPIFFS.h"

bool saveFrameToSPIFFS_safe(camera_fb_t* fb, const char* finalPath) {
    if (!fb) return false;

    const char* tmpPath = "/last.jpg.tmp";

    if (SPIFFS.exists(tmpPath)) SPIFFS.remove(tmpPath);

    File f = SPIFFS.open(tmpPath, FILE_WRITE);
    if (!f) {
        Serial.println("❌ Cannot open tmp file for write");
        return false;
    }

    const size_t CHUNK = 4096; // 4 KB
    size_t totalWritten = 0;

    while (totalWritten < fb->len) {
        size_t toWrite = ((fb->len - totalWritten) > CHUNK) ? CHUNK : (fb->len - totalWritten);
        size_t written = f.write(fb->buf + totalWritten, toWrite);
        if (written == 0) {
            Serial.printf("❌ Write failed at offset %u\n", (unsigned)totalWritten);
            f.close();
            SPIFFS.remove(tmpPath);
            return false;
        }
        totalWritten += written;
    }

    f.close();

    if (SPIFFS.exists(finalPath)) SPIFFS.remove(finalPath);

    if (!SPIFFS.rename(tmpPath, finalPath)) {
        Serial.println("❌ Rename tmp -> final failed");
        SPIFFS.remove(tmpPath);
        return false;
    }

    File vf = SPIFFS.open(finalPath, FILE_READ);
    if (!vf) return false;

    bool ok = (vf.size() == fb->len);
    vf.close();

    if (!ok) {
        Serial.println("❌ Final size mismatch");
        SPIFFS.remove(finalPath);
        return false;
    }

    Serial.printf("✅ Photo saved successfully: %s (%u bytes)\n", finalPath, (unsigned)fb->len);
    return true;
}


// sendBLEandSleep and phases (unchanged logic)...
void sendBLEandSleep(const char* msg, void (*waitFunc)(), uint32_t sleepSec = 0) {
  Serial.printf("📡 Preparing to send: %s\n", msg);

  BLEDevice::startAdvertising();
  Serial.println("📢 BLE advertising started");

  unsigned long start = millis();
  const unsigned long MAX_WAIT = 20000;
  while (!g_clientConnected && millis() - start < MAX_WAIT) {
    delay(200);
  }

  if (g_clientConnected) {
    Serial.println("🔗 BLE client connected!");
    delay(800);
    for (int i = 0; i < 3; ++i) {
      if (!g_clientConnected) break;
      pPhaseCharacteristic->setValue(msg);
      pPhaseCharacteristic->indicate();
      delay(500);
    }

    unsigned long extraWait = millis();
    while (g_clientConnected && millis() - extraWait < 3000) {
      delay(100);
    }
  } else {
    Serial.println("⚠️ No BLE client connected within timeout.");
  }

  BLEDevice::stopAdvertising();
  Serial.println("🛑 BLE advertising stopped");

  waitFunc();

  if (sleepSec > 0) {
    esp_sleep_enable_timer_wakeup((uint64_t)sleepSec * 1000000ULL);
    Serial.printf("⏲️ Timer wakeup after %u seconds\n", sleepSec);
  }

  Serial.println("💤 Going to deep sleep...\n");
  Serial.flush();
  delay(300);
  esp_deep_sleep_start();
}

// --- допоміжна функція для надсилання фази ---
void notifyPhase(const char* phaseMsg) {
    if (!pServer || !pPhaseCharacteristic) return;

    BLEAdvertising* adv = pServer->getAdvertising();
    adv->start();

    unsigned long start = millis();
    while (!g_clientConnected && millis() - start < 5000) delay(100);

    if (g_clientConnected && notificationsEnabled) {
        for (int i = 0; i < 3; i++) { // кілька повторів для надійності
            pPhaseCharacteristic->setValue(phaseMsg);
            pPhaseCharacteristic->indicate();
            delay(500);
        }
        Serial.printf("📡 Phase notified: %s\n", phaseMsg);
    } else {
        Serial.println("⚠️ BLE client not connected or indications not enabled");
    }

    adv->stop();
}

// --- фази ---
void phaseRED() {
    currentPhase = PHASE_RED;
    Serial.println("🔴 Phase RED");
    showColor(255,0,0);

    notifyPhase("PHASE_RED"); // 🔹 повідомляємо завжди

    if (hasSavedPhoto && SPIFFS.exists(SAVED_PHOTO_PATH)) {
        advertiseAndSendIfPhoto(m_wait_closing);
    } else {
        sendBLEandSleep("PHASE_RED", m_wait_closing);
    }
}

void phaseBLUE() {
    currentPhase = PHASE_BLUE;
    Serial.println("🔵 Phase BLUE");
    showColor(0,0,255);

    notifyPhase("PHASE_BLUE"); // 🔹 повідомляємо завжди

    sendBLEandSleep("PHASE_BLUE", m_wait_opening, 30); // таймер 30с
}

void phaseGREEN() {
    currentPhase = PHASE_GREEN;
    Serial.println("🟢 Phase GREEN");
    showColor(0, 255, 0);

    // Видаляємо старі фото
    if (SPIFFS.exists(SAVED_PHOTO_PATH)) {
        SPIFFS.remove(SAVED_PHOTO_PATH);
        Serial.println("🧹 Old photo removed");
    }
    hasSavedPhoto = false;

    // Вмикаємо камеру
    pinMode(CAM_PWR_NUM, OUTPUT);
    digitalWrite(CAM_PWR_NUM, HIGH);
    delay(800); // даємо камері прогрітись

    if (!cameraInit()) {
        Serial.println("❌ Camera init failed in GREEN");
        hasSavedPhoto = false;
        digitalWrite(CAM_PWR_NUM, LOW);
        sendBLEandSleep("PHASE_GREEN", [](){ m_is_closed() ? m_wait_opening() : m_wait_closing(); });
        return;
    }

    // Пропускаємо перші кадри для стабілізації
    for (int i = 0; i < 2; i++) {
        camera_fb_t* fb_temp = esp_camera_fb_get();
        if (fb_temp) esp_camera_fb_return(fb_temp);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
        Serial.printf("Captured frame len=%u\n", (unsigned)fb->len);
        if (saveFrameToSPIFFS_safe(fb, SAVED_PHOTO_PATH)) {
            hasSavedPhoto = true;
        } else {
            Serial.println("❌ Failed to save photo in GREEN");
            hasSavedPhoto = false;
        }
        esp_camera_fb_return(fb);
    } else {
        Serial.println("❌ Capture failed in GREEN");
        hasSavedPhoto = false;
    }

    // Вимикаємо камеру
    digitalWrite(CAM_PWR_NUM, LOW);

    // --- Тут прибираємо перевірку на підключення BLE ---
    // Просто зберігаємо фото і відправляємо повідомлення GREEN
    sendBLEandSleep("PHASE_GREEN", [](){ m_is_closed() ? m_wait_opening() : m_wait_closing(); });
}

// SETUP
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  Serial.begin(115200);
  delay(500);

  if (!SPIFFS.begin()) {
    Serial.println("❌ SPIFFS mount failed (initial). Not formatting automatically.");
  } else {
    Serial.println("✅ SPIFFS mounted successfully (initial)");
  }

  if (SPIFFS.exists(SAVED_PHOTO_PATH)) {
    hasSavedPhoto = true;
  } else {
    hasSavedPhoto = false;
  }

  pinMode(OPEN_SW_NUM, INPUT);
  pinMode(RGB_VDD_NUM, OUTPUT);
  digitalWrite(RGB_VDD_NUM, HIGH);

  rgb.begin();
  rgb.clear();
  rgb.show();
  delay(10);
  myLED.begin(RGB_DIN_NUM, 1);
  myLED.brightness(255);

  rtc_gpio_init(OPEN_SW_NUM);
  rtc_gpio_set_direction(OPEN_SW_NUM, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en(OPEN_SW_NUM);
  rtc_gpio_pulldown_dis(OPEN_SW_NUM);

  // BLE init
  BLEDevice::init("ESP_Camera_Phase");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new PhaseServerCallbacks());

  // Phase service
  BLEService* phaseService = pServer->createService(SERVICE_UUID);
  pPhaseCharacteristic = phaseService->createCharacteristic(
      CHAR_PHASE_UUID,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  pPhaseCharacteristic->addDescriptor(new BLE2902());
  phaseService->start();

  // Image service
  BLEService* imageService = pServer->createService(IMAGE_SERVICE_UUID);
  // <-- IMPORTANT: use NOTIFY property (not INDICATE), and add BLE2902 descriptor
pImageCharacteristic = imageService->createCharacteristic(
  IMAGE_CHARACTERISTIC_UUID,
  BLECharacteristic::PROPERTY_INDICATE | BLECharacteristic::PROPERTY_READ
);
BLE2902* img2902 = new BLE2902();
img2902->setCallbacks(new MyDescriptorCallbacks());
pImageCharacteristic->addDescriptor(img2902);
 
  // Command characteristic
  pCmdCharacteristic = imageService->createCharacteristic(
      CMD_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE
  );
  pCmdCharacteristic->setCallbacks(new CmdCallbacks());

  imageService->start();

  // advertising
  BLEAdvertising* adv = pServer->getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->addServiceUUID(IMAGE_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinInterval(0x00A0);
  adv->setMaxInterval(0x00F0);

  BLEAdvertisementData advData;
  advData.setName("ESP");
  adv->setAdvertisementData(advData);

  adv->start();
  Serial.println("📡 BLE advertising started");

  BLEDevice::setMTU(517);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  bool closed = m_is_closed();

  Serial.printf("\nWakeup cause: %d\n", cause);
  Serial.printf("Magnet state: %s\n", closed ? "ON (є)" : "OFF (нема)");
  Serial.printf("Last phase: %d\n", currentPhase);
  Serial.printf("Has saved photo: %d\n", hasSavedPhoto ? 1 : 0);

  if (currentPhase == PHASE_GREEN) {
    if ((cause == ESP_SLEEP_WAKEUP_EXT0) && (!closed) && hasSavedPhoto) {
      Serial.println("Wake from GREEN due to magnet removal -> will advertise and send photo");
      advertiseAndSendIfPhoto(m_wait_closing);
    }
  }

  switch (currentPhase) {
    case PHASE_RED:
      if (closed) phaseBLUE();
      else phaseRED();
      break;
    case PHASE_BLUE:
      if (cause == ESP_SLEEP_WAKEUP_TIMER) phaseGREEN();
      else if (!closed) phaseRED();
      else phaseBLUE();
      break;
    case PHASE_GREEN:
      if (!closed) phaseRED();
      else phaseBLUE();
      break;
  }
}

void loop() {
  if (sendRequested) {
    sendRequested = false;
    if (hasSavedPhoto && deviceConnected) {
      Serial.println("Manual send requested via write command — trying to send saved photo");
      if (waitForNotificationsEnabled(5000)) {
        advertiseAndWaitForClientThenNotify("READY_FOR_IMAGE");
        sendFileViaBLE_SPIFFS(SAVED_PHOTO_PATH);
      } else {
        Serial.println("Client didn't enable notifications");
      }
    } else {
      Serial.println("No saved photo or no client to send");
    }
  }
  delay(50);
}
