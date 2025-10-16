
// Об'єднаний скетч: фази + камера + збереження фото в SPIFFS + передача по BLE частинами
#include <Arduino.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"

#include <Adafruit_NeoPixel.h>

// ---------- Camera + SPIFFS ----------
#include "soc/soc.h"           // Brownout fix
#include "soc/rtc_cntl_reg.h"  // Brownout fix
#include "esp_camera.h"
#include "camera_pins.h"      // <- переконайтесь, що існує для вашого модуля
#include "FS.h"
#include "SPIFFS.h"

// ---------- BLE ----------
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------- Optional LED lib ----------
#include <LiteLED.h>
#include "rgb_led.h"

// ---------- Configuration ----------

 
#define OPEN_SW_NUM         GPIO_NUM_18
#define BAT_LEV_NUM         GPIO_NUM_1
#define RGB_DIN_NUM         GPIO_NUM_20
#define RGB_VDD_NUM         GPIO_NUM_19
#define OPEN_SWITCH_POLARITY 0   // 1 = замкнуто

#ifndef CAM_PWR_NUM
  #define CAM_PWR_NUM  4
#endif
#ifndef RESET_GPIO_NUM
  #define RESET_GPIO_NUM 15
#endif

// ---------- BLE UUIDs ----------
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0" // phase/info
#define CHAR_PHASE_UUID     "12345678-1234-5678-1234-56789abcdef1"

#define IMAGE_SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define IMAGE_CHARACTERISTIC_UUID "abcd1234-ab12-cd34-ef56-1234567890ab"
#define CMD_CHARACTERISTIC_UUID   "12345678-abcd-1234-abcd-1234567890cd"

// ---------- RGB ----------
Adafruit_NeoPixel rgb(1, RGB_DIN_NUM, NEO_GRB + NEO_KHZ800);
LiteLED myLED( LED_STRIP_SK6812, true );

// ---------- Globals ----------
BLEServer* pServer = nullptr;
BLECharacteristic* pPhaseCharacteristic = nullptr;

BLECharacteristic* pImageCharacteristic = nullptr;
BLEDescriptor* pImageCCCD = nullptr;
BLECharacteristic* pCmdCharacteristic = nullptr;

volatile bool g_clientConnected = false;
bool deviceConnected = false;
volatile bool notificationsEnabled = false;
volatile bool sendRequested = false;

// Phases and persistence across deep sleep
enum Phase { PHASE_RED = 0, PHASE_BLUE = 1, PHASE_GREEN = 2 };
RTC_DATA_ATTR Phase currentPhase = PHASE_RED;
RTC_DATA_ATTR bool hasSavedPhoto = false; // persist across deep sleep

const char * SAVED_PHOTO_PATH = "/last.jpg";

// Forward declarations
void phaseRED();
void phaseBLUE();
void phaseGREEN();
void showColor(uint8_t r, uint8_t g, uint8_t b);
bool cameraInit();
bool saveFrameToSPIFFS(camera_fb_t* fb, const char* path);
void sendFileViaBLE_SPIFFS(const char* path);
bool waitForNotificationsEnabled(uint32_t timeoutMs);
void advertiseAndSendIfPhoto(void (*waitFunc)(), uint32_t sleepSec = 0);  

// ---------- Helper: magnet ----------
bool m_is_closed() { return digitalRead(OPEN_SW_NUM) == OPEN_SWITCH_POLARITY; }
void m_wait_opening() { esp_sleep_enable_ext0_wakeup(OPEN_SW_NUM, !OPEN_SWITCH_POLARITY); }
void m_wait_closing() { esp_sleep_enable_ext0_wakeup(OPEN_SW_NUM, OPEN_SWITCH_POLARITY); }

// ---------- LEDs ----------
void showColor(uint8_t r, uint8_t g, uint8_t b) {
  rgb.setPixelColor(0, rgb.Color(r,g,b));
  rgb.show();
  rgb_t pix; pix.red = r; pix.green = g; pix.blue = b;
  myLED.setPixel(0, pix, 1);
}

// ---------- BLE Callbacks ----------
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
        notificationsEnabled = (val & 0x0002) != 0;  // indicate bit
        Serial.printf("CCCD written: 0x%04X, indicationsEnabled=%d\n", val, notificationsEnabled);
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

// ---------- BLE helpers ----------
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

void sendFileViaBLE_SPIFFS(const char* path) {
  if (!deviceConnected) {
    Serial.println("sendFileViaBLE_SPIFFS: no client connected");
    return;
  }
  if (!SPIFFS.exists(path)) {
    Serial.println("sendFileViaBLE_SPIFFS: file not found");
    return;
  }

  File f = SPIFFS.open(path, FILE_READ);
  if(!f) {
    Serial.println("sendFileViaBLE_SPIFFS: failed to open file");
    return;
  }

  size_t total = f.size();
  Serial.printf("sendFileViaBLE_SPIFFS: file size = %u bytes\n", (unsigned)total);

  // send 4-byte length (little-endian)
  uint8_t lenBuf[4];
  lenBuf[0] = (uint8_t)(total & 0xFF);
  lenBuf[1] = (uint8_t)((total >> 8) & 0xFF);
  lenBuf[2] = (uint8_t)((total >> 16) & 0xFF);
  lenBuf[3] = (uint8_t)((total >> 24) & 0xFF);

  pImageCharacteristic->setValue(lenBuf, 4);
  pImageCharacteristic->indicate();
  vTaskDelay(pdMS_TO_TICKS(50));

  const size_t chunkSize = 500; // safe chunk
  size_t sent = 0;
  uint8_t buf[chunkSize];

  while (sent < total) {
    if (!deviceConnected) {
      Serial.println("sendFileViaBLE_SPIFFS: client disconnected during transfer");
      break;
    }
    size_t toRead = (total - sent) > chunkSize ? chunkSize : (total - sent);
    size_t actuallyRead = f.read(buf, toRead);
    if (actuallyRead == 0) break;

    pImageCharacteristic->setValue(buf, actuallyRead);
    pImageCharacteristic->indicate();

    sent += actuallyRead;
    Serial.printf("Chunk sent: %u bytes, Total sent: %u bytes\n", (unsigned)actuallyRead, (unsigned)sent);
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  Serial.printf("sendFileViaBLE_SPIFFS: done, total bytes sent = %u\n", (unsigned)sent);
  f.close();

  if (sent == total) {
    SPIFFS.remove(path);
    hasSavedPhoto = false;
    Serial.println("Saved photo removed from SPIFFS after successful send");
  } else {
    Serial.println("Transfer incomplete, leaving file in SPIFFS");
  }
}

// Advertise and if photo exists send it, then configure wakeup and deep sleep
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
    // wait for client to enable indications (up to 8s)
    if (waitForNotificationsEnabled(8000)) {
      Serial.println("✅ Client enabled indications — proceeding to send if photo exists");
      if (hasSavedPhoto) sendFileViaBLE_SPIFFS(SAVED_PHOTO_PATH);
      else Serial.println("No saved photo to send");
    } else {
      Serial.println("⚠️ Client did not enable indications in time");
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

// ---------- Camera helpers ----------
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
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);
  s->set_lenc(s, 1);

  if (s->id.PID == OV5640_PID) Serial.println("Camera: OV5640 detected");
  return true;
}

bool saveFrameToSPIFFS(camera_fb_t* fb, const char* path) {
  if (!fb) return false;
  File f = SPIFFS.open(path, FILE_WRITE);
  if (!f) {
    Serial.println("saveFrameToSPIFFS: cannot open file for write");
    return false;
  }
  size_t written = f.write(fb->buf, fb->len);
  f.close();
  Serial.printf("saveFrameToSPIFFS: written %u bytes\n", (unsigned)written);
  return written == fb->len;
}

// ---------- PHASES ----------
void phaseRED() {
  currentPhase = PHASE_RED;
  Serial.println("🔴 Phase RED (магніт відсутній)");
  showColor(255, 0, 0);

  if (hasSavedPhoto) {
    // If photo exists, advertise and try to send it now (this covers case when photo saved earlier
    // and magnet removed while device was sleeping)
    if (pPhaseCharacteristic) {
      pPhaseCharacteristic->setValue("PHASE_RED_SENDING_PHOTO");
      pPhaseCharacteristic->notify();
    }
    advertiseAndSendIfPhoto(m_wait_closing);
  } else {
    if (pPhaseCharacteristic) {
      pPhaseCharacteristic->setValue("PHASE_RED");
      pPhaseCharacteristic->notify();
    }
    // short advertise so phone can read phase
    pServer->getAdvertising()->start();
    delay(300);
    pServer->getAdvertising()->stop();
    m_wait_closing();
    esp_deep_sleep_start();
  }
}

void phaseBLUE() {
  currentPhase = PHASE_BLUE;
  Serial.println("🔵 Phase BLUE (магніт присутній)");
  showColor(0, 0, 255);

  if (pPhaseCharacteristic) {
    pPhaseCharacteristic->setValue("PHASE_BLUE");
    pPhaseCharacteristic->notify();
  }

  // short advertise
  pServer->getAdvertising()->start();
  delay(300);
  pServer->getAdvertising()->stop();

  // after 30s go to GREEN
  if (pPhaseCharacteristic) {
    pPhaseCharacteristic->setValue("PHASE_BLUE_TIMER30");
    pPhaseCharacteristic->notify();
  }

  esp_sleep_enable_timer_wakeup((uint64_t)30 * 1000000ULL);
  m_wait_opening(); // also allow wake on opening
  Serial.println("💤 Going to deep sleep from BLUE (timer 30s)\n");
  Serial.flush();
  delay(200);
  esp_deep_sleep_start();
}

void phaseGREEN() {
  currentPhase = PHASE_GREEN;
  Serial.println("🟢 Phase GREEN (зйомка фото і збереження)");
  showColor(0, 255, 0);

  if (pPhaseCharacteristic) {
    pPhaseCharacteristic->setValue("PHASE_GREEN");
    pPhaseCharacteristic->notify();
  }

  // mount SPIFFS (format if needed)
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
  } else {
    Serial.println("SPIFFS mounted");
  }

  // power up camera if needed
  pinMode(CAM_PWR_NUM, OUTPUT);
  digitalWrite(CAM_PWR_NUM, HIGH);
  delay(200);

  if (!cameraInit()) {
    Serial.println("Camera init failed in GREEN phase");
  } else {
    // warm up frames
    for (int i=0;i<2;i++) {
      camera_fb_t* fb_temp = esp_camera_fb_get();
      if (fb_temp) {
        esp_camera_fb_return(fb_temp);
        fb_temp = NULL;
      }
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      Serial.printf("Captured frame len=%u\n", (unsigned)fb->len);
      if (saveFrameToSPIFFS(fb, SAVED_PHOTO_PATH)) {
        hasSavedPhoto = true;
        Serial.println("Photo saved to SPIFFS");
      } else {
        Serial.println("Failed to save photo to SPIFFS");
      }
      esp_camera_fb_return(fb);
    } else {
      Serial.println("Capture failed in GREEN");
    }
  }

  // power down camera
  digitalWrite(CAM_PWR_NUM, LOW);

  // GO TO SLEEP: wait for magnet change (opposite)
  if (m_is_closed()) m_wait_opening();
  else m_wait_closing();

  // small delay to let BLE/LED settle
  delay(200);
  Serial.println("💤 Going to deep sleep after GREEN (photo taken)\n");
  Serial.flush();
  delay(200);
  esp_deep_sleep_start();
}

// ---------- SETUP ----------
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // brownout fix
  
  Serial.begin(115200);
  delay(500);
if (!SPIFFS.begin(true)) {
    Serial.println("❌ SPIFFS mount failed (initial)");
  } else {
    Serial.println("✅ SPIFFS mounted successfully (initial)");
  }
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
  pImageCharacteristic = imageService->createCharacteristic(
      IMAGE_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_INDICATE
  );
  pImageCCCD = new BLEDescriptor((uint16_t)0x2902);
  pImageCCCD->setCallbacks(new MyDescriptorCallbacks());
  pImageCharacteristic->addDescriptor(pImageCCCD);

  // Command characteristic (optional)
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
  adv->start();
  Serial.println("📡 BLE advertising started");

  // Increase MTU (optional)
  BLEDevice::setMTU(517);

  // Wakeup cause & magnet state
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  bool closed = m_is_closed();

  Serial.printf("\nWakeup cause: %d\n", cause);
  Serial.printf("Magnet state: %s\n", closed ? "ON (є)" : "OFF (нема)");
  Serial.printf("Last phase: %d\n", currentPhase);
  Serial.printf("Has saved photo: %d\n", hasSavedPhoto ? 1 : 0);

  // ROUTING: handle special case: waking up after GREEN due to magnet removal
  if (currentPhase == PHASE_GREEN) {
    // If we woke from external wake (ext0) and magnet is now OFF (i.e. was removed)
    if ((cause == ESP_SLEEP_WAKEUP_EXT0) && (!closed) && hasSavedPhoto) {
      Serial.println("Wake from GREEN due to magnet removal -> will advertise and send photo");
      // advertise and send photo; after sending, m_wait_closing() will be set inside helper
      advertiseAndSendIfPhoto(m_wait_closing);
      // advertiseAndSendIfPhoto does deep_sleep_start() internally after sending config
      // (so code below won't execute in that path)
    } else {
      // either still closed or woke for another reason -> go to normal handling below
    }
  }

  // Normal phase routing
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
      // if here and didn't match special case above: if magnet removed -> RED, else BLUE
      if (!closed) phaseRED();
      else phaseBLUE();
      break;
  }
}

void loop() {
  // Manual command from client to request send (while awake)
  if (sendRequested) {
    sendRequested = false;
    if (hasSavedPhoto && deviceConnected) {
      Serial.println("Manual send requested via write command — trying to send saved photo");
      if (waitForNotificationsEnabled(5000)) {
        sendFileViaBLE_SPIFFS(SAVED_PHOTO_PATH);
      } else {
        Serial.println("Client didn't enable indications");
      }
    } else {
      Serial.println("No saved photo or no client to send");
    }
  }
  delay(50);
}
 