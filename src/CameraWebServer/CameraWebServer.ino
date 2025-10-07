#include "soc/soc.h"           // Brownout error fix
#include "soc/rtc_cntl_reg.h"  // Brownout error fix
#include <esp32-hal-log.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "esp_http_server.h"
#include "esp_camera.h"
#include "camera_pins.h"
#include <esp_task_wdt.h>

#include <LiteLED.h>
#include "rgb_led.h"

#include <Arduino.h> 

// --- NEW: BLE ---
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define OPEN_SWITCH_POLARITE        LOW
#define TEST_POINT_10       GPIO_NUM_5

#define PART_BOUNDARY "123456789000000000000987654321"                     

const char preload_ssid[] PROGMEM = "TP-Link_15AE"; // <- Here
const char preload_pass[] PROGMEM = "29117130"; // <- Here

LiteLED myLED( LED_STRIP_SK6812, true );

IPAddress local_IP(192, 168, 0, 222);  
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

float batVoltage = 3.77;
void startCameraServer();

int ledSideChannel = 5;
int ledTopChannel = 7;
const int pwmfreq = 50000;
const int pwmresolution = 9;
const int pwmMax = pow(2, pwmresolution)-1;

const uint8_t LED_TOP_PWM_HIGH = 80;
const uint8_t LED_SIDE_PWM_HIGH = 80;

static void gradient_nonblocking (void *parameters);
volatile bool task_done = true;
struct blink_params { rgb_t channel; uint max; uint time; uint count; };

class blink_ex {
  public:
  void create_gradient(uint v, uint t, uint c) {
    bp.max = v; bp.time = t; bp.count = c;
    while (!task_done) vTaskDelay(pdMS_TO_TICKS(1));
    task_done = false;
    xTaskCreatePinnedToCore(gradient_nonblocking, "gradientTask", 2000, (void*) &bp, 1, NULL, (BaseType_t)((xPortGetCoreID() == 0) ? 1 : 0));
  };
  void set_rgb(uint8_t red, uint8_t green, uint8_t blue) {
    rgb_t pixel; pixel.red = red; pixel.green = green; pixel.blue = blue;
    myLED.setPixel( 0, pixel, 1 );
  }
  private: blink_params bp;
} blinker;

static void set_led(int newVal, int ledChannel) {
  if (newVal != -1) {
    #if (HW_VERSION == ovulio_rev1)
      if (ledChannel == ledTopChannel) newVal = 100 - newVal;
    #endif
    int brightness = round(pwmMax*newVal/100);
    ledcWrite(ledChannel, brightness);
  }
}

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
httpd_handle_t stream_httpd = NULL;

// BLE UUIDs
#define SERVICE_UUID              "12345678-1234-1234-1234-1234567890ab"
#define IMAGE_CHARACTERISTIC_UUID "abcd1234-ab12-cd34-ef56-1234567890ab"
#define CMD_CHARACTERISTIC_UUID   "12345678-abcd-1234-abcd-1234567890cd"

BLEServer* pServer = nullptr;
BLECharacteristic* pImageCharacteristic = nullptr;
BLECharacteristic* pCmdCharacteristic = nullptr;
BLEDescriptor* pImageCCCD = nullptr;

bool deviceConnected = false;
volatile bool sendRequested = false;
volatile bool notificationsEnabled = false; // true коли клієнт підписався на indicate

// Серверні callback'и
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("BLE: client connected");
  }

  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    notificationsEnabled = false;
    Serial.println("BLE: client disconnected");

    // Перезапуск advertising
    pServer->getAdvertising()->start();
    Serial.println("BLE: advertising restarted");
  }
};

// CCCD callback — обробляємо запис клієнта (indicate)
class MyDescriptorCallbacks : public BLEDescriptorCallbacks {
  void onWrite(BLEDescriptor* desc) override {
    uint8_t* buf = desc->getValue();
    size_t len = desc->getLength();
    if(len >= 2) {
        uint16_t val = ((uint16_t)buf[1] << 8) | buf[0];
        notificationsEnabled = (val & 0x0002) != 0;  // 0x0002 для indicate
        Serial.printf("CCCD written: 0x%04X, indicationsEnabled=%d\n",
                      val, notificationsEnabled);
    }
  }
};

// Command callback — клієнт пише команду на фото
class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    std::string val = pCharacteristic->getValue();
    if (val.length() > 0) {
      sendRequested = true;
      Serial.println("BLE: Capture command received (flag set)");
    }
  }
};

// Helper: wait until notificationsEnabled or timeout (ms)
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

// Function to send JPEG via BLE safely
// Function to send JPEG via BLE safely with byte count output
void sendFrameViaBLE(camera_fb_t* fb) {
  if (!deviceConnected || !fb) {
    Serial.println("sendFrameViaBLE: no client or no frame");
    return;
  }

  Serial.println("sendFrameViaBLE: waiting for notifications enabled...");
  vTaskDelay(pdMS_TO_TICKS(200)); 

  uint32_t len = fb->len;
  Serial.printf("Frame length: %u bytes\n", (unsigned)len);

  // Відправляємо 4 байти довжини кадру
  uint8_t lenBuf[4];
  lenBuf[0] = (uint8_t)(len & 0xFF);
  lenBuf[1] = (uint8_t)((len >> 8) & 0xFF);
  lenBuf[2] = (uint8_t)((len >> 16) & 0xFF);
  lenBuf[3] = (uint8_t)((len >> 24) & 0xFF);

  pImageCharacteristic->setValue(lenBuf, 4);
  pImageCharacteristic->indicate();;
  vTaskDelay(pdMS_TO_TICKS(50)); // даємо стеку час

  // Передача кадру у більших чанках
  const size_t chunkSize = 500; 
  size_t sent = 0;
  size_t total = fb->len;
  size_t totalSent = 0;

  while (sent < total) {
    if (!deviceConnected) {
      Serial.println("sendFrameViaBLE: client disconnected during transfer");
      break;
    }

    size_t remain = total - sent;
    size_t toSend = (chunkSize < remain) ? chunkSize : remain;
    pImageCharacteristic->setValue(fb->buf + sent, toSend);
    pImageCharacteristic->indicate();
    sent += toSend;
    totalSent += toSend;

    Serial.printf("Chunk sent: %u bytes, Total sent: %u bytes\n", (unsigned)toSend, (unsigned)totalSent);

    // Трохи довша пауза між пакетами для стабільності
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  Serial.printf("sendFrameViaBLE: done, total bytes sent = %u\n", (unsigned)totalSent);
}

// HTTP stream handler
static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char part_buf[64];

  httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK) return res;

  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }

    if(res == ESP_OK){
      size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    }
    if(res != ESP_OK) break;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return res;
}

void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
  
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
  }
}

 
bool cameraInit ()
{
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
    log_e("Camera init failed with error 0x%x", err);
    return false;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_special_effect(s, 1); // (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 0); // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_lenc(s, 1);

  if (s->id.PID == OV5640_PID) {
    log_i("OV5640_PID");
  }
  else
  {
    return false;
  }

  return true;
}
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector

  WiFi.mode(WIFI_OFF);
  Serial.begin(115200);
  delay(100);

  pinMode(LED_TOP_NUM, OUTPUT);
  ledcSetup(ledTopChannel, pwmfreq, pwmresolution);
  ledcAttachPin(LED_TOP_NUM, ledTopChannel);

  pinMode(LED_SIDE_NUM, OUTPUT);
  ledcSetup(ledSideChannel, pwmfreq, pwmresolution);
  ledcAttachPin(LED_SIDE_NUM, ledSideChannel);

  pinMode(CAM_PWR_NUM, OUTPUT); digitalWrite(CAM_PWR_NUM, LOW);
  pinMode(RESET_GPIO_NUM, OUTPUT); digitalWrite(RESET_GPIO_NUM, HIGH);

  #if (HW_VERSION > ovulio_rev2)
    pinMode(OPEN_SW_NUM, INPUT);
    pinMode(TEST_POINT_10, INPUT_PULLUP);
  #else
    pinMode(OPEN_SW_NUM, INPUT_PULLDOWN);
  #endif

  set_led(0, ledTopChannel);
  set_led(0, ledSideChannel);

  pinMode(RGB_DIN_NUM, OUTPUT); digitalWrite(RGB_DIN_NUM, LOW);
  pinMode(RGB_VDD_NUM, OUTPUT); digitalWrite(RGB_VDD_NUM, HIGH);

  myLED.begin(RGB_DIN_NUM, 1); 
  myLED.brightness(255);

  log_i("Start");
  log_i("Battery level: %fV", batVoltage);
  log_i("Total PSRAM: %d", ESP.getPsramSize());

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(preload_ssid, preload_pass);
  log_i("Connecting");
  while(WiFi.status() != WL_CONNECTED) vTaskDelay(pdMS_TO_TICKS(100));
  log_i("Connected to the WiFi network");

  digitalWrite(CAM_PWR_NUM, HIGH);
  delay(200);

  if (cameraInit()) {
    log_i("Camera init successfully");
    log_i("IP: %s", WiFi.localIP().toString().c_str());
    startCameraServer();
    set_led(LED_SIDE_PWM_HIGH, ledSideChannel);
    set_led(LED_TOP_PWM_HIGH, ledTopChannel);
  }

  // --- BLE init ---
  BLEDevice::init("ESP32_Camera");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Характеристика з INDICATE
  pImageCharacteristic = pService->createCharacteristic(
      IMAGE_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_INDICATE
  );

  pImageCCCD = new BLEDescriptor((uint16_t)0x2902);
  pImageCCCD->setCallbacks(new MyDescriptorCallbacks());
  pImageCharacteristic->addDescriptor(pImageCCCD);

  // Командна характеристика
  pCmdCharacteristic = pService->createCharacteristic(
      CMD_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE
  );
  pCmdCharacteristic->setCallbacks(new CmdCallbacks());

  pService->start();

  // Запуск advertising без виклику відсутніх методів
  BLEAdvertising* pAdvertising = pServer->getAdvertising();
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  // Збільшення MTU для великих чанків
  BLEDevice::setMTU(517);

  log_i("BLE started, waiting for connection");
}
enum State {
  WAIT_BEFORE_CAPTURE,
  CAPTURE_PHOTO,
  DONE
};

State currentState = WAIT_BEFORE_CAPTURE;
unsigned long waitStartTime = 0;

void loop() {
  // --- BLE advertising ---
  if (!deviceConnected) {
    pServer->getAdvertising()->start();
  }

  // Оголошуємо змінну поза switch
  camera_fb_t* fb = NULL;

  // --- Обробка натискання кнопки (BLE) ---
  if (sendRequested) {
    sendRequested = false;
    if (!deviceConnected) {
      Serial.println("Send requested but no BLE client connected");
    } else {
      Serial.println("Sending photo via BLE (requested by client)");

      // Прогріваємо камеру 2 кадрами
      for (size_t counter = 0; counter < 2; counter++) {
        fb = esp_camera_fb_get();
        if(fb) { 
          esp_camera_fb_return(fb); 
          fb = NULL; 
        }
        vTaskDelay(pdMS_TO_TICKS(50));
      }

      // Беремо фінальний кадр
      fb = esp_camera_fb_get();
      if(fb) {
        Serial.printf("Frame size is %d bytes\n", fb->len);
        sendFrameViaBLE(fb);
        esp_camera_fb_return(fb);
        Serial.println("Photo sent via BLE (button)");
      } else {
        Serial.println("Camera capture failed");
      }
    }
  }

  // --- Становий автомати ---
  switch (currentState) {

    case WAIT_BEFORE_CAPTURE: {
      if (waitStartTime == 0) {
        waitStartTime = millis();
        Serial.println("Waiting 1 minute before capture...");
      }
      if (millis() - waitStartTime >= 60000) {
        currentState = CAPTURE_PHOTO;
      }
      break;
    }

    case CAPTURE_PHOTO: {
      Serial.println("Capturing photo automatically...");

      // Прогріваємо камеру 2 кадрами
      for (size_t counter = 0; counter < 2; counter++) {
        fb = esp_camera_fb_get();
        if(fb) { 
          esp_camera_fb_return(fb); 
          fb = NULL; 
        }
        vTaskDelay(pdMS_TO_TICKS(50));
      }

      // Беремо фінальний кадр
      fb = esp_camera_fb_get();
      if(fb) {
        Serial.printf("Frame size is %d bytes\n", fb->len);
        sendFrameViaBLE(fb);
        esp_camera_fb_return(fb);
        Serial.println("Photo sent automatically");
      } else {
        Serial.println("Camera capture failed");
      }

      currentState = DONE;
      break;
    }

    case DONE: {
      delay(1000); // коротка пауза
      break;
    }
  }

  delay(20);
}