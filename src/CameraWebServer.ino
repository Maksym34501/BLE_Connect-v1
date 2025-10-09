#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "esp_sleep.h"

#define OPEN_SW_NUM   18
#define LED_TOP_NUM   12
#define LED_SIDE_NUM  21
#define RGB_DIN_NUM   20
#define RGB_VDD_NUM   19

Adafruit_NeoPixel pixel(1, RGB_DIN_NUM, NEO_GRB + NEO_KHZ800);

void goToDeepSleepUntilMagnetState(int targetState) {
  Serial.printf("💤 Сон до стану магніта = %d...\n", targetState);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)OPEN_SW_NUM, targetState);
  esp_deep_sleep_start();
}

void goToDeepSleepTimer(uint64_t seconds) {
  Serial.printf("💤 Сон на %llu секунд...\n", seconds);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_timer_wakeup(seconds * 1000000ULL);
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

  delay(200);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  bool magnetPresent = (digitalRead(OPEN_SW_NUM) == LOW);

  if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
    // 🔋 Початковий запуск
    Serial.println("🔋 Початковий запуск");
    if (magnetPresent) {
      Serial.println("Магніт присутній → сон до його прибирання");
      goToDeepSleepUntilMagnetState(1);
    } else {
      activePhase(255, 0, 0, "🚫 Початково без магніта → червона фаза");
      goToDeepSleepUntilMagnetState(0);
    }
  }
  else if (cause == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("👉 Пробудження через зміну магніта");

    if (!magnetPresent) {
      // 🟥 Магніт щойно прибрали
      activePhase(255, 0, 0, "🚫 Магніт прибрано → червона фаза");
      goToDeepSleepUntilMagnetState(0); // спати поки не з’явиться
    } else {
      // 🟦 Магніт знову з’явився — синя фаза + таймер
      activePhase(0, 0, 255, "🔵 Магніт повернувся → синя фаза (запуск 30 сек)");
      goToDeepSleepTimer(30);
    }
  }
  else if (cause == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("⏰ Пробудження після таймера (30 секунд)");
    magnetPresent = (digitalRead(OPEN_SW_NUM) == LOW);

    if (magnetPresent) {
      // 🟩 Магніт досі присутній → нормальна зелена фаза
      activePhase(0, 255, 0, "🟩 30 сек минуло → зелена фаза");
      goToDeepSleepUntilMagnetState(1); // знову чекаємо прибирання
    } else {
      // 🟧 Магніт зник під час 30 секунд
      activePhase(255, 100, 0, "🟧 Магніт зник під час 30 секунд → помаранчева фаза");
      // Після цього пристрій не запускає новий таймер, а просто знову чекає появи
      goToDeepSleepUntilMagnetState(0);
    }
  }
}

void loop() {
  // після кожного пробудження setup() запускається заново
}