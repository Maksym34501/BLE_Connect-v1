#include <Arduino.h>
#include "esp_sleep.h"

#define OPEN_SW_NUM   18    // Магнітний датчик (геркон або Hall)
#define LED_TOP_NUM   12
#define LED_SIDE_NUM  21

void goToDeepSleepWithTimer(uint64_t seconds) {
  Serial.printf("Йдемо в глибокий сон на %llu секунд...\n", seconds);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_timer_wakeup(seconds * 1000000ULL);
  esp_deep_sleep_start();
}

void goToDeepSleepUntilMagnetChange() {
  Serial.println("Йдемо в сон, поки не зміниться стан магніту...");
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  // Прокидання, коли пін стає HIGH (тобто магніт зник)
  esp_sleep_enable_ext0_wakeup((gpio_num_t)OPEN_SW_NUM, 1);
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  pinMode(OPEN_SW_NUM, INPUT_PULLUP);
  pinMode(LED_TOP_NUM, OUTPUT);
  pinMode(LED_SIDE_NUM, OUTPUT);

  delay(200);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  if (cause == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("👉 Прокинулись через зміну магніту (EXT0)");
    // Магніт зник — вмикаємо світлодіоди на 5 секунд
    digitalWrite(LED_TOP_NUM, HIGH);
    digitalWrite(LED_SIDE_NUM, HIGH);
    delay(5000);
    digitalWrite(LED_TOP_NUM, LOW);
    digitalWrite(LED_SIDE_NUM, LOW);

    // Чекаємо появу магніту (LOW на вході)
    unsigned long start = millis();
    while (millis() - start < 30000) {  // максимум 30 секунд
      if (digitalRead(OPEN_SW_NUM) == LOW) {
        Serial.println("✅ Магніт знову з’явився!");
        goToDeepSleepWithTimer(30);  // спати 30 секунд
      }
      delay(100);
    }

    // Якщо магніт не з’явився — теж спимо на 30 секунд
    Serial.println("⏱ Магніт не з’явився — все одно йдемо спати на 30 сек...");
    goToDeepSleepWithTimer(30);
  } 
  else if (cause == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("⏰ Прокинулись після таймера");
    // Активні 5 секунд
    digitalWrite(LED_TOP_NUM, HIGH);
    digitalWrite(LED_SIDE_NUM, HIGH);
    delay(5000);
    digitalWrite(LED_TOP_NUM, LOW);
    digitalWrite(LED_SIDE_NUM, LOW);
    goToDeepSleepUntilMagnetChange();
  } 
  else {
    Serial.println("🔋 Початковий запуск або Reset");
    bool magnetPresent = (digitalRead(OPEN_SW_NUM) == LOW);
    if (magnetPresent) {
      goToDeepSleepUntilMagnetChange();
    } else {
      Serial.println("Магніт відсутній — активна фаза 5 сек");
      digitalWrite(LED_TOP_NUM, HIGH);
      digitalWrite(LED_SIDE_NUM, HIGH);
      delay(5000);
      digitalWrite(LED_TOP_NUM, LOW);
      digitalWrite(LED_SIDE_NUM, LOW);
      goToDeepSleepWithTimer(30);
    }
  }
}
//111f11git111
void loop() {
  // Не використовується, бо після кожного пробудження setup() викликається знову
  Serial.print("1121");
}