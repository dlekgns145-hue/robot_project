#include "HX711.h"
#include <Servo.h>

const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;
const int SERVO_PIN = 11;
const int READY_LED_PIN = LED_BUILTIN;

HX711 scale;
Servo myServo;

const float DETECT_THRESHOLD = 0.8;
const float REMOVE_THRESHOLD = 0.5;
const float SORT_THRESHOLD = 2.8;
const unsigned long SETTLE_TIME = 1500;
const unsigned long REMOVE_MID_BAND_TIMEOUT = 3000;
const unsigned long REMOVE_POLL_TIME = 100;
const int REMOVE_CONFIRM_SAMPLES = 3;

void waitForRemoval() {
  int belowRemoveCount = 0;
  unsigned long midBandStartedAt = 0;

  while (true) {
    float current = scale.get_units(1);

    if (current < REMOVE_THRESHOLD) {
      belowRemoveCount++;
      if (belowRemoveCount >= REMOVE_CONFIRM_SAMPLES) {
        return;
      }
    }
    else {
      belowRemoveCount = 0;
    }

    // 0.5~0.8g 잔류값은 새 물건으로 다시 감지되지 않는 구간이다.
    // 이 구간에 3초 이상 머물면 제거된 것으로 판정해 무한 대기를 막는다.
    if (current < DETECT_THRESHOLD) {
      if (midBandStartedAt == 0) {
        midBandStartedAt = millis();
      }
      else if (millis() - midBandStartedAt >= REMOVE_MID_BAND_TIMEOUT) {
        Serial.println("잔류 무게 확인. 제거된 것으로 판정");
        return;
      }
    }
    else {
      midBandStartedAt = 0;
    }

    delay(REMOVE_POLL_TIME);
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("무게 분류 시스템 시작!");

  pinMode(READY_LED_PIN, OUTPUT);
  digitalWrite(READY_LED_PIN, LOW);

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(2280.f);
  scale.tare();

  myServo.attach(SERVO_PIN);
  myServo.write(90);

  Serial.println("영점 조절 완료. 물건을 올려주세요.");
  digitalWrite(READY_LED_PIN, HIGH);
}

void loop() {
  float instant = scale.get_units(1);

  if (instant >= DETECT_THRESHOLD) {
    digitalWrite(READY_LED_PIN, LOW);
    Serial.println("물건 감지! 안정화 대기 중...");
    delay(SETTLE_TIME);

    float weight = scale.get_units(5);
    Serial.print("측정된 무게: ");
    Serial.print(weight, 1);
    Serial.println(" g");

    if (weight >= SORT_THRESHOLD) {
      Serial.println("2.8g 이상! [오른쪽(180도)]으로 분류");
      myServo.write(180);
      delay(2000);
      myServo.write(90);
      delay(1000);
    }
    else if (weight >= DETECT_THRESHOLD && weight < SORT_THRESHOLD) {
      Serial.println("2.8g 미만! [왼쪽(0도)]으로 분류");
      myServo.write(0);
      delay(2000);
      myServo.write(90);
      delay(1000);
    }

    Serial.println("물건이 치워지길 기다리는 중...");
    waitForRemoval();
    Serial.println("대기 상태로 복귀");
    digitalWrite(READY_LED_PIN, HIGH);
  }

  delay(200);
}
