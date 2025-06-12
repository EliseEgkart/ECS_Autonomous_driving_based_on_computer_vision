#include <Arduino.h>
#include "PinChangeInterrupt.h"
#include "Servo.h"

// --- PWM 입력 핀 정의 ---
#define pinRC2_straight       A0
#define pinRC1_direct         A1

// --- LED 핀 정의 ---
#define LEFT_LED_PIN   7
#define RIGHT_LED_PIN  8

// --- RC car 제어 핀 정의 ---
#define control_straight_pin  9
#define control_direct_pin    10

// --- PWM 실제 범위 ---
#define PWM_MIN   1068
#define PWM_MID   1492
#define PWM_MAX   1932


#define PWM_DEADZONE 20
#define PWM_STRAIGHT_FRONT 1535
#define PWM_STRAIGHT_BACK 1445

// 변동범위 설정
  // 직진 값 PWM_STRAIGHT_FRONT + PWM_VARIATION
  // 후진 값 PWM_STRAIGHT_BACK - PWM_VARIATION
#define PWM_VARIATION 22

// 좌우 깜빡이 Deadzone 설정
#define LR_SEPARATION 150

// --- RC 입력 변수 ---
volatile int            nRC2PulseWidth = PWM_MID;
volatile unsigned long  uRC2StartHigh  = 0;
volatile boolean        bNewRC2Pulse   = false;

volatile int            nRC1PulseWidth = PWM_MID;
volatile unsigned long  uRC1StartHigh  = 0;
volatile boolean        bNewRC1Pulse   = false;

// --- Servo 객체 ---
Servo straight_motor;
Servo direct_motor;

// --- 최종 제어 PWM 변수 ---
unsigned long straight_pwm = PWM_MID;
unsigned long direct_pwm   = PWM_MID;

void pwmRC2_Straight();
void pwmRC1_Direct();
void straight_Control();
void direct_Control();
void updateLEDs();

void setup() {
  pinMode(pinRC2_straight, INPUT_PULLUP);
  pinMode(pinRC1_direct,   INPUT_PULLUP);

  pinMode(LEFT_LED_PIN,   OUTPUT);
  pinMode(RIGHT_LED_PIN,  OUTPUT);
  digitalWrite(LEFT_LED_PIN,   LOW);
  digitalWrite(RIGHT_LED_PIN,  LOW);

  straight_motor.attach(control_straight_pin);
  direct_motor.attach(control_direct_pin);

  attachPCINT(digitalPinToPCINT(pinRC2_straight), pwmRC2_Straight, CHANGE);
  attachPCINT(digitalPinToPCINT(pinRC1_direct),   pwmRC1_Direct,   CHANGE);

  Serial.begin(9600);
}

// -------------------------------------------------------------

void pwmRC2_Straight() {
  if (digitalRead(pinRC2_straight) == HIGH) {
    uRC2StartHigh = micros();
  } else if (uRC2StartHigh && !bNewRC2Pulse) {
    nRC2PulseWidth = (int)(micros() - uRC2StartHigh);
    uRC2StartHigh  = 0;
    bNewRC2Pulse   = true;
  }
}

void pwmRC1_Direct() {
  if (digitalRead(pinRC1_direct) == HIGH) {
    uRC1StartHigh = micros();
  } else if (uRC1StartHigh && !bNewRC1Pulse) {
    nRC1PulseWidth = (int)(micros() - uRC1StartHigh);
    uRC1StartHigh  = 0;
    bNewRC1Pulse   = true;
  }
}

// -----------------------------------------------------------------------

volatile int drive_state = 0; // 0: 후진, 1: 좌회전, 2: 우회전, 3: 전진

// 전진·후진 제어 (데드존 + 스케일 + 클램프)
void straight_Control() {
  if (!bNewRC2Pulse) {
    straight_motor.writeMicroseconds(PWM_MID);
    return;
  }
  bNewRC2Pulse = false;
  int selected_pwm = nRC2PulseWidth; // 1800 이상이면 자동 모드, 아니면 수동 모드

  if((selected_pwm < PWM_MID + PWM_DEADZONE) && (selected_pwm > PWM_MID - PWM_DEADZONE)) {
    straight_pwm = PWM_MID;
  }
  else if(selected_pwm >= PWM_MID + PWM_DEADZONE) {
    straight_pwm = PWM_STRAIGHT_FRONT + PWM_VARIATION;
  }
  else if(selected_pwm <= PWM_MID - PWM_DEADZONE){
    straight_pwm = PWM_STRAIGHT_BACK - PWM_VARIATION;
  }
  
  straight_motor.writeMicroseconds(straight_pwm);
}

// 조향 제어 (원래대로 입력 펄스폭 그대로)
void direct_Control() {
  if (!bNewRC1Pulse) {
    direct_motor.writeMicroseconds(PWM_MID-157);
    return;
  }
  bNewRC1Pulse = false;
  direct_pwm = nRC1PulseWidth - 157;
  direct_motor.writeMicroseconds(direct_pwm);
}

void updateLEDs() {
  static unsigned long ledTimer = 0;
  static bool ledOn = false;

  switch (drive_state) {
    case 3: // 전진: 모두 끄기
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
      break;
    case 1: // 좌회전: 왼쪽 LED 350ms 켜짐, 350ms 꺼짐 반복
      if (millis() - ledTimer >= 200) {
        ledOn = !ledOn;
        ledTimer = millis();
      }
      digitalWrite(LEFT_LED_PIN, ledOn ? HIGH : LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
      break;
    case 2: // 우회전: 오른쪽 LED 350ms 켜짐, 350ms 꺼짐 반복
      if (millis() - ledTimer >= 200) {
        ledOn = !ledOn;
        ledTimer = millis();
      }
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, ledOn ? HIGH : LOW);
      break;
    case 0: // 후진: 양쪽 항상 켜기
      digitalWrite(LEFT_LED_PIN, HIGH);
      digitalWrite(RIGHT_LED_PIN, HIGH);
      break;
    default:
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
      break;
  }
}

void loop() {
  static unsigned long lastControlMicros = 0;
  unsigned long now = micros();

  // 20ms(20000µs)마다 한 번씩만 제어 함수 실행
  if (now - lastControlMicros >= 20000) {
    straight_Control();
    direct_Control();
    lastControlMicros = now;

    // --- drive_state 판별 ---
    int straight_val = nRC2PulseWidth;
    int direct_val   = nRC1PulseWidth;

    // drive_state 판별 로직 
    if (straight_val < PWM_MID - PWM_DEADZONE) drive_state = 0; // 후진
    else if (direct_val < PWM_MID - LR_SEPARATION) drive_state = 1; // 좌회전
    else if (direct_val > PWM_MID + LR_SEPARATION) drive_state = 2; // 우회전
    else if (straight_val > PWM_MID + PWM_DEADZONE) drive_state = 3; // 전진
    else drive_state = 3; // 기본 전진

    updateLEDs();
  }
}
