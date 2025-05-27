#include <Arduino.h>
#include "PinChangeInterrupt.h"
#include "Servo.h"

// --- 입력/출력 핀 정의 ---
#define pinRC2_straight       A0
#define pinRC1_direct         A1
#define pinRC9_Auto           A2

#define control_straight_pin  9
#define control_direct_pin    10

// --- PWM 실제 범위 ---
#define PWM_MIN   1068
#define PWM_MID   1492
#define PWM_MAX   1932
#define PWM_DEADZONE 20
#define PWM_STRAIGHT_FRONT 1545
#define PWM_STRAIGHT_BACK 1450

// 변동범위 설정
  // PWM_STRAIGHT_MIN + PWM_VARIATION = 1580, PWM_DIRECT_MIN - PWM_VARIATION = 1420
#define PWM_VARIATION 30  

// --- RC 입력 변수 ---
volatile int            nRC2PulseWidth = PWM_MID;
volatile unsigned long  uRC2StartHigh  = 0;
volatile boolean        bNewRC2Pulse   = false;

volatile int            nRC1PulseWidth = PWM_MID;
volatile unsigned long  uRC1StartHigh  = 0;
volatile boolean        bNewRC1Pulse   = false;

volatile int            nRC9PulseWidth = PWM_MID;
volatile unsigned long  uRC9StartHigh  = 0;
volatile boolean        bNewRC9Pulse   = false;

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
void processSerialInput();

void setup() {
  pinMode(pinRC2_straight, INPUT_PULLUP);
  pinMode(pinRC1_direct,   INPUT_PULLUP);
  pinMode(pinRC9_Auto,     INPUT_PULLUP);

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

void pwmRC9_Auto() {
  if (digitalRead(pinRC9_Auto) == HIGH) {
    uRC9StartHigh = micros();
  } else if (uRC9StartHigh && !bNewRC9Pulse) {
    nRC9PulseWidth = (int)(micros() - uRC9StartHigh);
    uRC9StartHigh  = 0;
    bNewRC9Pulse   = true;
  }
}

// -----------------------------------------------------------------------
volatile int auto_straight_pwm = 1500;
volatile int auto_direct_pwm = 1500;


// 전진·후진 제어 (데드존 + 스케일 + 클램프)
void straight_Control() {
  if (!bNewRC2Pulse) {
    straight_motor.writeMicroseconds(PWM_MID);
    return;
  }
  bNewRC2Pulse = false;
  int selected_pwm = nRC9PulseWidth >= 1800 ? auto_straight_pwm : nRC2PulseWidth; // 1800 이상이면 자동 모드, 아니면 수동 모드

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
  if( nRC9PulseWidth >= 1800) direct_pwm = auto_direct_pwm - 185; // 157 - 1500 + 1492 = 185
  else direct_pwm = nRC1PulseWidth - 157; // 수동 모드
  direct_motor.writeMicroseconds(direct_pwm);
}

void processSerialInput() {
  static String inputString = "";
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      // inputString: "123,456"
      int commaIndex = inputString.indexOf(',');
      if (commaIndex > 0) {
        auto_straight_pwm = inputString.substring(0, commaIndex).toInt();
        auto_direct_pwm = inputString.substring(commaIndex + 1).toInt();
      }
      inputString = "";
    }
    else {
      inputString += inChar;
    }
  }
}

void loop() {
  static unsigned long lastControlMicros = 0;
  unsigned long now = micros();
  processSerialInput();

  // 20ms(20000µs)마다 한 번씩만 제어 함수 실행
  if (now - lastControlMicros >= 20000) {
    straight_Control();
    direct_Control();
    lastControlMicros = now;
  }

  processSerialInput();
}
