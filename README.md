# 임베디드 통신시스템 프로젝트 - ~~

## 팀 구성원 및 역할분담
| 이름 | 담당 업무 |
|------|-----------|
| **김형진** | 라즈베리파이 : 영상 처리, 라인 검출기반 조향 제어 |
| **김수민** | 아두이노 : RC 신호 해석, 서보모터 · ESC 제어 로직 |

---

## 시연 영상
아래 썸네일을 클릭하면 실제 주행 영상을 확인할 수 있습니다.
[![Video Label](http://img.youtube.com/vi/iOE3hzLU1VU/0.jpg)](https://youtu.be/iOE3hzLU1VU)

---

## 시스템 다이어그램
![시스템 다이어그램](image/Arduino_Control_Diagram.png)
`Raspberry Pi → UART → Arduino → 서보/ESC → RC Car` 의 흐름으로 데이터가 이동합니다.

## 실제 RC Car 사진
![실제 RC Car 사진](image/Car_side_image.png)
다음은 실제 구동하는 RC Car 사진입니다. 위쪽 면, 우측 면, 좌측 면에 해당하는 사진입니다.

## 개요

---

## 소스 동작원리 - ECS_Project_Arduino_manual
### 전체 로직 설명

이 아두이노 코드는 **RC 수신기**로부터 PWM 신호를 직접 받아 ESC(직진/후진)과 서보모터(조향)를 제어하는 **수동 주행 전용 프로그램**입니다.

- **CH2 (A0 핀)**: 직진/후진을 위한 PWM 입력 (스틱 위아래)
- **CH1 (A1 핀)**: 조향(좌/우)을 위한 PWM 입력 (스틱 좌우)
- **LED (7, 8번 핀)**: 방향지시등 표시
- **제어 주기**: 약 20ms 간격으로 PWM 신호를 판독하고 주행 상태를 판단함
- **핵심 기능**:
  - `PinChangeInterrupt`로 PWM 신호 폭 측정
  - `straight_Control()`과 `direct_Control()`로 제어
  - `drive_state`에 따라 LED 점등/점멸
  - PWM 데드존 및 오프셋 적용하여 안전한 제어

### 코드 전체 및 상세 주석

```cpp
#include <Arduino.h>
#include "PinChangeInterrupt.h"
#include "Servo.h"

// --- PWM 입력 핀 정의 (RC 수신기 연결)
#define pinRC2_straight       A0   // CH2: 직진/후진
#define pinRC1_direct         A1   // CH1: 좌/우 조향

// --- 방향지시등 LED 핀 정의
#define LEFT_LED_PIN          7
#define RIGHT_LED_PIN         8

// --- 제어 출력 핀 (서보모터, ESC)
#define control_straight_pin  9    // 직진/후진 ESC
#define control_direct_pin    10   // 조향 서보

// --- PWM 신호 기준값 (µs 단위)
#define PWM_MIN               1068
#define PWM_MID               1492  // 중립값 (약 1500)
#define PWM_MAX               1932

// --- 데드존 설정 (정지 상태 허용범위)
#define PWM_DEADZONE          20    // ±20 µs 내에서는 정지로 판단

// --- 전진/후진 기준 PWM
#define PWM_STRAIGHT_FRONT    1535
#define PWM_STRAIGHT_BACK     1445
#define PWM_VARIATION         22    // 전진·후진 시 증감값

// --- 조향 LED 판단 기준
#define LR_SEPARATION         150   // 중립에서 ±150 이상이면 방향 판단

// --- RC 입력값 저장용 변수 (CH2)
volatile int            nRC2PulseWidth = PWM_MID;
volatile unsigned long  uRC2StartHigh  = 0;
volatile boolean        bNewRC2Pulse   = false;

// --- RC 입력값 저장용 변수 (CH1)
volatile int            nRC1PulseWidth = PWM_MID;
volatile unsigned long  uRC1StartHigh  = 0;
volatile boolean        bNewRC1Pulse   = false;

// --- 제어 대상 서보 객체
Servo straight_motor;
Servo direct_motor;

// --- 최종 PWM 출력값
unsigned long straight_pwm = PWM_MID;
unsigned long direct_pwm   = PWM_MID;

// --- 주행 상태 표시용 (LED 출력용)
volatile int drive_state = 0; // 0: 후진, 1: 좌회전, 2: 우회전, 3: 전진

// --- PWM 측정용 인터럽트 함수 (직진/후진 채널)
void pwmRC2_Straight() {
  if (digitalRead(pinRC2_straight) == HIGH) {
    uRC2StartHigh = micros(); // 상승엣지 시간 기록
  } else if (uRC2StartHigh && !bNewRC2Pulse) {
    nRC2PulseWidth = (int)(micros() - uRC2StartHigh); // 펄스폭 계산
    uRC2StartHigh  = 0;
    bNewRC2Pulse   = true;
  }
}

// --- PWM 측정용 인터럽트 함수 (조향 채널)
void pwmRC1_Direct() {
  if (digitalRead(pinRC1_direct) == HIGH) {
    uRC1StartHigh = micros();
  } else if (uRC1StartHigh && !bNewRC1Pulse) {
    nRC1PulseWidth = (int)(micros() - uRC1StartHigh);
    uRC1StartHigh  = 0;
    bNewRC1Pulse   = true;
  }
}

// --- 아두이노 기본 설정
void setup() {
  // 입력 핀 초기화
  pinMode(pinRC2_straight, INPUT_PULLUP);
  pinMode(pinRC1_direct,   INPUT_PULLUP);

  // LED 핀 초기화
  pinMode(LEFT_LED_PIN,   OUTPUT);
  pinMode(RIGHT_LED_PIN,  OUTPUT);
  digitalWrite(LEFT_LED_PIN,   LOW);
  digitalWrite(RIGHT_LED_PIN,  LOW);

  // 서보 핀 설정
  straight_motor.attach(control_straight_pin);
  direct_motor.attach(control_direct_pin);

  // PWM 측정을 위한 인터럽트 등록
  attachPCINT(digitalPinToPCINT(pinRC2_straight), pwmRC2_Straight, CHANGE);
  attachPCINT(digitalPinToPCINT(pinRC1_direct),   pwmRC1_Direct,   CHANGE);

  Serial.begin(9600); // 디버깅용 시리얼 통신
}

// --- 직진/후진 제어 함수
void straight_Control() {
  if (!bNewRC2Pulse) {
    // 새 신호 없으면 중립값 유지
    straight_motor.writeMicroseconds(PWM_MID);
    return;
  }
  bNewRC2Pulse = false;

  int selected_pwm = nRC2PulseWidth;

  if((selected_pwm < PWM_MID + PWM_DEADZONE) &&
     (selected_pwm > PWM_MID - PWM_DEADZONE)) {
    straight_pwm = PWM_MID; // 정지 상태
  }
  else if(selected_pwm >= PWM_MID + PWM_DEADZONE) {
    straight_pwm = PWM_STRAIGHT_FRONT + PWM_VARIATION; // 전진
  }
  else if(selected_pwm <= PWM_MID - PWM_DEADZONE){
    straight_pwm = PWM_STRAIGHT_BACK - PWM_VARIATION;  // 후진
  }

  straight_motor.writeMicroseconds(straight_pwm);
}

// --- 조향 제어 함수
void direct_Control() {
  if (!bNewRC1Pulse) {
    direct_motor.writeMicroseconds(PWM_MID - 157); // 중립 출력
    return;
  }
  bNewRC1Pulse = false;

  direct_pwm = nRC1PulseWidth - 157; // 보정값 적용
  direct_motor.writeMicroseconds(direct_pwm);
}

// --- 주행 상태에 따라 방향지시등 출력
void updateLEDs() {
  static unsigned long ledTimer = 0;
  static bool ledOn = false;

  switch (drive_state) {
    case 3: // 전진: 꺼짐
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
      break;

    case 1: // 좌회전: 왼쪽 점멸
      if (millis() - ledTimer >= 200) {
        ledOn = !ledOn;
        ledTimer = millis();
      }
      digitalWrite(LEFT_LED_PIN, ledOn ? HIGH : LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
      break;

    case 2: // 우회전: 오른쪽 점멸
      if (millis() - ledTimer >= 200) {
        ledOn = !ledOn;
        ledTimer = millis();
      }
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, ledOn ? HIGH : LOW);
      break;

    case 0: // 후진: 양쪽 항상 켜짐
      digitalWrite(LEFT_LED_PIN, HIGH);
      digitalWrite(RIGHT_LED_PIN, HIGH);
      break;

    default: // 예외 처리
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
      break;
  }
}

// --- 메인 루프
void loop() {
  static unsigned long lastControlMicros = 0;
  unsigned long now = micros();

  // 20ms 주기마다 제어 수행
  if (now - lastControlMicros >= 20000) {
    straight_Control();
    direct_Control();
    lastControlMicros = now;

    // 주행 상태 판단
    int straight_val = nRC2PulseWidth;
    int direct_val   = nRC1PulseWidth;

    if (straight_val < PWM_MID - PWM_DEADZONE) {
      drive_state = 0; // 후진
    }
    else if (direct_val < PWM_MID - LR_SEPARATION) {
      drive_state = 1; // 좌회전
    }
    else if (direct_val > PWM_MID + LR_SEPARATION) {
      drive_state = 2; // 우회전
    }
    else if (straight_val > PWM_MID + PWM_DEADZONE) {
      drive_state = 3; // 전진
    }
    else {
      drive_state = 3; // 기본 전진
    }

    updateLEDs(); // 방향지시등 갱신
  }
}
```

## 소스 동작원리 - ECS_Project_Arduino_Auto
### 전체 로직 설명
이 코드는 RC 수신기의 PWM 입력뿐만 아니라, **Raspberry Pi로부터 전달되는 직진/조향 제어 값**을 동시에 처리할 수 있는 **수동·자동 겸용 RC 제어 프로그램**입니다.  
RC 송신기 CH9(스위치)를 통해 두 모드 간 전환이 가능합니다.

- **수동 모드** (CH9 < 1500µs): RC 송신기 CH1, CH2 입력값을 직접 서보 및 ESC에 전달
- **자동 모드** (CH9 ≥ 1500µs): Raspberry Pi에서 UART로 전송된 `"직진PWM,조향PWM\n"` 값을 사용
- **직진/후진**은 ESC (핀 9), **좌우 조향**은 서보모터 (핀 10)를 통해 제어
- **LED**는 전진/후진/좌/우 상태에 따라 점등 또는 점멸
### 코드 전체 및 상세 주석

```cpp
#include <Arduino.h>
#include "PinChangeInterrupt.h"
#include "Servo.h"

// --- PWM 입력 핀 정의 --- (RC 수신기 입력 핀)
#define pinRC2_straight       A0  // CH2: 직진/후진
#define pinRC1_direct         A1  // CH1: 조향
#define pinRC9_Auto           A2  // CH9: 자동/수동 모드 전환

// --- LED 핀 정의 --- (방향지시용)
#define LEFT_LED_PIN          7
#define RIGHT_LED_PIN         8

// --- 제어 핀 정의 --- (서보 및 ESC 제어)
#define control_straight_pin  9   // ESC
#define control_direct_pin    10  // 서보

// --- PWM 기준값 --- (µs 단위)
#define PWM_MIN               1068
#define PWM_MID               1492
#define PWM_MAX               1932

#define PWM_DEADZONE          20
#define PWM_STRAIGHT_FRONT    1535
#define PWM_STRAIGHT_BACK     1445
#define PWM_VARIATION         22

// --- 조향 판단 기준값 --- (좌우 구분용 deadband)
#define LR_SEPARATION         150

// --- 수신한 PWM 펄스폭 저장용 변수들 (CH2, CH1, CH9)
volatile int            nRC2PulseWidth = PWM_MID;
volatile unsigned long  uRC2StartHigh  = 0;
volatile boolean        bNewRC2Pulse   = false;

volatile int            nRC1PulseWidth = PWM_MID;
volatile unsigned long  uRC1StartHigh  = 0;
volatile boolean        bNewRC1Pulse   = false;

volatile int            nRC9PulseWidth = PWM_MID;
volatile unsigned long  uRC9StartHigh  = 0;
volatile boolean        bNewRC9Pulse   = false;

// --- 제어용 서보 객체
Servo straight_motor;
Servo direct_motor;

// --- PWM 최종 출력값
unsigned long straight_pwm = PWM_MID;
unsigned long direct_pwm   = PWM_MID;

// --- Pi에서 수신한 자동 제어용 PWM 값
volatile int auto_straight_pwm = 1500;
volatile int auto_direct_pwm   = 1500;

// --- 주행 상태 (LED 패턴용)
volatile int drive_state = 0; // 0:후진, 1:좌, 2:우, 3:전진

// --- 인터럽트 핸들러: CH2 (직진/후진)
void pwmRC2_Straight() {
  if (digitalRead(pinRC2_straight) == HIGH) {
    uRC2StartHigh = micros();
  } else if (uRC2StartHigh && !bNewRC2Pulse) {
    nRC2PulseWidth = (int)(micros() - uRC2StartHigh);
    uRC2StartHigh  = 0;
    bNewRC2Pulse   = true;
  }
}

// --- 인터럽트 핸들러: CH1 (조향)
void pwmRC1_Direct() {
  if (digitalRead(pinRC1_direct) == HIGH) {
    uRC1StartHigh = micros();
  } else if (uRC1StartHigh && !bNewRC1Pulse) {
    nRC1PulseWidth = (int)(micros() - uRC1StartHigh);
    uRC1StartHigh  = 0;
    bNewRC1Pulse   = true;
  }
}

// --- 인터럽트 핸들러: CH9 (자동/수동 스위치)
void pwmRC9_Auto() {
  if (digitalRead(pinRC9_Auto) == HIGH) {
    uRC9StartHigh = micros();
  } else if (uRC9StartHigh && !bNewRC9Pulse) {
    nRC9PulseWidth = (int)(micros() - uRC9StartHigh);
    uRC9StartHigh  = 0;
    bNewRC9Pulse   = true;
  }
}

// --- 직진/후진 제어 함수
void straight_Control() {
  if (!bNewRC2Pulse) {
    straight_motor.writeMicroseconds(PWM_MID);
    return;
  }
  bNewRC2Pulse = false;

  // CH9 값에 따라 수동 또는 자동 PWM 선택
  int selected_pwm = (nRC9PulseWidth >= 1500) ? auto_straight_pwm : nRC2PulseWidth;

  // 데드존 내 정지
  if ((selected_pwm < PWM_MID + PWM_DEADZONE) && (selected_pwm > PWM_MID - PWM_DEADZONE)) {
    straight_pwm = PWM_MID;
  }
  else if (selected_pwm >= PWM_MID + PWM_DEADZONE) {
    straight_pwm = PWM_STRAIGHT_FRONT + PWM_VARIATION; // 전진
  }
  else if (selected_pwm <= PWM_MID - PWM_DEADZONE) {
    straight_pwm = PWM_STRAIGHT_BACK - PWM_VARIATION;  // 후진
  }

  straight_motor.writeMicroseconds(straight_pwm);
}

// --- 조향 제어 함수
void direct_Control() {
  if (!bNewRC1Pulse) {
    direct_motor.writeMicroseconds(PWM_MID - 157); // 중립값 출력
    return;
  }
  bNewRC1Pulse = false;

  // 자동 모드일 경우 보정값 적용
  if (nRC9PulseWidth >= 1500)
    direct_pwm = auto_direct_pwm - 165;
  else
    direct_pwm = nRC1PulseWidth - 157;

  direct_motor.writeMicroseconds(direct_pwm);
}

// --- 시리얼 입력 처리 함수 (Pi로부터 수신)
void processSerialInput() {
  static String inputString = "";
  static unsigned long lastSerialTime = 0;
  const unsigned long serialTimeout = 500; // 500ms 이상 무응답 시 fail-safe

  while (Serial.available()) {
    char inChar = (char)Serial.read();
    lastSerialTime = millis();
    if (inChar == '\n') {
      int commaIndex = inputString.indexOf(',');
      if (commaIndex > 0) {
        auto_straight_pwm = inputString.substring(0, commaIndex).toInt();
        auto_direct_pwm = inputString.substring(commaIndex + 1).toInt();
      }
      inputString = "";
    } else {
      inputString += inChar;
    }
  }

  // 타임아웃: 자동 PWM을 기본값으로 복귀
  if (millis() - lastSerialTime > serialTimeout) {
    auto_straight_pwm = 1500;
    auto_direct_pwm = 1500;
  }
}

// --- 주행 상태 기반 LED 패턴 출력
void updateLEDs() {
  static unsigned long ledTimer = 0;
  static bool ledOn = false;

  switch (drive_state) {
    case 3: // 전진
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
      break;

    case 1: // 좌회전: 왼쪽 점멸
      if (millis() - ledTimer >= 200) {
        ledOn = !ledOn;
        ledTimer = millis();
      }
      digitalWrite(LEFT_LED_PIN, ledOn ? HIGH : LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
      break;

    case 2: // 우회전: 오른쪽 점멸
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

// --- 메인 루프
void loop() {
  static unsigned long lastControlMicros = 0;
  unsigned long now = micros();

  processSerialInput(); // UART 수신 처리

  if (bNewRC9Pulse) bNewRC9Pulse = false; // 자동/수동 상태 갱신

  if (now - lastControlMicros >= 20000) { // 20ms 주기
    straight_Control();
    direct_Control();
    lastControlMicros = now;

    // 현재 주행 상태 파악
    int straight_val = (nRC9PulseWidth >= 1500) ? auto_straight_pwm : nRC2PulseWidth;
    int direct_val   = (nRC9PulseWidth >= 1500) ? auto_direct_pwm   : nRC1PulseWidth;

    if (straight_val < PWM_MID - PWM_DEADZONE)
      drive_state = 0; // 후진
    else if (direct_val < PWM_MID - LR_SEPARATION)
      drive_state = 1; // 좌회전
    else if (direct_val > PWM_MID + LR_SEPARATION)
      drive_state = 2; // 우회전
    else if (straight_val > PWM_MID + PWM_DEADZONE)
      drive_state = 3; // 전진
    else
      drive_state = 3; // 기본 전진

    updateLEDs(); // 방향지시등 상태 갱신
  }
}
```

## 소스 동작원리 - RaspberryPi_lineDetection
이 파이썬 코드는 Raspberry Pi에서 실행되며, **PiCamera2로부터 실시간 영상 프레임을 받아 차선을 검출하고**, 해당 정보를 기반으로 **PID 제어를 수행**한 뒤, **PWM 제어값을 아두이노에 시리얼로 전송**합니다.  
또한, 실시간 주행 상황을 외부에서 확인할 수 있도록 **WebSocket 서버를 통해 JPEG 영상 스트림을 송신**합니다.

### 🔧 전체 구조 요약

| 기능 | 설명 |
|------|------|
| **영상처리** | OpenCV를 활용하여 ROI 내 흰색 라인 검출, 중심점과 기울기 계산 |
| **PID 제어** | 라인 중심 오차(offset)와 기울기(angle)를 결합하여 조향량 계산 |
| **PWM 전송** | 계산된 speed/steer 값을 UART를 통해 아두이노에 전송 |
| **WebSocket** | JPEG 프레임을 base64로 인코딩하여 실시간으로 전송 |
| **라인 상실 대응** | 최근 offset 방향에 따라 조향을 반대로 주며 후진 (0.2초) |

###  상세 동작 흐름
#### 1. 주요 라이브러리 및 하드웨어 초기화

- `cv2`, `numpy` → 영상 처리
- `serial.Serial(...)` → 아두이노와의 UART 통신 초기화
- `Picamera2()` → Pi Camera 실시간 캡처 준비

```py
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"size": (FRAME_WIDTH, FRAME_HEIGHT)}))
cam.start()
```

#### 2. PID 제어 클래스
- compute(error) 함수는 전통적인 PID 제어 계산식을 따름
- Kp (비례), Ki (적분), Kd (미분)의 영향으로 조향을 부드럽게 조절
- angle_error는 rad 단위이므로 100을 곱해 degree-like 영향력을 부여
```py
class PID:
    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative
```

#### 3. extract_lane_features()
- PiCamera 프레임의 하단 영역(100~180px) 을 ROI로 사용
- cv2.threshold(..., THRESH_BINARY_INV)로 검은 바탕에 흰 라인만 강조
- 중심좌표(offset) = ROI 중심과 라인 중심 간 차이
- 기울기(angle) = cv2.HoughLinesP()를 이용한 가장 긴 라인의 각도
```py
offset_error = cx - (binary.shape[1] // 2)
angle_error = -np.arctan2(dy, dx)
```

#### 4. compute_pwm(offset_error, angle_error)
- PID에 전달하는 입력값은 오프셋과 기울기를 각각 가중치(W_OFFSET, W_ANGLE)로 보정한 total_error
- 이 total_error에 따라 조향 PWM은 STEER_CENTER를 기준으로 좌우 이동
- 오차가 클수록 speed는 감소 (회피 목적)
```py
steer = STEER_CENTER - pid.compute(total_error) * STEER_GAIN
speed = SPEED_BASE - min(abs(total_error) * 0.6, SPEED_DROP_MAX)
```

#### 5. stream(websocket): 핵심 제어 루프
1. 프레임 캡처
2. offset과 angle 추출
3. 라인 미검출 시 → last_offset_error 방향 반대로 후진
4. 정상 추적 시 → compute_pwm()로 제어값 계산
5. 시리얼로 \"speed,steer\\n\" 전송
6. 프레임 내 ROI 영역에 현재 오차를 시각화하여 JPEG 인코딩
7. base64 인코딩 후 WebSocket으로 전송
```py
ser.write(f"{speed},{steer}\n".encode())
await websocket.send(jpg_base64)
```
- 라인 미검출 시 좌/우 방향은 last_offset_error의 부호를 따라 후진 시 steer를 달리함
- 후진 시간은 LINE_LOST_REVERSE_DURATION (0.2초)
```py
if last_offset_error < 0:
    steer = STEER_CENTER - LINE_LOST_STEER_DELTA
else:
    steer = STEER_CENTER + LINE_LOST_STEER_DELTA
```

#### 6. main() 및 서버 실행
WebSocket 서버는 8765 포트에서 실행됨
클라이언트는 이 주소로 연결하여 실시간 JPEG 프레임을 받아볼 수 있음
```py
async with websockets.serve(...):
    await asyncio.Future()  # 서버 유지
```

### 주행 시나리오 요약

| 상황           | 판단 기준              | 조치                                |
|----------------|------------------------|--------------------------------------|
| 라인 정상 검출 | `offset_error` 존재     | PID 계산 수행 후 PWM 전송           |
| 라인 상실      | `offset_error is None` | 이전 offset 방향 따라 steer 보정 후 후진 |
| 오차 작음      | `total_error ≈ 0`      | 고속 직진 유지                       |
| 오차 큼        | `total_error` 값 큼    | 조향 크게, 속도 감속                 |
| Web 대시보드   | WebSocket 연결          | JPEG 영상 실시간 시각화 및 디버깅    |

---

## Raspberry Pi ↔ Arduino 통신 프로토콜
| 필드 | 설명                       | 예시         |
|------|---------------------------|--------------|
| `speed` | ESC(전·후진) PWM 값 (µs)  | `1525`, `1460` |
| `steer` | 서보(조향) PWM 값 (µs)   | `1492`, `1800` |
| 구분자   | 쉼표 **`,`**              |              |
| 종료     | 개행 **`\\n`**            |              |

- **전송 주기** : Raspberry Pi가 30 ms 주기로 문자열 전송  
- **패킷 구성** : `"speed,steer\\n"` 형태의 ASCII 문자열  
- **Fail-safe** : 500 ms 이상 무신호 시 Arduino가 PWM을 1500 µs로 복귀


## 라이선스
이 프로젝트는 [MIT License](./LICENSE) 하에 오픈소스로 공개됩니다.