# 임베디드 통신시스템 프로젝트 - 라인 인식(Cam)기반 자율주행 프로젝트

## 팀 구성원 및 역할분담
| 이름 | 담당 업무 |
|------|-----------|
| **김수민** | 아두이노 : RC 신호 해석, 서보모터 · ESC 제어 로직 |
| **김형진** | 라즈베리파이 : 영상 처리, 라인검출 기반 조향 제어, 웹 스트리밍 |
---
## 프로젝트 개요
본 프로젝트는 Raspberry Pi와 Arduino를 이용해 라인 인식 기반 자율주행 RC카를 구현한 임베디드 통신시스템 프로젝트입니다.  
카메라 기반 영상 처리와 PID 알고리즘을 통해 차선을 추종하며 주행하며, RC 조종기 또는 자동 제어로 전환 가능한 이중 제어 시스템을 구성합니다.

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

-------------------------------------------------------------------------------------------

## 소스 동작원리 - ECS_Project_Arduino_manual
이 코드는 RC 조종기의 PWM 신호를 실시간으로 수신하여 **직진/후진용 ESC**와 **조향용 서보모터**를 제어하고, **방향지시등 LED를 자동 점멸**하는 시스템입니다.

### 전체 구조 요약

| 기능             | 설명 |
|------------------|------|
| **PWM 입력**     | RC 수신기에서 CH2(직진/후진), CH1(조향) PWM 수신 |
| **PWM 디코딩**   | `PinChangeInterrupt` 라이브러리로 마이크로초 단위 펄스폭 측정 |
| **PWM 출력**     | `Servo` 라이브러리 사용 – ESC와 조향 서보 제어 |
| **상태 판단**    | 입력 PWM을 기반으로 주행 상태(`drive_state`) 계산 |
| **LED 제어**     | 주행 상태에 따라 좌우 방향지시등 점멸 또는 상시 점등 |

---

### 동작 흐름 요약

#### 1. 핀 및 초기값 정의

- `A0`, `A1` → RC PWM 입력 (CH2, CH1)
- `7`, `8` → 방향지시등 (좌/우)
- `9`, `10` → 제어 출력 핀 (ESC, 서보)
- `PWM_MID` 기준으로 정지/전진/후진 판별

#### 2. 인터럽트 기반 PWM 측정

- `PinChangeInterrupt` 라이브러리를 사용하여 핀의 HIGH/LOW 변화를 감지
- 상승엣지 시간 `micros()`로 기록, 하강엣지에서 폭 계산

```cpp
if (digitalRead(pinRC2_straight) == HIGH)
    uRC2StartHigh = micros(); // 상승엣지
else
    nRC2PulseWidth = micros() - uRC2StartHigh;
```

#### 3. PWM 해석 및 제어
- 데드존 ±20 µs 내에서는 정지
- 전진/후진 기준값은 PWM_STRAIGHT_FRONT, PWM_STRAIGHT_BACK
- PWM_VARIATION만큼 조정
```cpp
if (selected_pwm > PWM_MID + DEADZONE)
    straight_pwm = PWM_STRAIGHT_FRONT + PWM_VARIATION;
```
- 조향은 그대로 서보에 전달 (다만 보정값 -157 적용)
```cpp
direct_pwm = nRC1PulseWidth - 157;
direct_motor.writeMicroseconds(direct_pwm);
```
#### 4. 주행 상태 판단 및 LED 제어

- `drive_state`는 아래 기준으로 설정됨:

| 상태코드 | 의미     | 조건                           |
|----------|----------|--------------------------------|
| 0        | 후진     | CH2 < PWM_MID - PWM_DEADZONE   |
| 1        | 좌회전   | CH1 < PWM_MID - LR_SEPARATION  |
| 2        | 우회전   | CH1 > PWM_MID + LR_SEPARATION  |
| 3        | 전진     | CH2 > PWM_MID + PWM_DEADZONE   |

- LED 출력 방식:

| 상태       | 좌측 LED | 우측 LED | 설명                      |
|------------|----------|----------|---------------------------|
| 전진       | OFF      | OFF      | 방향지시등 끔             |
| 좌회전     | 깜빡임   | OFF      | 좌측 깜빡이 동작          |
| 우회전     | OFF      | 깜빡임   | 우측 깜빡이 동작          |
| 후진       | ON       | ON       | 양쪽 방향지시등 상시 점등 |

- 깜빡임 구현 방식:
  - `millis()`를 이용한 소프트웨어 타이머 방식
  - 200ms 주기로 ON/OFF 전환

```cpp
if (millis() - ledTimer >= 200) {
    ledOn = !ledOn;
    ledTimer = millis();
}
```
-------------------------------------------------------------------------------------------

## 소스 동작원리 - ECS_Project_Arduino_Auto
이 코드는 RC 수신기의 PWM 신호뿐만 아니라 **Raspberry Pi로부터 UART로 수신되는 제어 명령**을 기반으로 하는 **자동/수동 겸용 제어 시스템**입니다.  
RC 송신기의 CH9 스위치를 기준으로 두 모드 간 전환이 가능하며, 각각의 입력에 따라 **직진/조향 제어 및 방향지시등 동작을 구현**합니다.

| 모드 구분      | 판단 기준           | 제어 입력 원천                 | 제어 대상                |
|----------------|---------------------|-------------------------------|---------------------------|
| **수동 모드**   | CH9 < 1500µs         | RC 송신기 CH1 (조향), CH2 (속도) | 직접 서보/ESC 제어       |
| **자동 모드**   | CH9 ≥ 1500µs         | Raspberry Pi → Serial `"속도,조향"` | Pi로부터 전송된 제어값 사용 |

### 기능별 구조 요약

| 기능 구분     | 설명 |
|---------------|------|
| **PWM 입력**   | CH1(조향), CH2(속도), CH9(모드 전환) 입력 PWM 측정 (PinChangeInterrupt) |
| **자동 제어 수신** | Raspberry Pi로부터 `"speed,steer\n"` 문자열을 수신하여 PWM 값 파싱 |
| **직진/조향 제어** | 모드에 따라 입력 PWM 선택 → Servo 라이브러리로 서보/ESC 제어 |
| **방향지시등** | 현재 주행 상태(`drive_state`)에 따라 LED 점등/점멸 패턴 구현 |
| **Fail-safe** | Serial 수신이 0.5초 이상 없을 시 PWM을 기본값(1500)으로 복원 |

### 제어 흐름 개요

#### 1. 입력 PWM 처리
- 각 채널마다 PinChange 인터럽트를 등록하여 펄스 폭을 측정
- CH1: 좌우 조향, CH2: 직진/후진, CH9: 자동/수동 모드 전환

#### 2. 자동 제어 입력 (Serial 통신)
- Raspberry Pi로부터 `"1520,1465\n"` 형식의 문자열 수신
- comma 기준으로 `auto_straight_pwm`, `auto_direct_pwm`으로 분리
- 500ms 이상 수신 없을 경우 자동값을 기본값(1500)으로 복귀

#### 3. PWM 제어
- CH9 값이 기준(1500) 이상이면 자동 제어 사용
- 수동이면 RC 송신기의 CH1, CH2에서 직접 받은 PWM 사용
- 직진은 ESC에, 조향은 서보모터에 연결됨

#### 4. 주행 상태 판단 및 방향지시등 출력

| 상태       | 조건                                         | LED 동작                       |
|------------|----------------------------------------------|--------------------------------|
| 후진       | 직진 PWM < PWM_MID - DEADZONE               | 양쪽 LED 항상 ON              |
| 좌회전     | 조향 PWM < PWM_MID - LR_SEPARATION          | 좌측 LED 깜빡임               |
| 우회전     | 조향 PWM > PWM_MID + LR_SEPARATION          | 우측 LED 깜빡임               |
| 전진       | 직진 PWM > PWM_MID + DEADZONE               | LED OFF                       |
| 정지 또는 중립 | Deadzone 내 또는 예외 처리                  | LED OFF                       |

- 깜빡임 구현은 `millis()` 기반 200ms 간격 ON/OFF 토글

---

### 메인 루프 흐름

1. `processSerialInput()` 실행 → 자동 제어값 수신
2. 최신 CH9 상태 갱신 (모드 전환 판단)
3. 20ms마다:
   - `straight_Control()` 호출 → 직진 PWM 결정
   - `direct_Control()` 호출 → 조향 PWM 결정
   - 현재 주행 상태(`drive_state`) 판단
   - `updateLEDs()`로 방향지시등 상태 갱신

```cpp
// 예: 자동 모드 조건문
int selected_pwm = (nRC9PulseWidth >= 1500) ? auto_straight_pwm : nRC2PulseWidth;
```
- 모든 PWM 출력은 Servo 라이브러리를 통해 마이크로초 단위로 제어됨 (writeMicroseconds())

-------------------------------------------------------------------------------------------

## 소스 동작원리 - RaspberryPi_lineDetection
이 파이썬 코드는 Raspberry Pi에서 실행되며, **PiCamera2로부터 실시간 영상 프레임을 받아 차선을 검출하고**, 해당 정보를 기반으로 **PID 제어를 수행**한 뒤, **PWM 제어값을 아두이노에 시리얼로 전송**합니다.  
또한, 실시간 주행 상황을 외부에서 확인할 수 있도록 **WebSocket 서버를 통해 JPEG 영상 스트림을 송신**합니다.

### 전체 구조 요약

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