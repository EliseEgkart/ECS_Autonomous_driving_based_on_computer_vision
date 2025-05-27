#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2                     # OpenCV: 이미지 처리 라이브러리
import numpy as np             # NumPy: 수치 연산 및 배열 처리 라이브러리
import time                    # 시간 지연 및 타이밍 제어
import serial                  # pySerial: 시리얼 통신
from picamera2 import Picamera2  # Picamera2: Raspberry Pi 카메라 제어

# ────────────── 시스템 설정 ──────────────
SERIAL_PORT    = '/dev/ttyUSB0'  # Arduino 연결 시리얼 포트 (환경에 맞게 수정)
BAUD_RATE      = 9600            # 시리얼 통신 속도 (bps)
PWM_MIN, PWM_MID, PWM_MAX = 1068, 1492, 1932  # 서보(ESC) PWM 최소·중간·최대 값 (µs)

# 영상 처리 영역(ROI) 및 이진화 임계치 설정
ROI_Y_START    = 180   # 전체 프레임에서 관심 영역(ROI) 시작 Y 좌표
ROI_HEIGHT     = 60    # ROI 높이 (픽셀)
THRESHOLD      = 60    # 그레이스케일 임계값 (이진화 시) 

# 카메라 해상도 정의 (Picamera2 설정)
FRAME_WIDTH    = 320   # 가로 해상도
FRAME_HEIGHT   = 240   # 세로 해상도

# ────────────── 시리얼 통신 초기화 ──────────────
# Arduino 측이 부팅 및 리셋된 후 안정화 시간을 두기 위해 소량의 지연을 추가
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Arduino 리셋 및 통신 채널 준비 시간

# ────────────── PID 제어기 클래스 정의 ──────────────
class PID:
    """
    PID 제어기: 비례(Proportional), 적분(Integral), 미분(Derivative) 제어 구현
    """
    def __init__(self, Kp, Ki, Kd):
        # 제어 이득 설정
        self.Kp = Kp  # 비례 이득: 현재 오차 비례
        self.Ki = Ki  # 적분 이득: 과거 오차 누적 보상
        self.Kd = Kd  # 미분 이득: 오차 변화율 기반 보상
        
        # 내부 상태 변수 초기화
        self.prev_error = 0.0  # 이전 오차 값 (미분 계산용)
        self.integral   = 0.0  # 오차 누적 합 (적분 계산용)

    def compute(self, error):
        """
        error: 현재 오차 값 (목표 위치 - 실제 위치)
        반환값: 제어 입력 값 (PID 출력)
        """
        # 1. 적분항 업데이트: 오차를 계속 누적
        self.integral += error
        
        # 2. 미분항 계산: 현재 오차와 이전 오차의 차이
        derivative    = error - self.prev_error
        
        # 3. 이전 오차 갱신
        self.prev_error = error
        
        # 4. PID 합산: P + I + D
        output = (
            self.Kp * error +        # 비례항
            self.Ki * self.integral +# 적분항
            self.Kd * derivative     # 미분항
        )
        return output

    def reset(self):
        """
        PID 내부 상태 초기화: 오차 적분 및 이전 오차 값을 리셋
        """
        self.prev_error = 0.0
        self.integral   = 0.0

# ────────────── 제어 보조 함수 ──────────────
def map_steering_to_pwm(control_val, scale=50):
    """
    PID 제어 결과(control_val)를 서보/ESC PWM 범위로 매핑
    - control_val: PID 출력값 (e.g., -scale..+scale 범위 예상)
    - scale: control_val 클리핑(제한) 임계치
    """
    # 1. 제어값 클리핑: 음수/양수 과도한 진동 방지
    clipped = np.clip(control_val, -scale, scale)
    
    # 2. 중립 PWM 기준으로 제어값 반영
    pwm     = int(PWM_MID + clipped)
    
    # 3. 하드웨어 안전 범위 내로 제한
    pwm     = max(min(pwm, PWM_MAX), PWM_MIN)
    return pwm


def send_control_by_pid(error, fixed_throttle=1545):
    """
    PID 제어 오차(error) → steering PWM 계산 → 시리얼로 Arduino 전송
    - error: 혼합 오차(중심 오프셋 + 각도 오프셋) 입력
    - fixed_throttle: 전진 속도용 PWM (고정)
    """
    # 1. PID 연산 수행
    control_val   = pid.compute(error)
    
    # 2. 조향 PWM 매핑
    direct_pwm    = map_steering_to_pwm(control_val)
    
    # 3. 전진 PWM 고정
    straight_pwm  = fixed_throttle
    
    # 4. Arduino 호환 포맷으로 문자열 구성
    packet = f"{straight_pwm},{direct_pwm}\n"
    
    # 5. 시리얼 전송
    ser.write(packet.encode())
    
    # 6. 디버깅 로그 출력
    print(f"[SERIAL] Sent → Straight: {straight_pwm}, Direct: {direct_pwm} "
          f"(Err: {error:.2f}, PID: {control_val:.2f})")

# ────────────── PID 객체 생성 및 파라미터 설정 ──────────────
pid = PID(Kp=0.6, Ki=0.0, Kd=0.15)  # 경험적 값, 튜닝 필요

# ────────────── 메인 루프 ──────────────
def main():
    # 1. Picamera2 설정 및 시작
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1)  # 카메라 워밍업

    while True:
        # 2. 프레임 캡처 (RGB888 배열)
        frame = picam2.capture_array()
        # 3. 관심 영역(ROI) 추출
        roi   = frame[ROI_Y_START:ROI_Y_START+ROI_HEIGHT, :]

        # --- 영상 전처리 ---
        # 4. 그레이스케일 변환
        gray   = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        # 5. 임계값 이진화 (검은 선 강조)
        _, binary = cv2.threshold(gray, THRESHOLD, 255, cv2.THRESH_BINARY_INV)

        # --- ROI 기반 중심 오프셋 계산 ---
        offset_error = None
        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if contours:
            # 가장 큰 컨투어(라인) 선택
            largest = max(contours, key=cv2.contourArea)
            M       = cv2.moments(largest)
            if M["m00"] > 0:
                # 중심 좌표 계산
                cx           = int(M["m10"] / M["m00"] )
                # 화면 중심 대비 오프셋
                offset_error = cx - (binary.shape[1] // 2)
                # 디버그: 중심점 표시
                cv2.circle(roi, (cx, ROI_HEIGHT // 2), 5, (0,255,0), -1)

        # --- Hough 기반 방향 오프셋 계산 ---
        angle_error = 0.0
        edges       = cv2.Canny(binary, 50, 150)  # 에지 검출
        lines       = cv2.HoughLinesP(
            edges, 1, np.pi/180, threshold=30,
            minLineLength=40, maxLineGap=30
        )
        if lines is not None:
            # 가장 긴 선분 선택
            longest = max(
                lines,
                key=lambda l: np.hypot(
                    l[0][2]-l[0][0], l[0][3]-l[0][1]
                )
            )
            x1,y1,x2,y2 = longest[0]
            dy          = y2 - y1
            dx          = x2 - x1
            # 선 기울기를 라디안으로 변환
            angle_error = np.arctan2(dy, dx)
            # 디버그: 선분 표시
            cv2.line(roi, (x1,y1), (x2,y2), (255,0,0), 2)

        # --- 혼합 오차 계산 및 제어 호출 ---
        if offset_error is not None:
            # 중심 오프셋과 각도 오프셋을 가중 합
            total_error = 0.7 * offset_error + 0.3 * angle_error * 100
            send_control_by_pid(total_error)

        # --- 디버그 화면 표시 ---
        cv2.imshow("ROI", roi)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 정리: 카메라 및 창 종료, 시리얼 포트 닫기
    picam2.stop()
    cv2.destroyAllWindows()
    ser.close()

if __name__ == "__main__":
    main()
