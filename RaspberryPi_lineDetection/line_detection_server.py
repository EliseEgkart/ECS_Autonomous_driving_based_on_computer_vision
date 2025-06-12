import asyncio                # 비동기 처리용 (WebSocket 및 sleep)
import cv2                    # OpenCV – 영상 처리
import base64                 # 영상 데이터를 base64 인코딩 (웹 전송용)
import serial                 # 시리얼 통신용 (아두이노 제어)
import numpy as np            # 수치 연산 및 행렬 처리
import websockets             # WebSocket 서버 구현용
from picamera2 import Picamera2  # 라즈베리파이용 PiCamera2 제어 라이브러리

# ───── 하드웨어 및 제어 관련 설정값들 ─────
SERIAL_PORT = '/dev/ttyACM0'     # 아두이노 또는 시리얼 장치 포트
BAUD_RATE = 9600                 # 시리얼 통신 속도

FRAME_WIDTH = 320                # 영상 프레임 너비
FRAME_HEIGHT = 240               # 영상 프레임 높이
JPEG_QUALITY = 30                # JPEG 압축 품질 (낮을수록 데이터 작음)

ROI_Y_START = 100                # ROI(관심영역)의 y축 시작 위치
ROI_HEIGHT = 80                  # ROI의 높이 (라인 검출용)

THRESHOLD = 140                  # 이진화 임계값 (선 검출용 흑백 분리 기준)

# PID 제어 상수
Kp, Ki, Kd = 2.0, 0.0, 1.5       # 비례, 적분, 미분 계수

# 조향값(PWM) 범위 및 중앙값 설정
STEER_CENTER, STEER_MIN, STEER_MAX = 1492, 1068, 1932
STEER_GAIN = 5                   # 조향 보정값에 곱해지는 비율

SPEED_BASE = 1525                # 기본 주행 속도
SPEED_DROP_MAX = 10              # 회피/곡선 상황 시 속도 저하 한계

# 오차 가중치 (좌우 오프셋 vs 각도)
W_OFFSET = 0.7                   # 라인 중심으로부터의 오프셋 가중치
W_ANGLE = 0.5                    # 라인 각도 오차의 가중치

SEND_INTERVAL = 0.03             # 영상 프레임 전송 간격 (초 단위)
PING_INTERVAL = 30               # WebSocket ping 전송 간격
PING_TIMEOUT = 20                # ping 응답 없을 경우 타임아웃

# 라인 미검출 시 후진 및 조향 설정
LINE_LOST_REVERSE_SPEED = 1460           # 후진 속도 PWM
LINE_LOST_REVERSE_DURATION = 0.2         # 후진 지속 시간 (초)
LINE_LOST_STEER_DELTA = 300              # 후진 시 조향 보정량

# ───── 시리얼 및 카메라 초기화 ─────
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)  # 시리얼 포트 열기
cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"size": (FRAME_WIDTH, FRAME_HEIGHT)}))  # 해상도 설정
cam.start()  # 카메라 스트리밍 시작

# ───── PID 제어 클래스 정의 ─────
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error                        # 적분 계산
        derivative = error - self.prev_error          # 미분 계산
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative  # PID 계산식

pid = PID(Kp, Ki, Kd)
last_offset_error = 0                                # 라인 상실 시 사용될 마지막 offset 값

# ───── 라인 검출 함수 정의 ─────
def extract_lane_features(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 흑백 변환
    blur = cv2.GaussianBlur(gray, (5, 5), 0)        # 노이즈 제거용 블러
    roi = blur[ROI_Y_START:ROI_Y_START + ROI_HEIGHT, :]  # ROI 추출
    _, binary = cv2.threshold(roi, THRESHOLD, 255, cv2.THRESH_BINARY_INV)  # 이진화 (선만 추출)

    offset_error = None
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 외곽선 탐지
    if contours:
        largest = max(contours, key=cv2.contourArea)  # 가장 큰 외곽선 (주행선)
        M = cv2.moments(largest)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])  # 중심점 x좌표
            offset_error = cx - (binary.shape[1] // 2)  # 화면 중심으로부터의 오차

    # 라인의 기울기(각도)를 구함
    angle_error = 0.0
    edges = cv2.Canny(binary, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=30, minLineLength=40, maxLineGap=30)
    if lines is not None:
        longest = max(lines, key=lambda l: np.hypot(l[0][2] - l[0][0], l[0][3] - l[0][1]))
        x1, y1, x2, y2 = longest[0]
        dy, dx = y2 - y1, x2 - x1
        angle_error = -np.arctan2(dy, dx)  # 각도 오차 (기울기)

    return offset_error, angle_error, binary, roi

# ───── PWM 계산 함수 ─────
def compute_pwm(offset_error, angle_error):
    if offset_error is None:
        return LINE_LOST_REVERSE_SPEED, STEER_CENTER  # 라인 미검출 시 후진

    # 총 오차 계산 (오프셋 + 각도)
    total_error = W_OFFSET * offset_error + W_ANGLE * (angle_error * 100)
    steer_correction = pid.compute(total_error)  # PID 제어
    steer = int(np.clip(STEER_CENTER - steer_correction * STEER_GAIN, STEER_MIN, STEER_MAX))  # 조향 PWM 계산
    speed = int(SPEED_BASE - min(abs(total_error) * 0.6, SPEED_DROP_MAX))  # 오차 크기에 따라 속도 감소
    return speed, steer

# ───── WebSocket 핸들러 정의 ─────
async def stream(websocket):
    global last_offset_error
    try:
        while True:
            frame = cam.capture_array()  # 프레임 캡처
            offset_error, angle_error, binary, roi = extract_lane_features(frame)  # 라인 정보 추출

            if offset_error is None:
                # 라인 검출 실패 → 후진 및 방향 조정
                if last_offset_error < 0:
                    steer = max(STEER_CENTER - LINE_LOST_STEER_DELTA, STEER_MIN)  # 좌측
                else:
                    steer = min(STEER_CENTER + LINE_LOST_STEER_DELTA, STEER_MAX)  # 우측
                speed = LINE_LOST_REVERSE_SPEED
                print(f"[Line Lost] 후진: steer={steer}, based on last offset={last_offset_error}")
                try:
                    ser.write(f"{speed},{steer}\n".encode())  # 시리얼 전송
                except Exception as e:
                    print(f"[Serial Error] {e}")
                await asyncio.sleep(LINE_LOST_REVERSE_DURATION)
                continue

            # 정상 추적 시 PWM 계산 및 전송
            last_offset_error = offset_error
            speed, steer = compute_pwm(offset_error, angle_error)

            # ROI 상에 현재 중심점을 시각화
            cx = offset_error + (FRAME_WIDTH // 2)
            cv2.circle(roi, (cx, roi.shape[0] // 2), 4, (0, 255, 0), -1)
            cv2.putText(roi, f"Speed: {speed}  Steer: {steer}", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # ROI 삽입 (색상 변환)
            roi_resized = cv2.cvtColor(roi, cv2.COLOR_RGB2BGRA)
            frame[ROI_Y_START:ROI_Y_START + ROI_HEIGHT, :] = roi_resized

            # 시리얼로 PWM 값 전송
            try:
                ser.write(f"{speed},{steer}\n".encode())
            except Exception as e:
                print(f"[Serial Error] {e}")

            # JPEG 인코딩 및 base64 전송
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            jpg_base64 = base64.b64encode(buffer).decode('utf-8')
            await websocket.send(jpg_base64)

            await asyncio.sleep(SEND_INTERVAL)

    except websockets.exceptions.ConnectionClosedError:
        print("[WebSocket] 연결 종료됨.")
    except Exception as e:
        print(f"[WebSocket] 예외 발생: {e}")

# ───── WebSocket 서버 실행 함수 ─────
async def main():
    async with websockets.serve(
        stream, "0.0.0.0", 8765,
        ping_interval=PING_INTERVAL,
        ping_timeout=PING_TIMEOUT
    ):
        print("WebSocket 서버 실행 중 on ws://<라즈베리파이_IP>:8765")
        await asyncio.Future()  # 무한 대기

# ───── 메인 루프 실행 ─────
asyncio.run(main())
