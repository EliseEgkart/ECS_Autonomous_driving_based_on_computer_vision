import asyncio
import cv2
import base64
import serial
import numpy as np
import websockets
from picamera2 import Picamera2

# ───── 설정 ─────
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

FRAME_WIDTH = 320
FRAME_HEIGHT = 240
JPEG_QUALITY = 30

ROI_Y_START = 100
ROI_HEIGHT = 80
THRESHOLD = 140

Kp, Ki, Kd = 2.0, 0.0, 1.5
STEER_CENTER, STEER_MIN, STEER_MAX = 1492, 1068, 1932
STEER_GAIN = 5

SPEED_BASE = 1525
SPEED_DROP_MAX = 10

W_OFFSET = 0.7
W_ANGLE = 0.5

SEND_INTERVAL = 0.03
PING_INTERVAL = 30
PING_TIMEOUT = 20

LINE_LOST_REVERSE_SPEED = 1460
LINE_LOST_REVERSE_DURATION = 0.2
LINE_LOST_STEER_DELTA = 300

# ───── 초기화 ─────
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"size": (FRAME_WIDTH, FRAME_HEIGHT)}))
cam.start()

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

pid = PID(Kp, Ki, Kd)
last_offset_error = 0

# ───── 라인 검출 ─────
def extract_lane_features(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    roi = blur[ROI_Y_START:ROI_Y_START + ROI_HEIGHT, :]
    _, binary = cv2.threshold(roi, THRESHOLD, 255, cv2.THRESH_BINARY_INV)

    offset_error = None
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            offset_error = cx - (binary.shape[1] // 2)

    angle_error = 0.0
    edges = cv2.Canny(binary, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=30, minLineLength=40, maxLineGap=30)
    if lines is not None:
        longest = max(lines, key=lambda l: np.hypot(l[0][2] - l[0][0], l[0][3] - l[0][1]))
        x1, y1, x2, y2 = longest[0]
        dy, dx = y2 - y1, x2 - x1
        angle_error = -np.arctan2(dy, dx)

    return offset_error, angle_error, binary, roi

# ───── PWM 계산 ─────
def compute_pwm(offset_error, angle_error):
    if offset_error is None:
        return LINE_LOST_REVERSE_SPEED, STEER_CENTER

    total_error = W_OFFSET * offset_error + W_ANGLE * (angle_error * 100)
    steer_correction = pid.compute(total_error)
    steer = int(np.clip(STEER_CENTER - steer_correction * STEER_GAIN, STEER_MIN, STEER_MAX))
    speed = int(SPEED_BASE - min(abs(total_error) * 0.6, SPEED_DROP_MAX))
    return speed, steer

# ───── WebSocket 핸들러 ─────
async def stream(websocket):
    global last_offset_error
    try:
        while True:
            frame = cam.capture_array()
            offset_error, angle_error, binary, roi = extract_lane_features(frame)

            if offset_error is None:
                # 반전된 후진 조향 적용
                if last_offset_error < 0:
                    steer = max(STEER_CENTER - LINE_LOST_STEER_DELTA, STEER_MIN)  # 좌측 조향
                else:
                    steer = min(STEER_CENTER + LINE_LOST_STEER_DELTA, STEER_MAX)  # 우측 조향
                speed = LINE_LOST_REVERSE_SPEED
                print(f"[Line Lost] 후진: steer={steer}, based on last offset={last_offset_error}")
                try:
                    ser.write(f"{speed},{steer}\n".encode())
                except Exception as e:
                    print(f"[Serial Error] {e}")
                await asyncio.sleep(LINE_LOST_REVERSE_DURATION)
                continue


            last_offset_error = offset_error
            speed, steer = compute_pwm(offset_error, angle_error)

            cx = offset_error + (FRAME_WIDTH // 2)
            cv2.circle(roi, (cx, roi.shape[0] // 2), 4, (0, 255, 0), -1)
            cv2.putText(roi, f"Speed: {speed}  Steer: {steer}", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            roi_resized = cv2.cvtColor(roi, cv2.COLOR_RGB2BGRA)
            frame[ROI_Y_START:ROI_Y_START + ROI_HEIGHT, :] = roi_resized

            try:
                ser.write(f"{speed},{steer}\n".encode())
            except Exception as e:
                print(f"[Serial Error] {e}")

            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            jpg_base64 = base64.b64encode(buffer).decode('utf-8')
            await websocket.send(jpg_base64)

            await asyncio.sleep(SEND_INTERVAL)

    except websockets.exceptions.ConnectionClosedError:
        print("[WebSocket] 연결 종료됨.")
    except Exception as e:
        print(f"[WebSocket] 예외 발생: {e}")

# ───── WebSocket 서버 실행 ─────
async def main():
    async with websockets.serve(
        stream, "0.0.0.0", 8765,
        ping_interval=PING_INTERVAL,
        ping_timeout=PING_TIMEOUT
    ):
        print("WebSocket 서버 실행 중 on ws://<라즈베리파이_IP>:8765")
        await asyncio.Future()

asyncio.run(main())
