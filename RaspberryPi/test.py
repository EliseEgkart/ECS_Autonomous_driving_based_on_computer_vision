#!/usr/bin/env python3
import cv2
import numpy as np
import time
import serial
from picamera2 import Picamera2

# 전역 변수
selected_line = None  # (x1, y1, x2, y2)
FRAME_WIDTH  = 640

# ────────── 시리얼 설정 ──────────
# Arduino 쪽 시리얼 포트, 보드레이트는 실제 연결 환경에 맞춰 수정하세요.
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # Arduino reset 안정화

def process_frame(frame):
    global selected_line

    # 1) 그레이스케일 + 이진화 + 모폴로지 열기
    gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, th  = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
    kernel = np.ones((3,3), np.uint8)
    opened = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel, iterations=1)

    # 2) Canny → Probabilistic Hough
    edges = cv2.Canny(opened, 30, 100)
    lines = cv2.HoughLinesP(edges,
                            rho=1,
                            theta=np.pi/180,
                            threshold=30,
                            minLineLength=50,
                            maxLineGap=30)

    # 3) 가장 긴 선분 찾기
    best = None
    max_len = 0
    if lines is not None:
        for x1,y1,x2,y2 in lines[:,0]:
            L = np.hypot(x2-x1, y2-y1)
            if L > max_len:
                max_len = L
                best = (x1,y1,x2,y2)
    selected_line = best

    # 4) 결과 그리기
    out = frame.copy()
    if best:
        x1,y1,x2,y2 = best
        cv2.line(out, (x1,y1), (x2,y2), (0,0,255), 2)
    return out

def control_via_serial(selected_line):
    """
    selected_line: (x1,y1,x2,y2) or None
    전·후진 PWM: 1000~1500 후진, 1500~2000 전진
    조향 PWM       : 1000~1500 우회전, 1500~2000 좌회전
    """
    if selected_line is None:
        drive = 1500
        steer = 1500
    else:
        x1,y1,x2,y2 = selected_line
        center_x = (x1 + x2) / 2.0
        error = (center_x - FRAME_WIDTH/2) / (FRAME_WIDTH/2)  # -1..+1

        # 조향
        steer = int(1500 - error * 500)  

        # 속도: error 크기에 반비례
        speed_factor = max(0.0, 1 - abs(error))
        drive = int(1520 + speed_factor * 480)

    msg = f"{drive},{steer}\n"
    ser.write(msg.encode())
    # 디버깅용
    print(f"Sent -> drive: {drive}, steer: {steer}")

def main():
    # 1) Picamera2 설정
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format":"XRGB8888", "size":(FRAME_WIDTH, 480)}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)  # 워밍업

    # 2) 창 생성
    cv2.namedWindow("Line Detection", cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            # 3) 프레임 캡처
            raw = picam2.capture_array()                            # XRGB8888
            frame = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)            # BGR로 변환

            # 4) 처리 + 선 그리기
            out = process_frame(frame)

            # 5) 시리얼 제어
            control_via_serial(selected_line)

            # 6) 화면 표시
            cv2.imshow("Line Detection", out)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        ser.close()

if __name__ == "__main__":
    main()
