#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import cv2
import numpy as np
import time
import serial
from picamera2 import Picamera2
from PyQt5 import QtWidgets, QtCore, QtGui

# ────────────── 시스템 설정 ──────────────
SERIAL_PORT    = '/dev/ttyUSB0'   # Arduino 연결 시리얼 포트
BAUD_RATE      = 9600             # 시리얼 통신 속도 (bps)
PWM_MIN, PWM_MID, PWM_MAX = 1068, 1492, 1932  # 서보 PWM 범위 (µs)
FRAME_WIDTH    = 320              # 카메라 가로 해상도
FRAME_HEIGHT   = 240              # 카메라 세로 해상도
ROI_Y_START    = 180              # ROI 시작 Y 좌표
ROI_HEIGHT     = 60               # ROI 높이 (픽셀)

# ────────────── PID 제어기 클래스 ──────────────
class PID:
    def __init__(self, Kp=0.6, Ki=0.0, Kd=0.15):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral   = 0.0

    def compute(self, error):
        # 적분항 누적
        self.integral += error
        # 미분항 계산
        derivative     = error - self.prev_error
        self.prev_error = error
        # PID 연산
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

    def reset(self):
        self.prev_error = 0.0
        self.integral   = 0.0

# ────────────── 비전 처리 스레드 ──────────────
class VisionThread(QtCore.QThread):
    frameProcessed = QtCore.pyqtSignal(np.ndarray)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.running = False
        # Picamera2 설정
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
        )
        self.picam2.configure(config)
        # 시리얼 초기화
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)

    def run(self):
        self.running = True
        self.picam2.start()
        pid = PID()
        while self.running:
            frame = self.picam2.capture_array()
            roi   = frame[ROI_Y_START:ROI_Y_START+ROI_HEIGHT, :]

            # 그레이스케일 및 이진화
            gray      = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
            thresh    = int(self.parent.threshold_slider.value())
            _, binary = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY_INV)

            # ROI 기반 중심 오프셋
            offset_error = None
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                M       = cv2.moments(largest)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    offset_error = cx - (binary.shape[1] // 2)
                    cv2.circle(roi, (cx, ROI_HEIGHT // 2), 5, (0, 255, 0), -1)

            # Hough 기반 방향 오프셋
            angle_error = 0.0
            edges       = cv2.Canny(binary, 50, 150)
            lines       = cv2.HoughLinesP(
                edges, 1, np.pi/180,
                threshold=self.parent.hough_thresh_slider.value(),
                minLineLength=40, maxLineGap=30
            )
            if lines is not None:
                longest = max(lines, key=lambda l: np.hypot(l[0][2]-l[0][0], l[0][3]-l[0][1]))
                x1,y1,x2,y2 = longest[0]
                dy,dx       = y2 - y1, x2 - x1
                angle_error = np.arctan2(dy, dx)
                cv2.line(roi, (x1, y1), (x2, y2), (255, 0, 0), 2)

            # 혼합 오차 계산 및 제어
            if offset_error is not None:
                w_off = self.parent.offset_weight_slider.value() / 100.0
                w_ang = self.parent.angle_weight_slider.value() / 100.0
                total_error = w_off * offset_error + w_ang * (angle_error * 100)

                # PID 계수 갱신
                pid.Kp = self.parent.kp_slider.value() / 100.0
                pid.Ki = self.parent.ki_slider.value() / 100.0
                pid.Kd = self.parent.kd_slider.value() / 100.0

                control = pid.compute(total_error)
                scale   = self.parent.scale_slider.value()
                clipped = np.clip(control, -scale, scale)
                direct_pwm   = int(PWM_MID + clipped)
                direct_pwm   = max(min(direct_pwm, PWM_MAX), PWM_MIN)
                straight_pwm = self.parent.throttle_slider.value()

                # 시리얼 전송
                packet = f"{straight_pwm},{direct_pwm}\n"
                self.ser.write(packet.encode())

            # QImage로 변경하여 GUI로 전송
            display = cv2.cvtColor(roi, cv2.COLOR_RGB2BGR)
            self.frameProcessed.emit(display)

        self.picam2.stop()
        self.ser.close()

# ────────────── 메인 GUI ──────────────
class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('라인 트레이싱 튜너')
        layout = QtWidgets.QGridLayout(self)

        # 슬라이더 및 라벨 정의
        self.kp_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.kp_slider.setRange(0, 200); self.kp_slider.setValue(60)
        self.ki_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.ki_slider.setRange(0, 100); self.ki_slider.setValue(0)
        self.kd_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.kd_slider.setRange(0, 200); self.kd_slider.setValue(15)
        self.threshold_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.threshold_slider.setRange(1, 255); self.threshold_slider.setValue(60)
        self.hough_thresh_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.hough_thresh_slider.setRange(1, 100); self.hough_thresh_slider.setValue(30)
        self.offset_weight_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.offset_weight_slider.setRange(0, 100); self.offset_weight_slider.setValue(70)
        self.angle_weight_slider  = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.angle_weight_slider.setRange(0, 100); self.angle_weight_slider.setValue(30)
        self.scale_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.scale_slider.setRange(1, 200); self.scale_slider.setValue(50)
        self.throttle_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.throttle_slider.setRange(PWM_MIN, PWM_MAX); self.throttle_slider.setValue(1545)

        labels = ['Kp','Ki','Kd','Threshold','HoughThresh','OffWeight','AngWeight','Scale','Throttle']
        widgets = [self.kp_slider, self.ki_slider, self.kd_slider,
                   self.threshold_slider, self.hough_thresh_slider,
                   self.offset_weight_slider, self.angle_weight_slider,
                   self.scale_slider, self.throttle_slider]

        for i, (lab, wid) in enumerate(zip(labels, widgets)):
            layout.addWidget(QtWidgets.QLabel(lab), i, 0)
            layout.addWidget(wid, i, 1)

        # 영상 표시용 QLabel
        self.video_label = QtWidgets.QLabel()
        layout.addWidget(self.video_label, 0, 2, len(labels), 1)

        # Start/Stop 버튼
        self.start_btn = QtWidgets.QPushButton('Start')
        self.start_btn.clicked.connect(self.toggle_thread)
        layout.addWidget(self.start_btn, len(labels), 0, 1, 2)

        # VisionThread 생성 및 시그널 연결
        self.thread = VisionThread(self)
        self.thread.frameProcessed.connect(self.update_image)

    def toggle_thread(self):
        if not self.thread.running:
            self.thread.start()
            self.start_btn.setText('Stop')
        else:
            self.thread.running = False
            self.thread.wait()
            self.start_btn.setText('Start')

    def update_image(self, img):
        h, w = img.shape[:2]
        qimg = QtGui.QImage(img.data, w, h, w*3, QtGui.QImage.Format_BGR888)
        self.video_label.setPixmap(QtGui.QPixmap.fromImage(qimg))

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())

    # ================= 조정 가이드 (PyQt 실시간 슬라이더 파라미터) =================
#
# 1. Kp (비례 이득) — 범위: 0.00 ~ 2.00
#    • 조향 응답의 민감도를 결정합니다.
#    • 시작: 0.5로 설정 → 차선 중앙 오차에 비례해 스티어링이 즉시 반응.
#    • 과도: 값이 너무 크면 진동(oscillation) 발생 → 서서히 감소시키며 안정화 확인.
#
# 2. Ki (적분 이득) — 범위: 0.00 ~ 1.00
#    • 지속적인 오차(편향)를 보상합니다.
#    • 보통 0으로 시작 → Kp/Kd가 안정화된 뒤 조금씩 증가.
#    • 너무 크면 누적 과도 발생 → “Integral windup” 주의.
#
# 3. Kd (미분 이득) — 범위: 0.00 ~ 2.00
#    • 오차 변화율에 반응해 진동을 억제합니다.
#    • Kp만으로 진동이 심할 때 소량 추가 → 과도하면 제어가 둔해짐.
#
# 4. Threshold (이진화 임계값) — 범위: 1 ~ 255
#    • ROI 영역을 흑백으로 변환할 때 경계값을 설정합니다.
#    • 조도에 따라 최적값이 다름 → 어두운 환경에서는 낮추고, 밝은 환경에서는 높임.
#
# 5. HoughThresh (허프 선 검출 임계값) — 범위: 1 ~ 100
#    • 선 검출을 위한 최소 교차점 수를 조정합니다.
#    • 너무 높으면 선 검출 실패 → 낮춰서 민감도↑, 너무 낮으면 잡음 검출↑.
#
# 6. OffWeight (중심 오프셋 가중치) — 범위: 0% ~ 100%
#    • 중심 오프셋(error_offset)에 부여하는 비중을 설정합니다.
#    • 곡선이 많을 때는 낮추고, 직선이 많을 때는 높여 안정성↑.
#
# 7. AngWeight (각도 오프셋 가중치) — 범위: 0% ~ 100%
#    • Hough 선 기울기(error_angle)에 부여하는 비중을 설정합니다.
#    • 각도 예측이 유효할 때만 30% 이하로 시작 후 조정.
#
# 8. Scale (제어값 클리핑 범위) — 범위: 1 ~ 200
#    • PID 출력(control_val)을 ±scale로 제한합니다.
#    • 값이 작으면 반응이 부드럽고 느림, 크면 급격하게 반응.
#
# 9. Throttle (고정 전진 PWM) — 범위: 1068 ~ 1932 (µs)
#    • 기본 전진 속도를 설정합니다.
#    • 직선 구간에서는 높이고, 곡선 진입 시 GUI에서 직접 낮춰 확인.
#
# ------------------------- 튜닝 순서 추천 -------------------------
# 1) Threshold 먼저 조정 → 라인 이진화가 안정적일 때까지
# 2) Kp 조정 → 기본 반응 확인
# 3) Kd 추가 → 진동 억제
# 4) Ki 소량 추가 → 지속 오차 보상
# 5) HoughThresh 조정 → 방향성 보정 안정화
# 6) OffWeight vs AngWeight → 곡선/직선 비중 조율
# 7) Scale 및 Throttle → 클리핑·속도 균형
