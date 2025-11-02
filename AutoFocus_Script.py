# Authors : Siyabonga Madalane and Keletso Modiba
# Date : 02 November 2025
# An autofocus system, please ensure that a serial protocol is uploaded on primary microcontroller

import cv2
import numpy as np
import time
import serial
import csv
import os

# ---------------- CONFIGURATION ----------------
CAMERA_INDEX = 1 # 0 for host computer and 1 for peripheral camera
ROI_FRACTION = 0.5

SERIAL_PORT = "COM7" #select port connected to microcontroller
BAUD_RATE = 115200
SERVO_MIN, SERVO_MAX = 5, 175
INITIAL_POS = 90

COARSE_STEP = 25
COARSE_SETTLE_DELAY = 0.008
FINE_SWEEP_RANGE = 10
FINE_STEP = 3
FINE_SETTLE_DELAY = 0.008

KP, KI, KD = 2000, 0.25, 5
INTEGRAL_LIMIT = 50

REFOCUS_THRESHOLD = 0.75
REFOCUS_COOLDOWN = 3.0
STABILITY_MARGIN = 0.10

clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))


# ---------------- SHARPNESS METRIC ----------------
def compute_sharpness(frame):
    h, w = frame.shape[:2]
    x0 = int(w * (1 - ROI_FRACTION) / 2)
    y0 = int(h * (1 - ROI_FRACTION) / 2)
    roi = frame[y0:y0 + int(h * ROI_FRACTION), x0:x0 + int(w * ROI_FRACTION)]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = clahe.apply(gray)

    lap_var = cv2.Laplacian(gray, cv2.CV_64F).var()
    brenner = np.sum((gray[:, 2:] - gray[:, :-2]) ** 2)
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    sobel_energy = np.mean(sobelx ** 2 + sobely ** 2)

    lap_norm = lap_var / (lap_var + 1000.0)
    brenner_norm = brenner / (brenner + 1e6)
    sobel_norm = sobel_energy / (sobel_energy + 1000.0)

    score = float(0.4 * lap_norm + 0.3 * brenner_norm + 0.3 * sobel_norm)
    return score


# ---------------- SERVO CONTROLLER ----------------
class ServoController:
    def __init__(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(1.5)
            self.serial_ok = True
            print(f"✓ Connected to {SERIAL_PORT}")
        except Exception as e:
            print(f"Serial failed: {e} -> running in SIM mode")
            self.ser = None
            self.serial_ok = False

        self.current_pos = INITIAL_POS
        self.integral = 0.0
        self.last_error = 0.0
        self.send_position(INITIAL_POS)

    def send_raw(self, text):
        if not self.serial_ok or self.ser is None:
            return True
        try:
            self.ser.write(text.encode())
            return True
        except:
            self.serial_ok = False
            return False

    def send_position(self, angle):
        angle = int(max(SERVO_MIN, min(SERVO_MAX, round(angle))))
        cmd = f"SERVO:{angle}\n"
        if self.send_raw(cmd):
            self.current_pos = angle
        return True

    def pid_adjust(self, error):
        self.integral += error
        self.integral = np.clip(self.integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
        derivative = error - self.last_error
        self.last_error = error
        output = KP * error + KI * self.integral + KD * derivative
        return output

    def close(self):
        if self.serial_ok and self.ser:
            try:
                self.ser.close()
            except:
                pass


# ---------------- UI ----------------
def draw_overlay(frame, sharpness, servo_pos, status="Tracking"):
    overlay = frame.copy()
    box_w, box_h = 300, 110
    cv2.rectangle(overlay, (10, 10), (10 + box_w, 10 + box_h), (40, 40, 40), -1)
    cv2.rectangle(overlay, (10, 10), (10 + box_w, 10 + box_h), (255, 255, 255), 1)
    cv2.putText(overlay, f"Sharpness: {sharpness:.3f}", (25, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
    cv2.putText(overlay, f"Servo: {servo_pos} Degree", (25, 75),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
    cv2.putText(overlay, f"Mode: {status}", (25, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
    return cv2.addWeighted(overlay, 0.78, frame, 0.22, 0)


# ---------------- SWEEP ROUTINES ----------------
def coarse_sweep(cap, servo, time_data, sharpness_data, servo_data, t0):
    best_pos, best_sharp = servo.current_pos, -1.0
    for pos in range(SERVO_MIN, SERVO_MAX + 1, COARSE_STEP):
        servo.send_position(pos)
        time.sleep(COARSE_SETTLE_DELAY)
        ret, frame = cap.read()
        if not ret:
            continue
        s = compute_sharpness(frame)
        time_data.append(time.time() - t0)
        sharpness_data.append(s)
        servo_data.append(servo.current_pos)
        if s > best_sharp:
            best_sharp, best_pos = s, pos
        disp = draw_overlay(frame.copy(), s, servo.current_pos, status="Coarse sweep")
        cv2.imshow("Autofocus System", disp)
        cv2.waitKey(1)
    return best_pos, best_sharp


def fine_sweep(cap, servo, center, time_data, sharpness_data, servo_data, t0):
    start = max(SERVO_MIN, center - FINE_SWEEP_RANGE)
    end = min(SERVO_MAX, center + FINE_SWEEP_RANGE)
    best_pos, best_sharp = center, -1.0
    for pos in range(start, end + 1, FINE_STEP):
        servo.send_position(pos)
        time.sleep(FINE_SETTLE_DELAY)
        ret, frame = cap.read()
        if not ret:
            continue
        s = compute_sharpness(frame)
        time_data.append(time.time() - t0)
        sharpness_data.append(s)
        servo_data.append(servo.current_pos)
        if s > best_sharp:
            best_sharp, best_pos = s, pos
        disp = draw_overlay(frame.copy(), s, servo.current_pos, status="Fine sweep")
        cv2.imshow("Autofocus System", disp)
        cv2.waitKey(1)
    return best_pos, best_sharp


def refocus(cap, servo, time_data, sharpness_data, servo_data, t0):
    cpos, _ = coarse_sweep(cap, servo, time_data, sharpness_data, servo_data, t0)
    fpos, fsharp = fine_sweep(cap, servo, cpos, time_data, sharpness_data, servo_data, t0)
    servo.send_position(fpos)
    return fpos, fsharp


# ---------------- MAIN ----------------
def main():
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        print("Camera not accessible")
        return

    servo = ServoController()
    for _ in range(4):
        cap.read()

    time_data, sharpness_data, servo_data = [], [], []
    t0 = time.time()

    best_pos, best_sharp = refocus(cap, servo, time_data, sharpness_data, servo_data, t0)
    last_refocus = time.time()
    lower_limit = best_sharp * (1 - STABILITY_MARGIN)
    upper_limit = best_sharp * (1 + STABILITY_MARGIN)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.001)
                continue

            s = compute_sharpness(frame)
            error = best_sharp - s
            pid_out = servo.pid_adjust(error)
            new_angle = servo.current_pos + pid_out * 0.0001
            new_angle = max(SERVO_MIN, min(SERVO_MAX, new_angle))
            servo.send_position(new_angle)

            time_data.append(time.time() - t0)
            sharpness_data.append(s)
            servo_data.append(servo.current_pos)

            if (s < lower_limit or s > upper_limit) and (time.time() - last_refocus) > REFOCUS_COOLDOWN:
                best_pos, best_sharp = refocus(cap, servo, time_data, sharpness_data, servo_data, t0)
                lower_limit = best_sharp * (1 - STABILITY_MARGIN)
                upper_limit = best_sharp * (1 + STABILITY_MARGIN)
                last_refocus = time.time()

            display = draw_overlay(frame, s, servo.current_pos, status="Tracking")
            cv2.imshow("Autofocus System", display)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            elif key == ord('r'):
                best_pos, best_sharp = refocus(cap, servo, time_data, sharpness_data, servo_data, t0)

    finally:
        cap.release()
        cv2.destroyAllWindows()
        servo.close()

if __name__ == "__main__":
    main()
