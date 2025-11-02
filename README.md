# REAL-TIME-ACCOMODATION-CORRRECTING-VISION-DEVICE-25G64-
Project by: Keletso Modiba and Siyabonga Madalane

Overview 
This project implements a real-time adaptive autofocus system designed for compact and wearable imaging applications. The system uses a camera and servo-actuated lens mechanism to maintain sharp focus automatically by analyzing live image feedback. It integrates computer vision, signal processing, and PID control to achieve precise, stable, and responsive focusing under dynamic conditions.

System Architecture

The autofocus setup consists of the following key modules:
Image Acquisition – A camera module continuously captures video frames for analysis.
Sharpness Evaluation – Each frame is processed using a hybrid sharpness metric combining Laplacian variance, Brenner gradient, and Sobel energy to quantify image focus.
Control Algorithm (PID) – A tuned Proportional–Integral–Derivative (PID) controller adjusts the lens position based on real-time sharpness feedback, correcting for defocus with smooth and stable actuator motion.
Servo Actuation – A microcontroller (ESP32-CAM) controls a miniature servo motor that mechanically shifts the lens to the optimal focus point.
Data Logging and Analysis – The system records time, sharpness, and servo position to CSV for post-experimental evaluation of sensitivity, stability, and response dynamics.

Hardware Requirements

ESP32-CAM microcontroller
Servo motor (standard 180° or micro servo)
Lens assembly compatible with servo actuation
Beamsplitter or optical combiner for shared camera-user view
Web camera

Software Requirements

Python 3.8+ with OpenCV, NumPy, and pySerial
Arduino IDE (for ESP32-CAM firmware bootloading)
MATLAB ( for data visualization and analysis)
