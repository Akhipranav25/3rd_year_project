#completely relying on tVec_distance_combo

import cv2 as cv
from cv2 import aruco
import numpy as np
import math
import serial
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(16, GPIO.OUT)
GPIO.output(16, 0)

yaw_deg_list = []

ser = serial.Serial('/dev/ttyACM0', 1000000, timeout=1)
ser.reset_input_buffer()

calib_data_path = r"/home/akhipranav25/Documents/3rd_year_project/my_code/calibration_savez.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)
cam_mat = calib_data["mtx"]
dist_coef = calib_data["dist"]
r_vectors = calib_data["rvecs"]
t_vectors = calib_data["tvecs"]

command = ""

COMMAND_INTERVAL = 0.25  # seconds — reduced from 0.55 s now that the Arduino is non-blocking

last_command_time = 0.0

MARKER_SIZE = 9

# ── FIX 1: Create the detector ONCE, outside the loop ─────────────────────────
# Previously this was inside `while True`, causing a fresh allocation every frame.
marker_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
param_markers = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(marker_dict, param_markers)
# ──────────────────────────────────────────────────────────────────────────────

# Pre-build the object_points array once — it never changes
half_size = MARKER_SIZE / 2
object_points = np.array([
    [-half_size,  half_size, 0],
    [ half_size,  half_size, 0],
    [ half_size, -half_size, 0],
    [-half_size, -half_size, 0]
], dtype=np.float32)

cap = cv.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # ── FIX 2: Use the pre-built detector rather than reinstantiating it ───────
    marker_corners, marker_IDs, reject = detector.detectMarkers(gray_frame)
    # ──────────────────────────────────────────────────────────────────────────

    previous_command = command

    if marker_IDs is not None:
        cv.aruco.drawDetectedMarkers(frame, marker_corners, marker_IDs)

        if marker_corners:
            total_markers = range(0, marker_IDs.size)
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                cv.polylines(
                    frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                )

                image_points = corners.reshape(4, 2)
                corners_int = image_points.astype(int)
                top_right   = corners_int[0].ravel()
                top_left    = corners_int[1].ravel()
                bottom_right = corners_int[2].ravel()
                bottom_left  = corners_int[3].ravel()

                success, rVec, tVec = cv.solvePnP(
                    object_points, image_points, cam_mat, dist_coef
                )

                distance = np.sqrt(
                    tVec[2][0] ** 2 + tVec[0][0] ** 2 + tVec[1][0] ** 2
                )

                R, _ = cv.Rodrigues(rVec)

                roll  = math.atan2(R[1, 0], R[0, 0])
                yaw   = math.atan2(-R[2, 0], math.sqrt(R[2, 1]**2 + R[2, 2]**2))
                pitch = math.atan2(R[2, 1], R[2, 2])

                yaw_deg   = math.degrees(yaw)
                pitch_deg = math.degrees(pitch)
                roll_deg  = math.degrees(roll)
                
                if tVec[0][0] >= 0:
                    tVec_distance_combo = 575*(tVec[0][0]**2)/(distance**2)
                elif tVec[0][0] <0:
                    tVec_distance_combo = -575*(tVec[0][0]**2)/(distance**2)

                cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec, tVec, 4, 4)
                cv.putText(
                    frame,
                    f"id: {ids[0]} Dist: {round(distance, 2)}",
                    top_right,
                    cv.FONT_HERSHEY_PLAIN, 1.3, (0, 0, 255), 2, cv.LINE_AA,
                )
                cv.putText(
                    frame,
                    f"x:{round(tVec[0][0],1)} y: {round(tVec[1][0],1)} ",
                    bottom_left,
                    cv.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255), 2, cv.LINE_AA,
                )
                angle_text = f"pitch:{round(pitch_deg, 2)} \nyaw:{round(yaw_deg, 2)} \nroll:{round(roll_deg, 2)}"
                y0, dy = 0, 20
                for line_i, line in enumerate(angle_text.split('\n')):
                    y = y0 + line_i * dy
                    cv.putText(
                        frame,
                        line,
                        bottom_right + (0, y),
                        cv.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255), 2, cv.LINE_AA,
                    )

            # Build command string from the last-processed marker's tVec / angles
            command = ""
            """if tVec[1][0] > 1:
                command += "3%"
            elif tVec[1][0] < -1:
                command += "2%"""

            if tVec_distance_combo > 0.05:
                command += "0?" + str(round(tVec_distance_combo, 2)) + "%"
            elif tVec_distance_combo < -0.05:
                command += "1?" + str(abs(round(tVec_distance_combo, 2))) + "%"

            if yaw_deg > 4 and yaw_deg <= 180 and tVec_distance_combo < 0.05 and tVec_distance_combo > -0.05:
                command += "7?" + str(round(yaw_deg)) + "%"
            elif yaw_deg < -4 and yaw_deg > -180 and tVec_distance_combo < 0.05 and tVec_distance_combo > -0.05:
                command += "6?" + str(round(-yaw_deg)) + "%"
                
            elif tVec_distance_combo < 0.05 and tVec_distance_combo > -0.05:
                command += "12%"

            if distance > 40:
                command += "10?" + str(round(distance)) + "%"
                
            if distance <= 40:
                command += "13%"
                
            if distance<=40 and tVec_distance_combo < 0.05 and tVec_distance_combo > -0.05 and yaw_deg < 4 and yaw_deg > -4:
                if len(yaw_deg_list) < 10:
                    yaw_deg_list.append(yaw_deg)
                    #print(yaw_deg_list)
                elif len(yaw_deg_list) >= 10:
                    yaw_deg = sum(yaw_deg_list)/len(yaw_deg_list)
                    #print(yaw_deg)
                    if yaw_deg >= 0:
                        command += "7?" + str(round(yaw_deg, 2)) + "%"
                    elif yaw_deg < 0:
                        command += "6?" + str(round(-yaw_deg, 2)) + "%"
                    for y in yaw_deg_list:
                        yaw_deg_list.clear()
                
            if distance<=40 and tVec_distance_combo < 0.05 and tVec_distance_combo > -0.05 and yaw_deg < 0.5 and yaw_deg > -0.5:
                command = "11%"
                

    cv.imshow('Detected Markers:', frame)

    current_time = time.time()

    if (current_time - last_command_time) >= COMMAND_INTERVAL:
        command = "<" + command + ">\n"
        if command != "<>\n":
            print(command)
            ser.write(command.encode())
        command = ""
        last_command_time = current_time

    # ── FIX 3: Non-blocking serial read ───────────────────────────────────────
    # readline() blocks until '\n' arrives or the timeout expires (1 s), which
    # stalls the frame loop.  Use in_waiting + read() to stay non-blocking.
    #if ser.in_waiting > 0:
        #line = ser.read(ser.in_waiting).decode('utf-8', errors='replace')
        #print(line)
    # ──────────────────────────────────────────────────────────────────────────

    # ── FIX 4: waitKey(1) instead of waitKey(10) ──────────────────────────────
    # waitKey(10) forces a 10 ms pause every frame regardless.
    # waitKey(1) just pumps the GUI event queue without a significant stall.
    cv.waitKey(1)
    # ──────────────────────────────────────────────────────────────────────────

cv.destroyAllWindows()







