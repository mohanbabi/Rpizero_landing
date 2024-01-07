#!/usr/bin/env python3
import math
import numpy as np
from dronekit import connect, VehicleMode
from pymavlink import mavutil
from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2
from time import sleep, time

# UDP Connection String for ArduCopter (replace with your drone's IP and port)
connection_string = 'udp:192.168.29.235:3000'

# Camera parameters
horizontal_resolution = 320
vertical_resolution = 240

# Real-Time Moving Average of the TAG ID
readings = np.array([])
z = 17
max_samples = 4
Ratio = 13  # This is the ratio of the Aruco Tags ID to switch Tracking
x = 0
y = 0

# Connect to the drone over UDP
vehicle = connect(connection_string, wait_ready=True)

# Define function to send landing_target mavlink message for mavlink-based precision landing
def send_land_message(x, y, z, time_boot_ms):
    msg = vehicle.message_factory.landing_target_encode(
        time_boot_ms,
        0,  # target num, not used
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame, not used
        (x - horizontal_resolution / 2) * math.pi / horizontal_resolution,
        (y - vertical_resolution / 2) * math.pi / vertical_resolution,
        z,  # distance, in meters
        0,  # Target x-axis size, in radians
        0  # Target y-axis size, in radians
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()

# Initialize PiCamera
camera = PiCamera()
camera.resolution = (horizontal_resolution, vertical_resolution)
rawCapture = PiRGBArray(camera, size=(horizontal_resolution, vertical_resolution))

# Allow camera to warm up
sleep(2)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array

    # Process the image and obtain ArUco marker information
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Mock values for testing (replace this with your actual image processing results)
    if ids is not None and 72 in ids:
        # Get the index of the marker with ID 72
        target_index = np.where(ids == 72)[0][0]

        z = 17  # Mock distance value, replace with actual distance calculation

        readings = np.append(readings, z)
        avg = np.mean(readings)

        if len(readings) == max_samples:
            readings = np.delete(readings, 0)

        if (avg > Ratio) and (z == 17):
            x = int(corners[target_index][0][0][0] + corners[target_index][0][2][0]) // 2
            y = int(corners[target_index][0][0][1] + corners[target_index][0][2][1]) // 2
            send_land_message(x, y, z, time_boot_ms=int(time() * 1000))

        if (avg <= Ratio) and (z == 8):
            x = int(corners[target_index][0][0][0] + corners[target_index][0][2][0]) // 2
            y = int(corners[target_index][0][0][1] + corners[target_index][0][2][1]) // 2
            send_land_message(x, y, z, time_boot_ms=int(time() * 1000))

    rawCapture.truncate(0)  # Clear the stream for the next frame
    sleep(1)  # Adjust the delay based on your desired loop rate

