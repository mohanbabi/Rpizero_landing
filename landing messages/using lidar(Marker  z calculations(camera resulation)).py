#!/usr/bin/env python3
import math
import numpy as np
from dronekit import connect, VehicleMode
from pymavlink import mavutil
from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2
from time import sleep

# UDP Connection String for ArduCopter (replace with your drone's IP and port)
connection_string = 'udp:192.168.29.235:3000'

# Camera parameters
horizontal_resolution = 640
vertical_resolution = 480
horizontal_fov = 62.2 * math.pi / 180
vertical_fov = 48.8 * math.pi / 180

# Real-Time Moving Average of the TAG ID
readings = np.array([])
max_samples = 4
Ratio = 13  # This is the ratio of the Aruco Tags ID to switch Tracking

# Connect to the drone over UDP
vehicle = connect(connection_string, wait_ready=True)

# Define function to send landing_target MAVLink message for MAVLink-based precision landing
def send_land_message(x, y, z, horizontal_fov, vertical_fov):
    # Calculate angles based on camera FOV
    x_angle = (x - horizontal_resolution / 2) * (horizontal_fov / horizontal_resolution)
    y_angle = (y - vertical_resolution / 2) * (vertical_fov / vertical_resolution)

    # Create landing target MAVLink message
    msg = vehicle.message_factory.landing_target_encode(
        0,  # time since system boot, not used
        0,  # target num, not used
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame, not used
        int(x_angle * 1e4),  # convert angle to 1e-4 radians
        int(y_angle * 1e4),  # convert angle to 1e-4 radians
        z,  # actual distance, in meters
        0,  # Target x-axis size, in radians (not used)
        0  # Target y-axis size, in radians (not used)
    )

    # Send the MAVLink message
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Initialize PiCamera
camera = PiCamera()
camera.resolution = (horizontal_resolution, vertical_resolution)
rawCapture = PiRGBArray(camera, size=(horizontal_resolution, vertical_resolution))

# Allow the camera to warm up
sleep(2)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array

    # Process the image and obtain ArUco marker information
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Check if marker ID 72 is detected
    if ids is not None and 72 in ids:
        # Get the index of the marker with ID 72
        target_index = np.where(ids == 72)[0][0]

        # Get the rangefinder distance as the z value
        z = vehicle.rangefinder.distance

        # Mock average calculation for testing (replace with actual calculation)
        readings = np.append(readings, z)
        avg = np.mean(readings)

        if len(readings) == max_samples:
            readings = np.delete(readings, 0)

        # Trigger landing message when marker ID 72 is detected
        x = int(corners[target_index][0][0][0] + corners[target_index][0][2][0]) // 2
        y = int(corners[target_index][0][0][1] + corners[target_index][0][2][1]) // 2
        send_land_message(x, y, z, horizontal_fov, vertical_fov)

    rawCapture.truncate(0)  # Clear the stream for the next frame
    sleep(1)  # Adjust the delay based on your desired loop rate
