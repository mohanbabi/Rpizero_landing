#!/usr/bin/env python3
import math
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import cv2
import cv2.aruco as aruco

# Camera Parameters
horizontal_fov = 70.2 * math.pi / 180
vertical_fov = 43.3 * math.pi / 180
horizontal_resolution = 320
vertical_resolution = 240

# Real-Time Moving Average of the TAG ID
readings = np.array([])
max_samples = 4
Ratio = 13  # This is the ratio of the Aruco Tags ID to switch Tracking
aruco_id_to_find = 17
x = 0
y = 0

# Connect to the vehicle
connection_string = 'udp:192.168.29.235:3000'
vehicle = connect(connection_string, wait_ready=True)


# Initialize the camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, horizontal_resolution)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, vertical_resolution)

# Define the aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

# Define function to send landing_target mavlink message for mavlink-based precision landing
# http://mavlink.org/messages/common#LANDING_TARGET
def send_land_message(x, y, z):
    msg = vehicle.message_factory.landing_target_encode(
        0,  # time since system boot, not used
        0,  # target num, not used
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame, not used
        (x - horizontal_resolution / 2) * horizontal_fov / horizontal_resolution,
        (y - vertical_resolution / 2) * vertical_fov / vertical_resolution,
        z,  # distance, in meters
        0,  # Target x-axis size, in radians
        0  # Target y-axis size, in radians
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

while True:
    # Read the camera frame
    ret, frame = cap.read()

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None and aruco_id_to_find in ids:
        idx = np.where(ids == aruco_id_to_find)[0][0]
        x = int(np.mean(corners[idx][0][:, 0]))
        y = int(np.mean(corners[idx][0][:, 1]))

        # Display marker ID and centroid coordinates
        print(f"Detected ArUco ID {aruco_id_to_find} at (x, y): ({x}, {y})")

        # Add the ArUco ID to the readings array
        readings = np.append(readings, aruco_id_to_find)
        avg = np.mean(readings)

        if len(readings) == max_samples:
            readings = np.delete(readings, 0)

        # Check for the condition to send landing message
        if avg > Ratio:
            send_land_message(x, y, avg)

    # Display the frame with ArUco marker detection
    # frame = aruco.drawDetectedMarkers(frame, corners, ids)
    # cv2.imshow("ArUco Marker Detection", frame)

    # Break the loop if 'q' key is pressed
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# Release the camera and close OpenCV windows
cap.release()
cv2.destroyAllWindows()
