# aruco_landing_dronekit.py

from dronekit import connect, VehicleMode
from pymavlink import mavutil
import numpy as np
import cv2
import cv2.aruco as aruco
import time

#--- Define Tag
id_to_find = 72
marker_size = 10  # [cm]

# Get the camera calibration path
calib_path = "/home/pi/how_do_drones_work/opencv/"
camera_matrix = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')

# 180 deg rotation matrix around the x axis
R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0

# Define the aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

# UDP Connection String for ArduCopter (replace with your drone's IP and port)
connection_string = 'udp:192.168.29.235:3000'

# Connect to the drone over UDP
vehicle = connect(connection_string, wait_ready=True)

# while not vehicle.is_armable:
#     print("Waiting for vehicle to initialize...")
#     time.sleep(1)

# print("Arming motors")
# vehicle.mode = VehicleMode("GUIDED")
# vehicle.armed = True

# # Wait for arming
# while not vehicle.armed:
#     print("Waiting for arming...")
#     time.sleep(1)

# print("Taking off")
# vehicle.simple_takeoff(5)  # Take off to 5 meters

# # Wait for the vehicle to reach a safe height
# while vehicle.location.global_relative_frame.alt < 5:
#     print("Altitude: ", vehicle.location.global_relative_frame.alt)
#     time.sleep(1)

# print("Searching for ArUco marker")

# Capture the videocamera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True:
    # Read the camera frame
    ret, frame = cap.read()

    # Undistort the image
    frame_undistorted = cv2.undistort(frame, camera_matrix, camera_distortion)

    # Convert in gray scale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find all the aruco markers in the image
    corners, ids, rejected = aruco.detectMarkers(
        image=gray, dictionary=aruco_dict, parameters=parameters,
        # cameraMatrix=camera_matrix, distCoeff=camera_distortion
    )

    if ids is not None and ids[0] == id_to_find:
        # Pose estimation
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
        rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
        R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc = R_ct.T

        # MAVLink landing target message
        x_angle, y_angle, z = tvec[0], tvec[1], tvec[2]

        # Create landing target MAVLink message
        msg = vehicle.message_factory.landing_target_encode(
            0,  # time since system boot, not used
            0,  # target num, not used
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame, not used
            int(x_angle * 1e4),  # convert angle to 1e-4 radians
            int(y_angle * 1e4),  # convert angle to 1e-4 radians
            int(z * 1e2),  # convert distance to 1e-2 centimeters
            0,  # Target x-axis size, in radians (not used)
            0  # Target y-axis size, in radians (not used)
        )

        # Send the MAVLink message
        vehicle.send_mavlink(msg)

    # use 'q' to quit
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        vehicle.close()
        break
