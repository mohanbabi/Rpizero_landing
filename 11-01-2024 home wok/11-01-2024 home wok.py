from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil
import numpy as np
import cv2
import cv2.aruco as aruco
import time
import math
import argparse

#--- Define Tag
id_to_find = 72
marker_size = 10  # [cm]

# Camera Parameters
horizontal_res = 640
vertical_res = 480

# Camera fov
horizontal_fov = 62.2 * (math.pi / 180)
vertical_fov = 48.8 * (math.pi / 180)


# Get the camera calibration path
calib_path = "/home/pi/how_do_drones_work/opencv/"
camera_matrix = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')


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



cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)



def send_land_message(x, y):
    global vehicle
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,
        0,
        0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


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
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix=camera_matrix,
                                                distCoeffs=camera_distortion)
        (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
        x = '{:.2f}'.format(tvec[0])
        y = '{:.2f}'.format(tvec[1])
        z = '{:.2f}'.format(tvec[2])

        y_sum = 0
        x_sum = 0

        x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
        y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]

        x_avg = x_sum * 0.25
        y_avg = y_sum * 0.25

        x_ang = (x_avg - horizontal_res * 0.5) * (horizontal_fov / horizontal_res)
        y_ang = (y_avg - vertical_res * 0.5) * (vertical_fov / vertical_res)


        print("peparing")
        send_land_message(x_ang, y_ang)
        print("sent x, y ", x_ang,y_ang)