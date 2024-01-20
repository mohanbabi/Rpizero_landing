import cv2
import cv2.aruco as aruco
import sys
import time
import math
import numpy as np
from dronekit import connect, VehicleMode,vehicle
from pymavlink import mavutil


# ArUco marker parameters
aruco_id_to_find = 72
marker_size = 10  # [cm]

# Camera Parameters
horizontal_res = 640
vertical_res = 480

# Camera fov
horizontal_fov = 62.2 * (math.pi / 180)
vertical_fov = 48.8 * (math.pi / 180)


velocity = 0.5
takeoff_height = 4



# Initialize the camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, horizontal_res)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, vertical_res)


# ArUco marker variables
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()


# Camera intrinsics
dist_coeff = [9.108362098228629800e-02, -2.202857928995194114e-01, -1.305365599931483786e-03, -4.125787958119479272e-03, -1.456454030882755324e-01]
camera_matrix = [[6.432544414926587706e+02, 0.000000000000000000e+00, 3.099099237849833912e+02], [0.000000000000000000e+00, 6.382225798965612285e+02, 2.301900953005993529e+02], [0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]]
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)


# Variables for ArUco marker detection
found_count = 0
notfound_count = 0
time_last = 0
time_to_wait = 0.1

def arm_and_takeoff(targetHeight):
    while vehicle.is_armable != True:
        print('Waiting for vehicle to become armable...')
        time.sleep(1)
    print('Vehicle is now armable!')

    vehicle.mode = VehicleMode('GUIDED')

    while vehicle.mode != 'GUIDED':
        print('Waiting for drone to enter GUIDED flight mode...')
        time.sleep(1)
    print('Vehicle now in GUIDED mode!')

    vehicle.armed = True
    while vehicle.armed == False:
        print('Waiting for vehicle to become armed...')
        time.sleep(1)

    while True:
        vehicle.simple_takeoff(targetHeight)

    while True:
        print('Current Altitude: %d' % vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= 0.95 * targetHeight:
            break
        time.sleep(1)
    print('Target altitude reached!')


    def send_local_ned_velocity(vx, vy, vz):
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,
            0,
            0,
            0,
            vx,
            vy,
            vz,
            0, 0, 0, 0, 0)
        vehicle.send_mavlink(msg)
        vehicle.flush()

def send_land_message(x, y):
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0, 0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()






if __name__ == '__main__':
    # Connect to the vehicle
    connection_string = 'udp:10.0.2.100:3000'
    vehicle = connect(connection_string, wait_ready=True)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame")
                break

            
            if time.time() - time_last > time_to_wait:
                gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                ids, corners, rejected = aruco.detectMarkers(gray_img, aruco_dict, parameters=parameters)
                if ids is not None:
                        
                    if aruco_id_to_find in ids:
                        marker_index = np.where(ids == aruco_id_to_find)[0][0]
                        ret = aruco.estimatePoseSingleMarkers(corners[marker_index], marker_size,
                                                            cameraMatrix=np_camera_matrix, distCoeffs=np_dist_coeff)
                        rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
                        x, y, z = '{:.2f}'.format(tvec[0]), '{:.2f}'.format(tvec[1]), '{:.2f}'.format(tvec[2])

                        x_sum = np.sum(corners[marker_index][0], axis=0)[0]
                        y_sum = np.sum(corners[marker_index][0], axis=0)[1]

                        x_avg = x_sum / 4
                        y_avg = y_sum / 4

                        x_ang = (x_avg - horizontal_res * 0.5) * horizontal_fov / horizontal_res
                        y_ang = (y_avg - vertical_res * 0.5) * vertical_fov / vertical_res

                        send_land_message(x_ang, y_ang)

                

    except KeyboardInterrupt:
        print("User interrupted the program.")
    finally:
        # Land the vehicle
        print("Landing the vehicle.")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(5)  # Assuming it takes 5 seconds to land
        vehicle.close()

