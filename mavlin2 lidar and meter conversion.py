from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math

# ArUco marker parameters
aruco_id_to_find = 72
marker_size = 10  # [cm]

# Camera Parameters
horizontal_res = 640
vertical_res = 480

# Camera fov
horizontal_fov = 62.2 * (math.pi / 180)
vertical_fov = 48.8 * (math.pi / 180)

# Connection to the drone
connection_string = 'udp:192.168.29.235:3000'
vehicle = connect(connection_string, wait_ready=True)

# Initialize the camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, horizontal_res)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, vertical_res)

# Define the aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

# Function to send landing target message
def send_land_message(x, y, z):
    msg = vehicle.message_factory.landing_target_encode(
        0,  # time since system boot, not used
        0,  # target num, not used
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # using local NED frame
        x,
        y,
        z,
        [1, 0, 0, 0],  # Quaternion (w, x, y, z order)
        mavutil.mavlink.MAV_LANDING_TARGET_TYPE_VISION_FIDUCIAL,
        1  # position_valid, set to 1 for a valid target position
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
        x_pixel = np.mean(corners[idx][0][:, 0])
        y_pixel = np.mean(corners[idx][0][:, 1])
        #use below line if lidar is in your drone
        #distance_to_target=vehicle.rangefinder.distance
        distance_to_target=z = 0  # Assuming the drone altitude is already controlled separately

        x_meters = ((x_pixel - horizontal_res / 2) / horizontal_res) * \
                    (horizontal_fov * np.pi / 180) * distance_to_target

        y_meters = ((y_pixel - vertical_fov / 2) / vertical_res) * \
                    (vertical_fov * np.pi / 180) * distance_to_target

        # Send landing target message
        send_land_message(x_meters, y_meters, z)
        time.sleep(0.1)

        print(f"Detected ArUco ID {aruco_id_to_find} at (x, y): ({x_pixel}, {y_pixel})")

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
