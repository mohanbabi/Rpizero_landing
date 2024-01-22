import cv2
import cv2.aruco as aruco
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# Connect to the drone
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

# Capture video from the drone's camera
cap = cv2.VideoCapture(0)

# ArUco marker ID to land on
target_marker_id = 72

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters_create()

    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None and target_marker_id in ids:
        # Calculate the centroid of the marker
        centroid_x = int(sum(corners[0][0][:, 0]) / 4)
        centroid_y = int(sum(corners[0][0][:, 1]) / 4)

        # Send a landing target message
        vehicle.message_factory.landing_target_encode(
            0,  # time_usec (not used)
            0,  # target num, not used
            mavutil.mavlink.MAV_FRAME_BODY_FRD,  # coordinate frame
            centroid_x,
            centroid_y,
            0,  # distance (not used)
            0,  # Target x-axis size, in radians (not used)
            0  # Target y-axis size, in radians (not used)
        )

        # Optionally, you can also set the mode to LAND
        vehicle.mode = VehicleMode("LAND")
        print("Target ArUco marker detected. Sending landing target message and initiating landing...")
        break

    # Display the frame with detected markers
    aruco.drawDetectedMarkers(frame, corners, ids)
    cv2.imshow('ArUco Marker Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
