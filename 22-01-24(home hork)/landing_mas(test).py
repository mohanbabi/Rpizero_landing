import cv2
import cv2.aruco as aruco
import sys
if sys.version_info.major == 3 and sys.version_info.minor >= 10:
    import collections
    from collections.abc import MutableMapping
    setattr(collections, "MutableMapping", MutableMapping)

import time
import math
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil


def send_land_message1(x, y, z):
    msg = vehicle.message_factory.landing_target_encode(
        0,  # time since system boot, not used
        0,  # target num, not used
        mavutil.mavlink.MAV_FRAME_BODY_FRD,  # frame, not used
        x,
        y,
        z,  # distance, in meters
        0,  # Target x-axis size, in radians
        0  # Target y-axis size, in radians
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


connection_string = 'udp:192.168.29.164:3000'
vehicle = connect(connection_string, wait_ready=True)
print("connected")

while True:
    x= 0
    y= -0.1
    z = 5
    send_land_message1(x,y,z)
    # time.sleep(2)
