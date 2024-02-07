from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

if sys.version_info.major == 3 and sys.version_info.minor >= 10:
    import collections
    from collections.abc import MutableMapping
    setattr(collections, "MutableMapping", MutableMapping)

import time
import math
import argparse
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil



def send_landing(x, y, z):
    # Convert x, y, z to float
    x = float(x)
    y = float(y)
    z = float(z)

    # Create landing target MAVLink2 message with quaternion
    msg =vehicle.message_factory.landing_target_encode(
        0,  # time since system boot, not used
        0,  # target num, not used
        mavutil.mavlink.MAV_FRAME_BODY_FRD,  # frame, specify your desired frame
        0.0,  # x position (ignored for quaternion)
        0.0,  # y position (ignored for quaternion)
        0.0,  # z position (ignored for quaternion)
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        [x, y, z, 0.0],  # Quaternion of landing target orientation (w, x, y, z order)
        0,  # Type of landing target (specify your desired type)
        1  # position_valid (1: valid, 0: invalid)
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

parser = argparse.ArgumentParser()
# -----replace udp:192.168.29.235:14554 with your drone mavlink data
parser.add_argument('--connect', default='udp:192.168.1.214:3000')
args = parser.parse_args()
print('Connecting...')
vehicle = connect(args.connect)


while True:
    x = 1
    y = 1
    z = 1
    send_landing(x, y, z)
    print("message sent")