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


# Connect to the vehicle
vehicle = connect('udp:192.168.1.214:3000', wait_ready=True)

# Set the desired yaw angle in degrees
desired_yaw_degrees = -90

# Convert the yaw angle to radians
desired_yaw_radians = math.radians(desired_yaw_degrees)

# Set the vehicle mode to GUIDED
vehicle.mode = VehicleMode("GUIDED")

# Arm the drone
vehicle.armed = True

# Wait for the drone to arm
while not vehicle.armed:
    print("Waiting for the drone to arm...")
    time.sleep(1)

# Main control loop
while True:
    # Calculate the quaternion representing the desired yaw
    # Assuming no pitch or roll (i.e., the drone is level)
    quaternion = [math.cos(desired_yaw_radians / 2), 0, 0, math.sin(desired_yaw_radians / 2)]

    # Set the target orientation for the drone
    vehicle.message_factory.set_attitude_target_send(
        0,  # time_boot_ms
        10,
        0,
        0b00000000,  # type_mask - ignore all orientation and angular rate fields
        quaternion,
        0, 0, 0,  # body roll, pitch, yaw rates (all set to 0)
        0.5,  # thrust (not used in this example)
    )

    # Sleep for a short duration to avoid excessive updates
    time.sleep(0.1)
