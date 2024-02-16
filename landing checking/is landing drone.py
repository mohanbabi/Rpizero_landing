import sys
if sys.version_info.major == 3 and sys.version_info.minor >= 10:
    import collections
    from collections.abc import MutableMapping
    setattr(collections, "MutableMapping", MutableMapping)
from dronekit import connect, VehicleMode
import time

# Connect to the vehicle
vehicle = connect('udp:192.168.1.214:3000', wait_ready=True)
print("Connected to vehicle")

# Main loop
while True:
    # Get the current altitude and vertical velocity
    altitude = vehicle.location.global_relative_frame.alt
    vertical_velocity = vehicle.velocity[2]

    # Check if the vehicle is descending and in GUIDED or LAND mode
    if vehicle.mode in [VehicleMode("RTL"), VehicleMode("LAND")] or vertical_velocity > 0.03:
        if (vehicle.armed):
             print(vehicle.armed)
             print("Drone is in the process of landing")
             print("velocity",vertical_velocity)
        else:
            print("disamed")
    else:
        
        print("Drone is not landing")
        

    # Sleep for a short duration before the next check
    time.sleep(1)
