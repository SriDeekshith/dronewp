from dronekit import connect, VehicleMode
import time

# ---------------------------
# Connect to the Pixhawk
# ---------------------------
print("Connecting to vehicle...")
vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)
# Change to /dev/ttyAMA0 and 57600 if using TELEM2

# ---------------------------
# ARM the drone
# ---------------------------
def arm_drone():
    print("Checking pre-arm status...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialize...")
        time.sleep(1)

    print("Setting mode to GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(1)

    print("Arming motors...")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("✔ Drone is ARMED!")

# ---------------------------
# DISARM the drone
# ---------------------------
def disarm_drone():
    print("Disarming motors...")
    vehicle.armed = False

    while vehicle.armed:
        print(" Waiting for disarm...")
        time.sleep(1)

    print("✔ Drone is DISARMED!")

# ---------------------------
# MAIN
# ---------------------------

arm_drone()
time.sleep(5)   # Wait for 5 seconds while armed
disarm_drone()

vehicle.close()
print("Done.")
