from dronekit import connect, VehicleMode, LocationGlobalRelative
import math
import argparse
import geopy.distance
import time
import firebase_admin
from firebase_admin import db, credentials
import threading
import cv2

# ---------- CAMERA THREAD VARIABLES ----------
camera_running = True


# ---------- CAMERA PREVIEW FUNCTION (RUNS IN BACKGROUND) ----------
def camera_preview_live():
    global camera_running

    cap = cv2.VideoCapture(0)  # Logitech camera

    cap.set(3, 1280)  # width
    cap.set(4, 720)   # height

    if not cap.isOpened():
        print("Camera not detected!")
        return

    print("Camera preview started...")

    while camera_running:
        ret, frame = cap.read()
        if not ret:
            print("Frame grab failed")
            break

        cv2.imshow("Drone Camera Live", frame)

        # Close window when camera_running = False
        if cv2.waitKey(1) & 0xFF == ord('x'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Camera preview stopped.")


# -------------------- FIREBASE SETUP --------------------
cred = credentials.Certificate("dronecred.json")
firebase_admin.initialize_app(cred, {
    "databaseURL": "https://drone-986f4-default-rtdb.firebaseio.com/"
})

waypoints = db.reference("/mission/waypoints").get()
print("\nFetched Waypoints:", waypoints)


# -------------------- DRONE FUNCTIONS --------------------
def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    vehicle = connect(args.connect, baud=57600, wait_ready=True)
    return vehicle


def get_distance(cord1, cord2):
    return geopy.distance.geodesic(cord1, cord2).km * 1000


def arm_and_takeoff(aTargetAltitude):
    print("Pre-arm checks...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(1)
    vehicle.armed = True
    time.sleep(2)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(" Alt:", alt)
        if alt >= aTargetAltitude * 0.95:
            print("Target altitude reached")
            break
        time.sleep(1)


def goto_location(to_lat, to_lon):
    print("Going to:", to_lat, to_lon)

    curr_alt = vehicle.location.global_relative_frame.alt
    target = LocationGlobalRelative(to_lat, to_lon, curr_alt)

    vehicle.simple_goto(target, groundspeed=4)

    target_pt = (to_lat, to_lon)

    while True:
        curr = (
            vehicle.location.global_relative_frame.lat,
            vehicle.location.global_relative_frame.lon
        )

        dist = get_distance(curr, target_pt)
        print("Distance:", dist)

        if dist <= 2:
            print("Reached waypoint.")
            break

        time.sleep(1)


def land_and_wait():
    print("Landing...")
    vehicle.mode = VehicleMode("LAND")

    while vehicle.location.global_relative_frame.alt > 1:
        print(" Alt:", vehicle.location.global_relative_frame.alt)
        time.sleep(1)

    print("Landed. Waiting 5 seconds...")
    time.sleep(5)


# -------------------- MAIN PROGRAM --------------------

# 1️⃣ START CAMERA IN BACKGROUND (NO Q KEY REQUIRED)
camera_thread = threading.Thread(target=camera_preview_live)
camera_thread.start()

# 2️⃣ CONNECT TO DRONE
vehicle = connectMyCopter()

home_lat = vehicle.location.global_relative_frame.lat
home_lon = vehicle.location.global_relative_frame.lon

takeoff_height = 10

print("\n--- Starting Mission ---\n")

# 3️⃣ TAKEOFF FIRST
arm_and_takeoff(takeoff_height)
time.sleep(2)

# 4️⃣ FOLLOW FIREBASE WAYPOINTS
for wp_name, wp in waypoints.items():

    lat = wp["lat"]
    lon = wp["lon"]

    print(f"\nFlying to {wp_name} → {lat}, {lon}")

    goto_location(lat, lon)
    time.sleep(1)

    land_and_wait()  # Land, wait 5s

    print("Taking off again...")
    arm_and_takeoff(takeoff_height)
    time.sleep(2)

# 5️⃣ FINISH → RTL MODE
print("\nAll waypoints done → Switching to RTL...")
goto_location(home_lat, home_lon)

while vehicle.location.global_relative_frame.alt > 1:
    print("RTL... Alt:", vehicle.location.global_relative_frame.alt)
    time.sleep(1)

print("\nRTL Completed. Drone Landed at Home.\n")

# 6️⃣ STOP CAMERA AFTER MISSION
camera_running = False
camera_thread.join()
