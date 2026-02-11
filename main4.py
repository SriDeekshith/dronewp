from dronekit import connect, VehicleMode, LocationGlobalRelative
import argparse
import geopy.distance
import time
import firebase_admin
from firebase_admin import db, credentials
import threading
import cv2

# ---------- CAMERA THREAD CONTROL ----------
camera_running = True


# ---------- CAMERA PREVIEW THREAD ----------
def camera_preview_live():
    global camera_running

    cap = cv2.VideoCapture(0)
    cap.set(3, 1280)
    cap.set(4, 720)

    if not cap.isOpened():
        print("Camera not detected!")
        return

    print("📷 Camera preview started...")

    while camera_running:
        ret, frame = cap.read()
        if not ret:
            break

        cv2.imshow("Drone Camera Live", frame)

        if cv2.waitKey(1) & 0xFF == ord('x'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("📷 Camera preview stopped.")


# -------------------- FIREBASE SETUP --------------------
cred = credentials.Certificate("dronecred.json")
firebase_admin.initialize_app(cred, {
    "databaseURL": "https://drone-986f4-default-rtdb.firebaseio.com/"
})

waypoints = db.reference("/mission/waypoints").get()
print("\nFetched Waypoints:", waypoints)


# -------------------- CONNECT TO DRONE --------------------
def connectMyCopter():
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect')
    args = parser.parse_args()

    vehicle = connect(args.connect, baud=57600, wait_ready=True)
    return vehicle


# -------------------- DISTANCE FUNCTION --------------------
def get_distance(cord1, cord2):
    return geopy.distance.geodesic(cord1, cord2).km * 1000


# -------------------- ARDUPILOT REAL LANDING STATE --------------------
def is_really_landed():
    """
    Reads ArduPilot EXTENDED_SYS_STATE message.
    landed_state:
        1 = ON GROUND
        2 = IN AIR
    """
    msg = vehicle._master.messages.get('EXTENDED_SYS_STATE', None)

    if msg is None:
        return False

    return msg.landed_state == 1


# -------------------- TAKEOFF FUNCTION --------------------
def arm_and_takeoff(aTargetAltitude):
    print("Pre-arm checks...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors…")
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(1)
    vehicle.armed = True
    time.sleep(2)

    print("Taking off…")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(" Alt:", alt)
        if alt >= aTargetAltitude * 0.95:
            print("🚁 Target altitude reached!")
            break
        time.sleep(1)


# -------------------- GOTO WAYPOINT --------------------
def goto_location(to_lat, to_lon):
    print("➡ Going to:", to_lat, to_lon)

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
        print(" Distance:", dist)

        if dist <= 2:
            print("🎯 Reached waypoint!")
            break

        time.sleep(1)


# -------------------- SAFE DESCEND (REMAIN ARMED) --------------------
def descend_and_stay_armed():
    print("⬇ Descending close to ground (stay ARMED)…")

    target = LocationGlobalRelative(
        vehicle.location.global_relative_frame.lat,
        vehicle.location.global_relative_frame.lon,
        0.2
    )

    vehicle.simple_goto(target, groundspeed=0.5)

    # Wait until altitude drops close to ground
    while vehicle.location.global_relative_frame.alt > 0.6:
        print(" Alt:", vehicle.location.global_relative_frame.alt)
        time.sleep(0.3)

    print("Near ground… waiting for Pixhawk to detect landing…")

    # Wait for Pixhawk's REAL landing detection
    while not is_really_landed():
        print(" Waiting for real landing confirmation...")
        time.sleep(0.2)

    print("✔ REAL LANDING DETECTED (Pixhawk confirmed)")
    print("⏳ Holding on ground for 10 seconds (ARMED)…")

    time.sleep(10)


# -------------------- MAIN PROGRAM --------------------

# 1️⃣ Start camera
camera_thread = threading.Thread(target=camera_preview_live)
camera_thread.start()

# 2️⃣ Connect to Pixhawk
vehicle = connectMyCopter()

home_lat = vehicle.location.global_relative_frame.lat
home_lon = vehicle.location.global_relative_frame.lon

takeoff_height = 10

print("\n--- 🚀 Starting Mission ---\n")

# 3️⃣ First takeoff
arm_and_takeoff(takeoff_height)
time.sleep(2)

# 4️⃣ WAYPOINT LOOP
for wp_name, wp in waypoints.items():

    lat = wp["lat"]
    lon = wp["lon"]

    print(f"\n📌 Flying to {wp_name} → {lat}, {lon}")

    goto_location(lat, lon)
    time.sleep(2)

    # DESCEND SAFELY, STAY ARMED
    descend_and_stay_armed()

    # Switch to STABILIZE for manual RC takeoff
    print("\n🔄 Switching to STABILIZE — YOU TAKE OFF MANUALLY NOW")
    vehicle.mode = VehicleMode("STABILIZE")
    time.sleep(2)

    print("🎮 TAKE OFF USING TRANSMITTER")
    print("⬆ Fly to ~10 meters")
    print("🔁 Then flip mode switch to GUIDED")

    # WAIT FOR PILOT TO SWITCH BACK
    while vehicle.mode.name != "GUIDED":
        print(" Waiting for GUIDED mode...")
        time.sleep(1)

    print("✔ GUIDED detected — resuming mission…")
    arm_and_takeoff(takeoff_height)
    time.sleep(2)


# 5️⃣ RETURN HOME
print("\n🏠 All waypoints done → RTL…")
vehicle.mode = VehicleMode("RTL")

while vehicle.location.global_relative_frame.alt > 1:
    print(" RTL Alt:", vehicle.location.global_relative_frame.alt)
    time.sleep(1)

print("\n🛬 RTL Complete. Drone Landed at Home.\n")

# 6️⃣ Stop camera
camera_running = False
camera_thread.join()
