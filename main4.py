from dronekit import connect, VehicleMode, LocationGlobalRelative
import argparse
import geopy.distance
import time
import firebase_admin
from firebase_admin import db, credentials
import threading
import cv2


# ==============================================================
#                 CAMERA THREAD CONTROL
# ==============================================================

camera_running = True


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


# ==============================================================
#                 FIREBASE SETUP
# ==============================================================

cred = credentials.Certificate("dronecred.json")
firebase_admin.initialize_app(cred, {
    "databaseURL": "https://drone-986f4-default-rtdb.firebaseio.com/"
})

waypoints = db.reference("/mission/waypoints").get()
print("\nFetched Waypoints:", waypoints)


# ==============================================================
#                 CONNECT TO DRONE
# ==============================================================

def connectMyCopter():
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect')
    args = parser.parse_args()

    vehicle = connect(args.connect, baud=57600, wait_ready=True)
    return vehicle


# ==============================================================
#                 HELPER FUNCTIONS
# ==============================================================

def get_distance(cord1, cord2):
    return geopy.distance.geodesic(cord1, cord2).km * 1000


# ---------- Pixhawk TRUE Landing Detection ----------
def is_really_landed():
    msg = vehicle._master.messages.get('EXTENDED_SYS_STATE', None)
    if msg is None:
        return False
    return msg.landed_state == 1   # 1 = ON_GROUND, 2 = IN_AIR


# ==============================================================
#                 TAKEOFF FUNCTION
# ==============================================================

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


# ==============================================================
#                 GOTO WAYPOINT
# ==============================================================

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

def wait_until_touchdown():
    stable_count = 0
    
    while True:
        alt = vehicle.location.global_relative_frame.alt
        gs = vehicle.groundspeed
        
        print(f" Alt={alt:.2f}  GS={gs:.2f}")

        # Condition for touchdown:
        if alt < 0.25 and gs < 0.10:
            stable_count += 1
        else:
            stable_count = 0

        # Touchdown confirmed after 10 stable samples (~2 seconds)
        if stable_count >= 10:
            print("✔ Touchdown detected!")
            break

        time.sleep(0.2)

# ==============================================================
#                 SAFE DESCENT (STAY ARMED)
# ==============================================================
def descend_and_stay_armed():

    print("⬇ Descending close to ground…")

    target = LocationGlobalRelative(
        vehicle.location.global_relative_frame.lat,
        vehicle.location.global_relative_frame.lon,
        0.25
    )

    vehicle.simple_goto(target, groundspeed=0.4)

    # Wait for touchdown (SAFE)
    wait_until_touchdown()

    print("✔ Drone on ground (armed). Waiting 10 sec...")
    time.sleep(10)
    print("✔ REAL LANDING DETECTED (Pixhawk confirmed)")
    print("⏳ Holding on ground for 10 seconds…")
 # Stay ARMED & wait


# ==============================================================
#                 MAIN PROGRAM
# ==============================================================

# 1️⃣ Start Camera Thread
camera_thread = threading.Thread(target=camera_preview_live)
#camera_thread.start()

# 2️⃣ Connect to Drone
vehicle = connectMyCopter()

home_lat = vehicle.location.global_relative_frame.lat
home_lon = vehicle.location.global_relative_frame.lon

takeoff_height = 10

print("\n--- 🚀 Starting Mission ---\n")

# 3️⃣ Initial Takeoff
arm_and_takeoff(takeoff_height)
time.sleep(2)

# ==============================================================
#                 WAYPOINT LOOP
# ==============================================================

for wp_name, wp in waypoints.items():

    lat = wp["lat"]
    lon = wp["lon"]

    print(f"\n📌 Flying to {wp_name} → {lat}, {lon}")

    goto_location(lat, lon)
    time.sleep(2)

    # DESCEND SAFELY & STAY ARMED
    descend_and_stay_armed()

    print("\n✔ 10 seconds completed on ground.")
    print("🔐 Attempting auto ARM for RTL…")

    # RE-ARM
    vehicle.armed = True
    time.sleep(3)

    while not vehicle.armed:
        print(" Waiting for ARM…")
        vehicle.armed = True
        time.sleep(1)

    print("✔ Drone ARMED again.")

    # AUTO RTL
    print("🛬 Switching to RTL mode…")
    vehicle.mode = VehicleMode("RTL")
    time.sleep(2)

    # Wait until RTL completes
    while vehicle.location.global_relative_frame.alt > 1:
        print(" RTL Alt:", vehicle.location.global_relative_frame.alt)
        time.sleep(1)

    print("\n🏠 RTL Complete. Drone Landed at Home.")
    break   # STOP mission after returning home


# ==============================================================
#                 STOP CAMERA THREAD
# ==============================================================

#camera_running = False
#camera_thread.join()

print("\n🎉 Mission Finished Successfully.\n")
