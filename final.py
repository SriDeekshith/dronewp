from dronekit import connect, VehicleMode, LocationGlobalRelative
import argparse
import geopy.distance
import time
import firebase_admin
from firebase_admin import db, credentials


# ==============================================================
#                 FIREBASE SETUP
# ==============================================================

cred = credentials.Certificate("dronecred.json")
firebase_admin.initialize_app(cred, {
    "databaseURL": "https://drone-986f4-default-rtdb.firebaseio.com/"
})

waypoints = db.reference("/mission/waypoints").get()

# ==============================================================
# 1️⃣ FETCH WAYPOINTS → IF EMPTY EXIT
# ==============================================================

if not waypoints:
    print("❌ No waypoints found. Mission ending safely.")
    exit()

print("✅ Waypoints received:", waypoints)
print("Total waypoints:", len(waypoints))


# ==============================================================
#                 CONNECT TO DRONE
# ==============================================================

def connectMyCopter():
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect')
    args = parser.parse_args()

    vehicle = connect(args.connect, baud=57600, wait_ready=True)
    return vehicle


vehicle = connectMyCopter()


# ==============================================================
#                 CUSTOM HOME LOCATION (EDIT HERE)
# ==============================================================

HOME_LAT = 16.565980   # 🔁 CHANGE TO YOUR DESIRED HOME LATITUDE
HOME_LON = 81.521722   # 🔁 CHANGE TO YOUR DESIRED HOME LONGITUDE
RETURN_ALT = 30        # Altitude while returning


# ==============================================================
#                 HELPER FUNCTIONS
# ==============================================================

def get_distance(cord1, cord2):
    return geopy.distance.geodesic(cord1, cord2).km * 1000


def arm_and_takeoff(target_alt):

    print("Pre-arm checks...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(1)

    if not vehicle.armed:
        print("Arming motors...")
        vehicle.armed = True
        time.sleep(2)

    print("Taking off...")
    vehicle.simple_takeoff(target_alt)

    while vehicle.location.global_relative_frame.alt < target_alt * 0.95:
        print(" Alt:", vehicle.location.global_relative_frame.alt)
        time.sleep(1)

    print("🚁 Takeoff complete.")


def goto_location(lat, lon):

    current_alt = vehicle.location.global_relative_frame.alt
    target = LocationGlobalRelative(lat, lon, current_alt)

    vehicle.simple_goto(target, groundspeed=4)

    target_point = (lat, lon)

    while True:
        current = (
            vehicle.location.global_relative_frame.lat,
            vehicle.location.global_relative_frame.lon
        )

        distance = get_distance(current, target_point)
        print(" Distance:", distance)

        if distance <= 2:
            print("🎯 Waypoint reached.")
            break

        time.sleep(1)


def land_and_wait():

    print("Switching to LAND mode...")
    vehicle.mode = VehicleMode("LAND")

    while vehicle.location.global_relative_frame.alt > 0.2:
        print(" Alt:", vehicle.location.global_relative_frame.alt)
        time.sleep(0.5)

    print("✔ Landed. Waiting 10 seconds...")
    time.sleep(10)


# ==============================================================
# 2️⃣ TAKEOFF
# ==============================================================

takeoff_height = 30

print("\n--- 🚀 Starting Mission ---\n")

arm_and_takeoff(takeoff_height)
time.sleep(2)


# ==============================================================
# 3️⃣ FLY ALL WAYPOINTS
# ==============================================================

for wp_name, wp in waypoints.items():

    # Validate waypoint
    if not wp:
        print(f"Skipping empty waypoint {wp_name}")
        continue

    lat = wp.get("lat")
    lon = wp.get("lon")

    if lat is None or lon is None:
        print(f"Invalid waypoint {wp_name}, skipping...")
        continue

    try:
        lat = float(lat)
        lon = float(lon)
    except:
        print(f"Waypoint {wp_name} format error, skipping...")
        continue

    print(f"\n📍 Flying to {wp_name} → {lat}, {lon}")

    goto_location(lat, lon)
    time.sleep(2)

    land_and_wait()

    # Prepare for next waypoint
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(2)

    arm_and_takeoff(takeoff_height)
    time.sleep(2)


# ==============================================================
# 4️⃣ RETURN TO CUSTOM HOME
# ==============================================================

print("\n🏠 All waypoints completed → Returning to CUSTOM HOME")

vehicle.mode = VehicleMode("GUIDED")
time.sleep(2)

# Ensure proper altitude before travel
if vehicle.location.global_relative_frame.alt < RETURN_ALT:
    arm_and_takeoff(RETURN_ALT)

print(f"Flying to custom home: {HOME_LAT}, {HOME_LON}")

goto_location(HOME_LAT, HOME_LON)
time.sleep(2)

print("Reached custom home. Landing...")

vehicle.mode = VehicleMode("LAND")

while vehicle.location.global_relative_frame.alt > 0.2:
    print(" Alt:", vehicle.location.global_relative_frame.alt)
    time.sleep(1)

print("✅ Custom Home Landing Complete.")


# ==============================================================
# 5️⃣ END
# ==============================================================

print("\n🎉 Mission Finished Successfully.\n")
