import firebase_admin
from firebase_admin import credentials, db

# Load Firebase credentials
cred = credentials.Certificate("dronecred.json")

# Initialize Firebase
firebase_admin.initialize_app(cred, {
    "databaseURL": "https://drone-986f4-default-rtdb.firebaseio.com/"
})

# Reference the waypoint list
ref = db.reference("mission/waypoints")

# Get all waypoint data
waypoints = ref.get()

print("All Waypoints:", waypoints)
print("\n--- Loop Output ---\n")

# Loop through each waypoint
for wp_name, values in waypoints.items():
    alt = values.get("alt")
    lat = values.get("lat")
    lon = values.get("lon")
    
    print(f"{wp_name} -> ALT: {alt}, LAT: {lat}, LON: {lon}")
