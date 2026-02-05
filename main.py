from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import math
import socket
import argparse
import geopy.distance
import serial
import numpy as np
import time
import firebase_admin
from firebase_admin import db,credentials
import random

import RPi.GPIO as GPIO
from time import sleep

# -------------------- FIREBASE --------------------
cred = credentials.Certificate("dronecred.json")
firebase_admin.initialize_app(cred, {
    "databaseURL": "https://drone-986f4-default-rtdb.firebaseio.com/"
})

# Read all waypoints
waypoints = db.reference("/mission/waypoints").get()
print("\nFetched Waypoints:", waypoints)


# -------------------- DRONE FUNCTIONS --------------------
def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect
    baud_rate = 57600
    print("\nConnecting to vehicle on:", connection_string)
    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    return vehicle


def get_distance(cord1, cord2):
    return (geopy.distance.geodesic(cord1, cord2).km) * 1000


def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(1)
    vehicle.armed = True
    time.sleep(3)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude:", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def goto_location(to_lat, to_lon):
    print("Going to:", to_lat, to_lon)
    
    curr_alt = vehicle.location.global_relative_frame.alt
    target_point = LocationGlobalRelative(to_lat, to_lon, curr_alt)

    vehicle.simple_goto(target_point, groundspeed=4)

    target_cord = (to_lat, to_lon)
    while True:
        curr_lat = vehicle.location.global_relative_frame.lat
        curr_lon = vehicle.location.global_relative_frame.lon
        curr_cord = (curr_lat, curr_lon)

        dist = get_distance(curr_cord, target_cord)
        print("Distance remaining:", dist)

        if dist <= 2:
            print("Reached within 2m of target")
            break
        time.sleep(1)


def servo():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(11, GPIO.OUT)
    GPIO.setup(13, GPIO.OUT)

    p1 = GPIO.PWM(11, 50)
    p2 = GPIO.PWM(13, 50)

    p1.start(0)
    p2.start(0)

    try:
        p1.ChangeDutyCycle(3)
        sleep(5)
        p2.ChangeDutyCycle(3)
        sleep(20)

        p2.ChangeDutyCycle(12)
        sleep(3)
        p1.ChangeDutyCycle(12)
        sleep(1)

    except KeyboardInterrupt:
        pass

    finally:
        p1.stop()
        p2.stop()
        GPIO.cleanup()


# -------------------- MAIN PROGRAM --------------------
vehicle = connectMyCopter()

home_lat = vehicle.location.global_relative_frame.lat
home_lon = vehicle.location.global_relative_frame.lon

ht = 10   # takeoff height

print("\n--- Starting Mission ---\n")

# TAKEOFF FIRST
arm_and_takeoff(ht)
time.sleep(2)

# LOOP THROUGH ALL WAYPOINTS IN ORDER
for wp_name, wp in waypoints.items():
    lat = wp["lat"]
    lon = wp["lon"]
    alt = wp["alt"]

    print(f"\nFlying to {wp_name}: LAT={lat}, LON={lon}, ALT={alt}")

    goto_location(lat, lon)
    time.sleep(3)

    print(f"Landing at {wp_name}...")
    vehicle.mode = VehicleMode("LAND")
    time.sleep(30)

    servo()

    print(f"Taking off again from {wp_name}...")
    arm_and_takeoff(ht)
    time.sleep(2)

# RETURN HOME
print("\nReturning to Home...")
goto_location(home_lat, home_lon)
time.sleep(3)

vehicle.mode = VehicleMode("LAND")
print("\nMission Completed.\n")
