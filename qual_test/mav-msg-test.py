from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import argparse
import os
import socket
import math
from pymavlink import mavutil



# Connect to the Vehicle function
def connectRover():
  print("Start Connection")
  parser = argparse.ArgumentParser(description='commands')
  parser.add_argument('--connect')
  args = parser.parse_args()

  connection_string = "/dev/ttyAMA0"
  baud_rate = 57600

  print("Connecting...")
  vehicle = connect(connection_string,baud=baud_rate) 
  print("GPS: %s" % vehicle.gps_0)
  print("Battery: %s" % vehicle.battery)
  print("Armable?: %s" % vehicle.is_armable)
  print("Mode: %s" % vehicle.mode.name)
  #print("GPS Location: " % vehicle.location.global_frame)    

  return vehicle



# Function to manually arm the vehicle
def manaul_arm():
  print ("    Pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print ("    Waiting for vehicle to initialise...")
    time.sleep(1)

  while not vehicle.armed:
    print ("    Waiting for arming...")
    time.sleep(1)

  print("   Waiting for manual arming...")
  while not vehicle.armed:
    print("   Waiting for arming...")
    time.sleep(1)

  vehicle.mode = VehicleMode("GUIDED")

  print("   Vehicle armed.")
  print("   Mode: %s" % vehicle.mode.name) 



# Function to calculate distance between two GPS coordinates
def distance_to(target_location, current_location):
    dlat = target_location.lat - current_location.lat
    dlong = target_location.lon - current_location.lon
    return math.sqrt((dlat ** 2) + (dlong ** 2)) * 1.113195e5  # Convert lat/lon degrees to meters



# Function to move to a waypoint and check when it is reached
def goto_waypoint(lat,lon, alt, waypoint_number):
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,  # Time since system boot (not used)
        0, 0,  # Target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Use relative altitude
        int(0b0000111111111000),  # Type mask (only positions enabled)
        int(lat * 1e7),  # Latitude (scaled)
        int(lon * 1e7),  # Longitude (scaled)
        alt,  # Altitude (meters, relative)
        0, 0, 0,  # Velocity (not set)
        0, 0, 0,  # Acceleration (not set)
        0, 0  # Yaw, Yaw rate (not set)
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

    while True:
        current_location = vehicle.location.global_relative_frame
        distance = distance_to(LocationGlobalRelative(lat, lon, alt), current_location)

        if distance < 0.1:  # Stop when within 1 meter of the target
            print(f"Reached waypoint {waypoint_number}")
            break

        print(f"Distance to waypoint {waypoint_number}: {distance:.2f}m")
        time.sleep(0.5)  # Check every second


# Main execution
print("MAIN:  Code Started")

vehicle = connectRover()
print("Vehicle connected")

manaul_arm()

goto_waypoint(27.9866689,-82.30177,18.44, 1)
print("Finished")

exit()