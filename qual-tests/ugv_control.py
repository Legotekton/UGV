from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import argparse
import os
import socket
import math

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
def goto_waypoint(waypoint, waypoint_number):
    print(f"Going towards waypoint {waypoint_number}...")
    vehicle.simple_goto(waypoint, groundspeed=2)

    while True:
        current_location = vehicle.location.global_relative_frame
        distance = distance_to(waypoint, current_location)

        if distance < 0.5:  # Stop when within 1 meter of the target
            print(f"Reached waypoint {waypoint_number}")
            break

        print(f"Distance to waypoint {waypoint_number}: {distance:.2f}m")
        time.sleep(1)  # Check every second



# Function to calculate new GPS coordinates based on distance and heading
def get_target_location(original_location, distance_meters, heading_degrees):
    """
    Compute a new latitude/longitude given a starting point, distance, and heading.
    """
    earth_radius = 6378137.0  # Earth radius in meters

    # Convert latitude/longitude from degrees to radians
    lat1 = math.radians(original_location.lat)
    lon1 = math.radians(original_location.lon)

    # Convert heading to radians
    heading = math.radians(heading_degrees)

    # Calculate new latitude
    lat2 = math.asin(math.sin(lat1) * math.cos(distance_meters / earth_radius) +
                     math.cos(lat1) * math.sin(distance_meters / earth_radius) * math.cos(heading))

    # Calculate new longitude
    lon2 = lon1 + math.atan2(math.sin(heading) * math.sin(distance_meters / earth_radius) * math.cos(lat1),
                             math.cos(distance_meters / earth_radius) - math.sin(lat1) * math.sin(lat2))

    # Convert back to degrees
    return LocationGlobalRelative(math.degrees(lat2), math.degrees(lon2), original_location.alt)



# Main execution
print("MAIN:  Code Started")

vehicle = connectRover()
print("Vehicle connected")

manaul_arm()

start_location = vehicle.location.global_relative_frame

target_location = get_target_location(start_location, 27.4, vehicle.heading)  

goto_waypoint(target_location, 1)

exit()