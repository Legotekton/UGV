from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import argparse
import os
import socket
import math
from pymavlink import mavutil



# Function to setup the telemetry connection
def setup_telem_connection():
    telem_port = "/dev/ttyUSB0"  # Same telemetry module
    baud_rate = 57600  # Same baud rate

    print("Connecting to telemetry module for Pi-to-Pi communication...")
    telem_link = mavutil.mavlink_connection(telem_port, baud=baud_rate)
    print("Telemetry link established, waiting for data...")

    return telem_link



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





# Main execution
print("MAIN:  Code Started")

vehicle = connectRover()
print("Vehicle connected")

telem_link = setup_telem_connection()

print("Waiting for GPS data...")
while True:
    # Wait for the next GLOBAL_POSITION_INT_COV message
    msg = telem_link.recv_match(type="GLOBAL_POSITION_INT_COV", blocking=True)

    if msg:
        time_usec = msg.time_usec  # Timestamp in microseconds
        estimator_type = msg.estimator_type  # Class id of the estimator this estimate originated from
        lat = msg.lat / 1e7  # Convert back to decimal degrees
        lon = msg.lon / 1e7
        alt = msg.alt / 1000  # Convert back to meters
        relative_alt = msg.relative_alt / 1000 
        vx = msg.vx / 100
        vy = msg.vy / 100
        vz = msg.vz / 100
        covariance = msg.covariance

        print(f"GPS data received: {lat}, {lon}, {alt}")
        print(f"Altitude: {alt}")
        print(f"Relative altitude: {relative_alt}")
        print(f"Velocity: {vx}, {vy}, {vz}")
        print(f"Covariance: {covariance}")

exit()