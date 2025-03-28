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
def goto_waypoint(waypoint, waypoint_number):
    print(f"Going towards waypoint {waypoint_number}...")

    while True:
        current_location = vehicle.location.global_relative_frame
        distance = distance_to(waypoint, current_location)
        vehicle.simple_goto(waypoint, groundspeed=2)

        if distance < 0.3:  # Stop when within 1 meter of the target
            print(f"Reached waypoint {waypoint_number}")
            break

        print(f"Distance to waypoint {waypoint_number}: {distance:.2f}m")
        time.sleep(2)  # Check every second


# Function to set servo PWM
def set_servo_pwm(channel, pwm_value):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # Command
        0,  # Confirmation
        channel,  # Servo channel
        pwm_value,  # PWM value
        0, 0, 0, 0, 0  # Unused parameters
    )                                                                                                                                      
    vehicle.send_mavlink(msg)
    vehicle.flush()
    print(f"Servo {channel} set to {pwm_value} µs")




# Main execution
print("MAIN:  Code Started")

vehicle = connectRover()
print("Vehicle connected")

manaul_arm()

waypoints = [
  LocationGlobalRelative(27.9866475, -82.3017249, 0)
]

for i, waypoint in enumerate(waypoints):
    goto_waypoint(waypoint, i + 1)

# Example: Move servo on Channel 4 to 1500µs
set_servo_pwm(4, 1900)
time.sleep(5)
print("Finished moving servo")
set_servo_pwm(4, 981)
time.sleep(1)

exit()