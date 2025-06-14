from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import argparse
import os
import socket
import math
from pymavlink import mavutil
import logging



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
    time.sleep(3)
    while True:
        time.sleep(1)
        print("Driving...")
        if -0.03 <= vehicle.velocity[0] <= 0.03 and -0.03 <= vehicle.velocity[1] <= 0.03 and -0.03 <= vehicle.velocity[2] <= 0.03:
          time.sleep(1.5)
          if -0.03 <= vehicle.velocity[0] <= 0.03 and -0.03 <= vehicle.velocity[1] <= 0.03 and -0.03 <= vehicle.velocity[2] <= 0.03:
            print("Reached Waypoint %d" % waypoint_number)
            break
          else:
            continue
        else:
          continue




def send_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # velocity only
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()



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

logging.basicConfig(
    filename='ugv_log.txt',
    filemode='w',
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s]: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger()


vehicle = connectRover()
print("Vehicle connected")

manaul_arm()

home_point = vehicle.location.global_relative_frame

lat = 27.986606
lon = -82.3017108
alt = 18.63

time.sleep(2)
goto_waypoint(lat,lon,alt, 1)

logger.info(f"GPS data recieved {lat},{lon},{alt}!")

time.sleep(7)
goto_waypoint(lat,lon,alt, 1)

set_servo_pwm(4, 1000)
time.sleep(7)
print("Finished moving servo.")
set_servo_pwm(4, 1500)
time.sleep(1)
print("Moving Forward") 
start_time = time.time()
while time.time() - start_time < 1:
  send_ned_velocity(1,0,0)

logger.info("Delivered Payload.")
print("Returning Home")

goto_waypoint(home_point.lat,home_point.lon,home_point.alt, 2)
logger.info("Mission completed.")