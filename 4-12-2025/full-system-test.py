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
        if -0.03 <= vehicle.velocity[2] <= 0.03 and -0.03 <= vehicle.velocity[2] <= 0.033 and -0.03 <= vehicle.velocity[2] <= 0.03:
          time.sleep(5)
          if -0.03 <= vehicle.velocity[2] <= 0.03 and -0.03 <= vehicle.velocity[2] <= 0.033 and -0.03 <= vehicle.velocity[2] <= 0.03:
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



# Function to setup the telemetry connection
def setup_telem_connection():
    telem_port = "/dev/ttyUSB0"  # Same telemetry module
    baud_rate = 57600  # Same baud rate

    print("Connecting to telemetry module for Pi-to-Pi communication...")
    telem_link = mavutil.mavlink_connection(telem_port, baud=baud_rate)
    print("Telemetry link established, waiting for data...")

    return telem_link




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

telem_link = setup_telem_connection()
print("Telemetry link on pi established")

while True:
    msg = mav.recv_match(type='STATUSTEXT', blocking=True)
    if msg:
        print(f"[{msg.severity}] {msg.text}")
        break

manaul_arm()

home_point = vehicle.location.global_relative_frame

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
        break

time.sleep(10)
start_time = time.time()
while time.time() - start_time < 120:
    if vehicle.gps_0.fix_type != 6:
        print("\nError: GPS does not have RTK Fixed")
        print("GPS STATUS: %s" % vehicle.gps_0.fix_type)
    else:
        break

time.sleep(2)
goto_waypoint(lat,lon,alt, 1)
set_servo_pwm(4, 1000)
time.sleep(9)
print("Finished moving servo")
set_servo_pwm(4, 1500)
time.sleep(1)
print("Moving Forward") 
start_time = time.time()
while time.time() - start_time < 1:
  send_ned_velocity(1,0,0)
print("Returning Home")
goto_waypoint(home_point.lat,home_point.lon,home_point.alt, 2)