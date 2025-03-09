from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import argparse
import os

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

vehicle = connectRover()
print("Vehicle connected")

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

print("MAIN:  Code Started")


manaul_arm()
print("MAIN:  Manual Arm Success")

vehicle.simple_goto(LocationGlobalRelative(28.0597368, -82.4154961, 0), groundspeed=1)
time.sleep(15)
vehicle.simple_goto(LocationGlobalRelative(28.0596067, -82.4155176, 0), groundspeed=1)
time.sleep(15)
exit()