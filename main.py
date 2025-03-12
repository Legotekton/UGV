from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import argparse
import os

UGV_IP = "10.42.0.120"  # Update with actual UGV IP
PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UGV_IP, PORT))

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


#manaul_arm()
print("MAIN:  Manual Arm Success")
print("Waiting for GPS coordinates from UAV...")
recived_data = False
lat = 0
while recived_data == False:
  data, addr = sock.recvfrom(1024)
  lat, lon, alt = map(float, data.decode().split(","))
  if lat != 0:
    recived_data = True
    
print(f"Received GPS coordinates: Latitude: {lat}, Longitude: {lon}, Altitude: {alt}")
#vehicle.simple_goto(LocationGlobalRelative(lat, lon, 0), groundspeed=5)
#time.sleep(25)
print("Arrived at the destination.")
exit()