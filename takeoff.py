

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import argparse

parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

ConnectionString = args.connect
#ConnectionString = "/dev/ttyS0"
#USB on Rasperry Pi = "/dev/ttyACM0"


BaudRate = 115200
#USB can go 115200 or even more(try)
#Serial is not working above 38400  (Debug)


# Connect to the Vehicle
print("Connecting to vehicle on :  %s", ConnectionString)
print("Baudrate   :    %d", BaudRate)

vehicle = connect(ConnectionString, baud=BaudRate, wait_ready=True)
print("Connected to the vehicle")

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

  print( "Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print( " Waiting for vehicle to initialise...")
    time.sleep(1)

  print("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print( " Waiting for arming...")
    time.sleep(1)

  print( "Taking off!")
  time.sleep(5)
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    #Break and return from function just below target altitude.
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
      print("Reached target altitude")
      break
    time.sleep(1)

#initialize the takeoff
arm_and_takeoff(10)

print("Take off complete")

#Hover for 10 seconds
time.sleep(10)

print("Now let's land")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object
vehicle.close()
