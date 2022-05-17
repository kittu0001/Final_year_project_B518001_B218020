#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Apr  3 19:24:47 2022

@author: rgb
"""

from pynput import keyboard
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import time
import argparse
import Tkinter as tk

print("Connecting.......")


parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

ConnectionString = args.connect
BaudRate = 115200
print("Connecting to vehicle on :  ", ConnectionString)
print("Baudrate   :  " , BaudRate)
vehicle = connect(ConnectionString, baud=BaudRate, wait_ready=True)
print("Connected to the vehicle")

gnd_speed = 1

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

def velocity_control(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def key(event):
    if event.char == event.keysym:
        if event.keysym == 'r':
            print(">>set the vehicle to RTL")
            vehicle.mode = VehicleMode("RTL")
    else:
        if event.keysym == 'Up':
            print("Forward")
            velocity_control(vehicle,gnd_speed,0,0)
        elif event.keysym == 'Down':
            print("backward")
            velocity_control(vehicle,-gnd_speed,0,0)
        elif event.keysym == 'Left':
            print("left")
            velocity_control(vehicle,0,-gnd_speed,0)
        elif event.keysym == 'Right':
            print("right")
            velocity_control(vehicle,0,gnd_speed,0)
        elif event.keysym == 'Alt_L':
            print("down")
            velocity_control(vehicle,0,0,gnd_speed)
        elif event.keysym == 'Alt_R':
            print("up")
            velocity_control(vehicle,0,0,-gnd_speed)

arm_and_takeoff(10)

root = tk.Tk()
print(">>Control drone with keys. Press r for RTL mode")
root.bind_all('<Key>', key)
root.mainloop()
