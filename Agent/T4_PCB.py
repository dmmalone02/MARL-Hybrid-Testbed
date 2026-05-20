#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  rf_led.py
#  
#  Copyright 2026  <ucanlab@raspberrypi>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  

from gpiozero import LED, Button
from signal import pause
from time import sleep
import subprocess 
import argparse

parser = argparse.ArgumentParser()

parser.add_argument(
     "-n",
     "--node",
     type=int,
     required=True,
     help="Node ID"
)

parser.add_argument(
    "-l",
    "--led",
    type=int,
    choices=[1, 2],
    required=True,
    help="LED number to use (1 - Green LED, 2 - Red LED)"
)

args = parser.parse_args()

# LED Setup
if args.led == 1:
    led = LED(17) # GPIO(17) = physical pin 11
if args.led == 2:
    led = LED(27)
    
# STOP Button
stop_button = Button(5, pull_up=True, bounce_time=0.05)

# Node IP Address
TARGET_IP = f"10.1.1.{args.node}"

is_blocked = False

# Kill Functionality
def emergency_stop():
    print("Soft-Stop Initiated")
    
    # Turn off LED
    led.off()
    
    # Kill GNURadio
    subprocess.run(["pkill", "-f", ".grc"])
    
    # Kill Python scripts
    subprocess.run(["pkill", "-f", ".py"])
    
    # Kill ROS (Turtlebot)
    subprocess.run(["pkill", "-f", "ros"])
    
    print("Processes terminated. Pi remains on.")

# Blink LED 

led.on()
sleep(5)
led.off()
    
# Wait for Button
print("Press button to kill processes")

stop_button.when_pressed = emergency_stop

pause()


