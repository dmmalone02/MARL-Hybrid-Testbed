#!/usr/bin/env python3
#
# T4_STOP.py
# Waits for button press and kills all agent processes (emergency stop).
# Run once at mission start in the background.
#
# Usage:
#   python3 T4_STOP.py -n 103

from gpiozero import Button
from signal import pause
from time import sleep
import subprocess
import argparse
import sys

parser = argparse.ArgumentParser()
parser.add_argument("-n", "--node", type=int, required=True, help="Node ID")
args = parser.parse_args()

stop_button = Button(5, pull_up=True, bounce_time=0.05)

def emergency_stop():
    print("[STOP] Emergency stop initiated!")

    # Kill GNURadio
    subprocess.run(["pkill", "-f", ".grc"])

    # Kill ROS
    subprocess.run(["pkill", "-f", "ros"])

    # Kill Python scripts (including this one — must be last)
    subprocess.run(["pkill", "-f", ".py"])

print(f"[STOP] Emergency stop armed on node {args.node} — press button to kill all processes.")

stop_button.when_pressed = emergency_stop

pause()
