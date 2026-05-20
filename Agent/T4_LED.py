#!/usr/bin/env python3
#
# T4_LED.py
# Blinks an LED continuously until the script is killed.
# Called by move_tx_move.sh in the background during transmission.
#
# Usage:
#   python3 T4_LED.py -n 103 -l 2       # blink red LED on node 103
#   python3 T4_LED.py -n 103 -l 1       # blink green LED on node 103

from gpiozero import LED
from time import sleep
import argparse
import signal
import sys

parser = argparse.ArgumentParser()
parser.add_argument("-n", "--node", type=int, required=True, help="Node ID")
parser.add_argument("-l", "--led",  type=int, choices=[1, 2], required=True,
                    help="LED number (1=Green, 2=Red)")
args = parser.parse_args()

# LED setup
led = LED(17) if args.led == 1 else LED(27)

# Clean shutdown on kill signal — turn LED off before exiting
def cleanup(signum, frame):
    led.off()
    sys.exit(0)

signal.signal(signal.SIGTERM, cleanup)
signal.signal(signal.SIGINT, cleanup)

print(f"[LED] Blinking LED {args.led} on node {args.node} — kill process to stop.")

# Blink continuously until killed
try:
    while True:
        led.on()
        sleep(0.5)
        led.off()
        sleep(0.5)
except KeyboardInterrupt:
    led.off()
