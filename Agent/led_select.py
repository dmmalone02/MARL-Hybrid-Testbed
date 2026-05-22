#!/usr/bin/env python3
"""
led_select.py
-------------
Randomly selects an LED number, weighted toward a preferred LED.

Usage:
    python3 led_select.py --pref 2 --bias 0.7
    → outputs: 1 or 2

Arguments:
    --pref  : preferred LED (1=Green, 2=Red)
    --bias  : probability of picking the preferred LED (0.0-1.0, default 0.7)
              0.5 = purely random 50/50
              1.0 = always pick preferred
"""

import random
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--pref", type=int, choices=[1, 2], required=True,
                    help="Preferred LED (1=Green, 2=Red)")
parser.add_argument("--bias", type=float, default=0.7,
                    help="Probability of picking preferred LED (default 0.7)")
args = parser.parse_args()

other = 2 if args.pref == 1 else 1
led = args.pref if random.random() < args.bias else other

print(led)
