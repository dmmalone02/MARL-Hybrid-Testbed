#!/bin/bash

# move_tx_move.sh <episode_number>
# Called by TC_script.sh per episode.
# TX flowgraph (SDR_RF_Hardware_01.py) is started once by TC_script.sh
# at mission start — this script just senses and transmits.

EP=${1:-1}
NODE=183
LED=$(python3 led_select.py --pref 1 --bias 1)
OWC_SENSE="/home/ucanlab/Downloads/test.txt"

echo "[1] Sensing..."
SENSING_OUTPUT=$(cat "$OWC_SENSE" 2>/dev/null | grep -v '^[[:space:]]*$' | tail -1)
echo "[SENSING] $SENSING_OUTPUT"
sleep 3

echo "[2] Transmitting..."
# Start LED blinking during transmission
python3 T4_LED.py -n $NODE -l $LED &
LED_PID=$!

python3 trigger_episode_optical.py --tx_node $NODE --rx_node 100 --ep "$EP"

# Stop LED when transmission is done
kill $LED_PID 2>/dev/null
wait $LED_PID 2>/dev/null

echo "Done."
