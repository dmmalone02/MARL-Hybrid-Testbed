#!/bin/bash

# move_tx_move.sh <episode_number>
# Called by TC_script.sh per episode.
# TX flowgraph (SDR_RF_Hardware_01.py) is started once by TC_script.sh
# at mission start — this script just senses and transmits.

EP=${1:-1}

echo "[1] Sensing..."
SENSING_RAW=$(python3 sensing.py)
# Compact string is always the last non-empty line of sensing.py output
SENSING_OUTPUT=$(echo "$SENSING_RAW" | grep -v '^[[:space:]]*$' | tail -1)
echo "[SENSING] $SENSING_OUTPUT"
sleep 3

echo "[2] Transmitting..."
python3 trigger_episode.py --tx_node 120 --rx_node 100 --ep "$EP"

echo "Done."
