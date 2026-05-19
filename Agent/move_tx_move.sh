#!/bin/bash

# move_tx_move.sh <episode_number>
# Called by TC_script.sh per episode.
# TX flowgraph (SDR_RF_Hardware_01.py) is started once by TC_script.sh
# at mission start — this script just senses and transmits.

EP=${1:-1}

echo "[1] Sensing..."
SENSING_OUTPUT=$(python3 sensing.py)
echo "[SENSING] $SENSING_OUTPUT"
sleep 3

echo "[2] Transmitting..."
python3 trigger_episode.py --tx_node 120 --rx_node 100 --ep "$EP"

echo "Done."
