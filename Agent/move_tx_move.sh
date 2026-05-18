#!/bin/bash

# move_tx_move.sh <episode_number>
# Called by TC_script.sh per episode.

EP=${1:-1}
N=1

python3 SDR_RF_Hardware_01.py -n 120 &
sleep 5

for ((i=1; i<=N; i++)); do
    echo "===== Iteration $i ======"

    echo "[1] Sensing..."
    SENSING_OUTPUT=$(python3 sensing.py)
    echo "[SENSING] $SENSING_OUTPUT"
    sleep 3

    echo "[2] Transmitting..."
    python3 trigger_episode.py --tx_node 120 --rx_node 100 --ep "$EP"

    echo "[3] Iteration $i complete"
    sleep 1
done

echo "Done."
