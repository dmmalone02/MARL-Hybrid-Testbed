#!/bin/bash

# move_tx_move.sh <episode_number>
# Called by TC_script.sh per episode.

EP=${1:-1}
N=1

# Start TX flowgraph fully detached from the SSH session so it keeps
# running after this script exits and does not block TC_script.sh
nohup python3 SDR_RF_Hardware_01.py -n 120 > /dev/null 2>&1 &
TX_PID=$!
echo "[TX] SDR flowgraph started (PID $TX_PID)"
sleep 5

# Confirm it is still running after startup
if kill -0 "$TX_PID" 2>/dev/null; then
    echo "[TX] SDR flowgraph confirmed running."
else
    echo "[TX] WARNING: SDR flowgraph may have failed — check sdr_tx.log"
fi

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
