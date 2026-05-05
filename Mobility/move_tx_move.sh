#!/bin/bash

#.move_tx_move.sh {CMD}

N=1

TX_PATH="$HOME/ucanlab"
#FORWARD="./move_agent.sh move_forward 0.5 0.5"
#ROTATE="./move_agent.sh rotate 180.0 0.5"
#RAND_WALK="./random_walk_tx.sh start 1 0.3"

python3 SDR_RF_Hardware_01.py -n 120 &
sleep 5

for ((i=1; i<=N; i++)); do
    echo "===== Iteration $i ======"

    echo "[1] Sensing..."
    python3 sensing.py
    sleep 3

    echo "[2] Transmitting..."
    python3 trigger_episode.py --tx_node 120 --rx_node 100 --ep 1

    echo "[3] Iteration $i complete"
    sleep 1
done

echo "Done."
