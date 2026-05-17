#!/bin/bash

cd "$(dirname "$0")"

REMOTE_USER="ucanlab"
REMOTE_HOST="10.1.1.120"

EP=$1

echo "Locating agent..."
python3 mapingfromtxtfile.py --ep "$EP" --node 120
sleep 5
echo "Located agent..." 

echo "Deciding movement..."
MOVES=("forward" "backward" "turn_left" "turn_right")
RANDOM_INDEX=$(( RANDOM % 4 ))
SELECTED_MOVE=${MOVES[$RANDOM_INDEX]}
echo ""
echo "Selected move: $SELECTED_MOVE"

echo "Telling agent movment..."
ssh "${REMOTE_USER}@${REMOTE_HOST}" "bash -ic ' ./random_walk_tx.sh start 1 0.3 $SELECTED_MOVE'" 
 
echo "Done"

