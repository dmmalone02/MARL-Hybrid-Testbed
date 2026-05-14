#!/bin/bash
# HOW TO RUN (called by main controller):
#   ./ML_Script.sh <episode_number>
#   Example: ./ML_Script.sh 1


cd "$(dirname "$0")"


REMOTE_USER="ucanlab"
AGENT1_HOST="10.1.1.120"       # Agent 1 Raspberry Pi IP
AGENT2_HOST="10.1.1.121"       # Agent 2 Raspberry Pi IP - UPDATE THIS


NODE1="120"
NODE2="121"


EP=$1
if [ -z "$EP" ]; then
    echo "Usage: ./ML_Script.sh <episode_number>"
    exit 1
fi


echo "================================================"
echo "Episode $EP | 2-Agent ML Mission"
echo "================================================"


echo ""
echo "Locating agents..."
python3 mapingfromtxtfile.py --ep "$EP" --node "$NODE1"
STATUS1=$?
if [ $STATUS1 -ne 0 ]; then
    echo "[WARN] Could not localize Agent 1 (node $NODE1)"
fi
python3 mapingfromtxtfile.py --ep "$EP" --node "$NODE2"
STATUS2=$?
if [ $STATUS2 -ne 0 ]; then
    echo "[WARN] Could not localize Agent 2 (node $NODE2)"
fi
echo "Located agents."


echo ""
echo "Deciding movements..."
#ML_OUTPUT=$(python3 ml_decide.py --ep "$EP" --nodes "$NODE1" "$NODE2" 2>&1 >/dev/null)
MOVES_CSV=$(python3 ml_decide.py --ep "$EP" --nodes "$NODE1" "$NODE2" 2>/dev/null)
echo "ML output: $MOVES_CSV"
MOVE1=$(echo "$MOVES_CSV" | cut -d',' -f1)
MOVE2=$(echo "$MOVES_CSV" | cut -d',' -f2)
# Validate moves - fallback to forward if empty
if [ -z "$MOVE1" ]; then MOVE1="forward"; fi
if [ -z "$MOVE2" ]; then MOVE2="forward"; fi
echo ""
echo "Agent 1 (node $NODE1): $MOVE1"
echo "Agent 2 (node $NODE2): $MOVE2"


echo ""
echo "Sending moves to agents..."
ssh "${REMOTE_USER}@${AGENT1_HOST}" "bash -ic './random_walk_tx.sh start 1 0.3 $MOVE1'"
SSH1=$?
if [ $SSH1 -ne 0 ]; then
    echo "[WARN] SSH to Agent 1 failed"
fi
ssh "${REMOTE_USER}@${AGENT2_HOST}" "bash -ic './random_walk_tx.sh start 1 0.3 $MOVE2'"
SSH2=$?
if [ $SSH2 -ne 0 ]; then
    echo "[WARN] SSH to Agent 2 failed"
fi


echo "================================================"
echo "Done. Episode $EP complete."
echo "================================================"
