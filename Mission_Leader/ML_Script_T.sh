#!/bin/bash
# HOW TO RUN (called by main controller):
#   ./ML_Script.sh <episode_number> <target_x> <target_y>
#   Example: ./ML_Script.sh 1 5 5

cd "$(dirname "$0")"

REMOTE_USER="ucanlab"

# ── Agent nodes and their IPs ─────────────────────────────────────────────────
# To add a new agent: add its node ID to NODES and its IP to AGENT_HOSTS
# Order must match between the two arrays.
NODES=("103" "104" "183")
AGENT_HOSTS=("10.1.1.103" "10.1.1.104" "STATIC")
STATIC_NODE="183"
# ─────────────────────────────────────────────────────────────────────────────

# ── Arguments ─────────────────────────────────────────────────────────────────
EP=$1
TARGET_X=$2
TARGET_Y=$3

if [ -z "$EP" ] || [ -z "$TARGET_X" ] || [ -z "$TARGET_Y" ]; then
    echo "Usage: ./ML_Script.sh <episode_number> <target_x> <target_y>"
    echo "Example: ./ML_Script.sh 1 5 5"
    exit 1
fi

NUM_AGENTS=${#NODES[@]}

echo "================================================"
echo "Episode $EP | Target ($TARGET_X, $TARGET_Y) | ${NUM_AGENTS}-Agent ML Mission"
echo "================================================"

# ── Locate all agents ─────────────────────────────────────────────────────────
echo ""
echo "Locating agents..."
for i in "${!NODES[@]}"; do
    NODE="${NODES[$i]}"
    # Delete old results file before localizing so stale data cant be read
    rm -f Results/compact_map_result_Ep_*_Node_${NODE}.txt
    python3 mapingfromtxtfile.py --ep "$EP" --node "$NODE"
    if [ $? -ne 0 ]; then
        echo "[WARN] Could not localize Agent $((i+1)) (node $NODE)"
    fi
done
echo "Located agents."

# ── Decide movements for all agents ──────────────────────────────────────────
echo ""
echo "Deciding movements..."
MOVES_CSV=$(python3 ml_Tdecide.py --ep "$EP" --nodes "${NODES[@]}" --target "$TARGET_X" "$TARGET_Y" 2>/dev/null)
echo "ML output: $MOVES_CSV"

# Split CSV into an array of individual moves
IFS=',' read -ra MOVES <<< "$MOVES_CSV"

echo ""
for i in "${!NODES[@]}"; do
    echo "Agent $((i+1)) (node ${NODES[$i]}): ${MOVES[$i]}"
done

# ── Send moves to all agents ──────────────────────────────────────────────────
echo ""
echo "Sending moves to agents..."
for i in "${!NODES[@]}"; do
    NODE="${NODES[$i]}"
    HOST="${AGENT_HOSTS[$i]}"
    MOVE="${MOVES[$i]}"
    if [ "$NODE" = "$STATIC_NODE" ]; then
        echo "Agent $((i+1)) (node $NODE): decision = $MOVE, static agent — skipping SSH move"
        continue
    fi
    if [ "$MOVE" = "hold" ]; then
        echo "Agent $((i+1)) (node $NODE): holding position — skipping move command"
        continue
    fi
    ssh "${REMOTE_USER}@${HOST}" "bash -ic './random_walk_tx.sh start 1 0.24 $MOVE'"
    if [ $? -ne 0 ]; then
        echo "[WARN] SSH to Agent $((i+1)) (node $NODE @ $HOST) failed"
    fi
done

echo ""
echo "================================================"
echo "Done. Episode $EP complete."
echo "================================================"
