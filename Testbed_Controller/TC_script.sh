#!/bin/bash

# ── Mission parameters ────────────────────────────────────────────────────────
# Change these here, or override by passing arguments:
#   ./TC_script.sh <episodes> <target_x> <target_y>
#   Example: ./TC_script.sh 3 5 5
N=${1:-1}
TARGET_X=${2:-5}
TARGET_Y=${3:-5}
# ─────────────────────────────────────────────────────────────────────────────

# ── Hosts ─────────────────────────────────────────────────────────────────────
REMOTE_USER="ucanlab"
REMOTE_HOST_ML="10.1.1.100"

# To add a second agent: add its IP to AGENT_HOSTS
AGENT_HOSTS=("10.1.1.120")
# ─────────────────────────────────────────────────────────────────────────────

# ── Timing (seconds) ──────────────────────────────────────────────────────────
WAIT_RX_STARTUP=10      # Time for Rx flowgraph to start
WAIT_AGENT_TX=40        # Time for agent to finish moving and start transmitting
WAIT_ML_RX=80           # Time for ML Rx to receive full transmission
# ─────────────────────────────────────────────────────────────────────────────

# ── Logging ───────────────────────────────────────────────────────────────────
LOG_FILE="TC_mission_$(date +%Y%m%d_%H%M%S).log"

log() {
    local msg="[$(date '+%H:%M:%S')] $*"
    echo "$msg"
    echo "$msg" >> "$LOG_FILE"
}
# ─────────────────────────────────────────────────────────────────────────────

log "================================================"
log "Mission start | Episodes: $N | Target: ($TARGET_X, $TARGET_Y)"
log "Agents: ${AGENT_HOSTS[*]}"
log "================================================"

log "Starting ML Rx flowgraph..."
ssh "${REMOTE_USER}@${REMOTE_HOST_ML}" \
    "bash -c 'nohup python3 /home/ucanlab/Mission_Leader/Integrated_Comms_Rx.py > rx.log 2>&1 &'"
if [ $? -ne 0 ]; then
    log "[ERROR] Failed to start Rx flowgraph on ML — aborting mission."
    exit 1
fi
log "Rx flowgraph started. Waiting ${WAIT_RX_STARTUP}s for startup..."
sleep "$WAIT_RX_STARTUP"

for ((ep=1; ep<=N; ep++)); do
    log ""
    log "======== Episode $ep / $N | Target ($TARGET_X, $TARGET_Y) ========"

    # ── Trigger all agents ───────────────────────────────────────────────────
    log "Triggering agents..."
    for HOST in "${AGENT_HOSTS[@]}"; do
        log "  Starting agent @ $HOST"
        ssh "${REMOTE_USER}@${HOST}" "bash -ic './move_tx_move.sh $ep'" &
        if [ $? -ne 0 ]; then
            log "  [WARN] SSH to agent @ $HOST may have failed"
        fi
    done

    # ── Wait for agents to finish moving and start transmitting ──────────────
    log "Waiting ${WAIT_AGENT_TX}s for agents to finish moving and start TX..."
    sleep "$WAIT_AGENT_TX"

    # ── Wait for ML Rx to receive full transmission ──────────────────────────
    log "Waiting ${WAIT_ML_RX}s for ML Rx to receive transmission..."
    sleep "$WAIT_ML_RX"

    # ── Run ML localization and decision ─────────────────────────────────────
    log "Running ML script (localize + decide)..."
    ssh "${REMOTE_USER}@${REMOTE_HOST_ML}" \
        "bash /home/ucanlab/Mission_Leader/ML_Script.sh $ep $TARGET_X $TARGET_Y"
    ML_STATUS=$?
    if [ $ML_STATUS -ne 0 ]; then
        log "[WARN] ML script exited with status $ML_STATUS on episode $ep"
    else
        log "ML script completed successfully."
    fi

    log "======== Episode $ep complete ========"
done

log ""
log "================================================"
log "Mission complete. Log saved to $LOG_FILE"
log "================================================"
