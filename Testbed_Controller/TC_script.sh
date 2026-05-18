#!/bin/bash

# ── Mission parameters ────────────────────────────────────────────────────────
# Change these here, or override by passing arguments:
#   ./TC_script.sh <episodes> <ml_script> <target_x> <target_y>
#   Example: ./TC_script.sh 3 ML_Script_T.sh 5 5
#            ./TC_script.sh 3 ML_Script_R.sh       (no target needed)
#
# ML script options:
#   ML_Script_T.sh  — target-directed, needs target_x and target_y
#   ML_Script_R.sh  — random walk, no target needed
#   ML_Script_M.sh  — multi-agent, no target needed
N=${1:-1}
ML_SCRIPT=${2:-ML_Script_T.sh}
TARGET_X=${3:-5}
TARGET_Y=${4:-5}
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

# log_only  — goes to log file only (silent on terminal)
# log_error — goes to both terminal and log file (errors only on terminal)
# log_info  — goes to both terminal and log file (key mission steps)
log_only() {
    echo "[$(date '+%H:%M:%S')] $*" >> "$LOG_FILE"
}
log_error() {
    local msg="[$(date '+%H:%M:%S')] [ERROR] $*"
    echo "$msg"
    echo "$msg" >> "$LOG_FILE"
}
log_info() {
    local msg="[$(date '+%H:%M:%S')] $*"
    echo "$msg"
    echo "$msg" >> "$LOG_FILE"
}
# ─────────────────────────────────────────────────────────────────────────────

# ── Build ML script call based on which script is selected ───────────────────
needs_target() {
    case "$1" in
        ML_Script_T.sh) return 0 ;;
        *)              return 1 ;;
    esac
}
# ─────────────────────────────────────────────────────────────────────────────

log_info "================================================"
log_info "Mission start"
log_info "  Episodes  : $N"
log_info "  ML script : $ML_SCRIPT"
needs_target "$ML_SCRIPT" && log_info "  Target    : ($TARGET_X, $TARGET_Y)"
log_info "  Agents    : ${AGENT_HOSTS[*]}"
log_info "  Log file  : $LOG_FILE"
log_info "================================================"

# ── Start Rx flowgraph ────────────────────────────────────────────────────────
log_info "Starting ML Rx flowgraph..."
ssh -q "${REMOTE_USER}@${REMOTE_HOST_ML}" \
    "bash -c 'nohup python3 /home/ucanlab/Mission_Leader/Integrated_Comms_Rx.py > rx.log 2>&1 &'" \
    >> "$LOG_FILE" 2>&1
if [ $? -ne 0 ]; then
    log_error "Failed to start Rx flowgraph on ML — aborting mission."
    exit 1
fi
log_only "Rx flowgraph started. Waiting ${WAIT_RX_STARTUP}s..."
sleep "$WAIT_RX_STARTUP"

# ── Episode loop ──────────────────────────────────────────────────────────────
for ((ep=1; ep<=N; ep++)); do
    log_info "======== Episode $ep / $N ========"

    # Trigger all agents — sensing string printed to terminal, SSH noise to log
    log_only "Triggering agents..."
    for HOST in "${AGENT_HOSTS[@]}"; do
        SENSE_OUTPUT=$(ssh -q "${REMOTE_USER}@${HOST}" "bash -ic './move_tx_move.sh $ep'" 2>> "$LOG_FILE")
        if [ $? -ne 0 ]; then
            log_error "Failed to trigger agent @ $HOST on episode $ep"
        else
            log_info "  Agent @ $HOST sensing: $SENSE_OUTPUT"
            echo "$SENSE_OUTPUT" >> "$LOG_FILE"
        fi
    done &

    log_only "Waiting ${WAIT_AGENT_TX}s for agents to finish moving and start TX..."
    sleep "$WAIT_AGENT_TX"

    log_only "Waiting ${WAIT_ML_RX}s for ML Rx to receive transmission..."
    sleep "$WAIT_ML_RX"

    # Run ML script — stdout (location + decision) shown on terminal and logged
    log_only "Running $ML_SCRIPT..."
    if needs_target "$ML_SCRIPT"; then
        ssh -q "${REMOTE_USER}@${REMOTE_HOST_ML}" \
            "bash /home/ucanlab/Mission_Leader/$ML_SCRIPT $ep $TARGET_X $TARGET_Y" \
            2>> "$LOG_FILE" | tee -a "$LOG_FILE"
    else
        ssh -q "${REMOTE_USER}@${REMOTE_HOST_ML}" \
            "bash /home/ucanlab/Mission_Leader/$ML_SCRIPT $ep" \
            2>> "$LOG_FILE" | tee -a "$LOG_FILE"
    fi
    if [ $? -ne 0 ]; then
        log_error "$ML_SCRIPT failed on episode $ep — check $LOG_FILE for details."
    else
        log_only "$ML_SCRIPT completed successfully."
    fi

    log_info "======== Episode $ep complete ========"
done

log_info "================================================"
log_info "Mission complete. Full log: $LOG_FILE"
log_info "================================================"
