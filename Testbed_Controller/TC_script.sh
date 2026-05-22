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

# ── Jammer parameters ─────────────────────────────────────────────────────────
# Jamming is optional — only activates if start/stop episodes are passed as args
#   ./TC_script.sh <episodes> <ml_script> <target_x> <target_y> <jam_start> <jam_stop>
#   Example: ./TC_script.sh 5 ML_Script_T.sh 5 6 2 3    (jam episodes 2-3)
#   Example: ./TC_script.sh 5 ML_Script_T.sh 5 6         (no jamming)
JAMMER_START_EP=${5:-}      # Episode to start jamming (empty = no jamming)
JAMMER_STOP_EP=${6:-}       # Episode to stop jamming (inclusive)
JAMMER_NOISE_AMP=10         # Noise amplitude
JAMMER_TX_GAIN=50           # Transmit gain
# ─────────────────────────────────────────────────────────────────────────────

# ── Hosts ─────────────────────────────────────────────────────────────────────
REMOTE_USER="ucanlab"
REMOTE_HOST_ML="10.1.1.100"
REMOTE_HOST_JAMMER="10.1.1.105"

# RF agents — use SDR_RF_Hardware_01.py flowgraph
AGENT_HOSTS=("10.1.1.103" "10.1.1.104")
AGENT_NODES=("103" "104")

# OWC agent — static node, uses OWC_Push_Button.py flowgraph
OWC_HOST="10.1.1.183"
OWC_NODE="183"
OWC_ENABLED=true   # set to false to disable OWC agent
# ─────────────────────────────────────────────────────────────────────────────

# ── Timing (seconds) ──────────────────────────────────────────────────────────
WAIT_RX_STARTUP=10      # Time for ML Rx flowgraph to start
WAIT_TX_STARTUP=15      # Time for agent TX flowgraph to start
WAIT_ML_RX=10           # Time for ML Rx to fully receive transmission
# ─────────────────────────────────────────────────────────────────────────────

# ── Logging ───────────────────────────────────────────────────────────────────
LOG_DIR="Logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/TC_mission_$(date +%Y%m%d_%H%M%S).log"

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
log_info "  RF Agents : ${AGENT_HOSTS[*]}"
log_info "  OWC Agent : $OWC_HOST (node $OWC_NODE, enabled=$OWC_ENABLED)"
if [ -n "$JAMMER_START_EP" ]; then
    log_info "  Jamming   : episodes $JAMMER_START_EP to $JAMMER_STOP_EP (noise=$JAMMER_NOISE_AMP gain=$JAMMER_TX_GAIN)"
else
    log_info "  Jamming   : disabled"
fi
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
log_info "Rx flowgraph started. Waiting ${WAIT_RX_STARTUP}s for startup..."
sleep "$WAIT_RX_STARTUP"

# ── Start TX flowgraph and emergency stop on each agent once for the whole mission
log_info "Starting TX flowgraph and emergency stop on agents..."
for i in "${!AGENT_HOSTS[@]}"; do
    HOST="${AGENT_HOSTS[$i]}"
    AGENT_NODE="${AGENT_NODES[$i]}"

    ssh -q "${REMOTE_USER}@${HOST}"         "bash -c 'nohup python3 SDR_RF_Hardware_01.py -n ${AGENT_NODE} > tx.log 2>&1 &'"         >> "$LOG_FILE" 2>&1
    if [ $? -ne 0 ]; then
        log_error "Failed to start TX flowgraph on agent @ $HOST — aborting mission."
        exit 1
    else
        log_only "  TX flowgraph started on agent @ $HOST"
    fi
    ssh -q "${REMOTE_USER}@${HOST}"         "bash -c 'nohup python3 T4_STOP.py -n ${AGENT_NODE} > /dev/null 2>&1 &'"         >> "$LOG_FILE" 2>&1
    if [ $? -ne 0 ]; then
        log_error "Failed to start emergency stop on agent @ $HOST"
    else
        log_only "  Emergency stop armed on agent @ $HOST"
    fi
done
# ── Start OWC flowgraph on OWC agent ─────────────────────────────────────────
if [ "$OWC_ENABLED" = true ]; then
    log_info "Starting OWC flowgraph on OWC agent @ $OWC_HOST..."
    ssh -q "${REMOTE_USER}@${OWC_HOST}"         "bash -c 'nohup python3 /home/ucanlab/ucan_TB/TB_Scripts/OWC_Push_Button.py -n $OWC_NODE > owc.log 2>&1 &'"         >> "$LOG_FILE" 2>&1
    if [ $? -ne 0 ]; then
        log_error "Failed to start OWC flowgraph on OWC agent @ $OWC_HOST"
    else
        log_only "  OWC flowgraph started on OWC agent @ $OWC_HOST"
    fi
    ssh -q "${REMOTE_USER}@${OWC_HOST}"         "bash -c 'nohup python3 /home/ucanlab/ucan_TB/TB_Scripts/T4_STOP.py -n $OWC_NODE > /dev/null 2>&1 &'"         >> "$LOG_FILE" 2>&1
    if [ $? -ne 0 ]; then
        log_error "Failed to start emergency stop on agent @ $OWC_HOST"
    else
        log_only "  Emergency stop armed on agent @ $OWC_HOST"
    fi
fi

log_info "TX flowgraphs started. Waiting ${WAIT_TX_STARTUP}s for initialisation..."
sleep "$WAIT_TX_STARTUP"

# ── Episode loop ──────────────────────────────────────────────────────────────
for ((ep=1; ep<=N; ep++)); do
    log_info "======== Episode $ep / $N ========"

    # ── Jammer control ───────────────────────────────────────────────────────
    if [ -n "$JAMMER_START_EP" ] && [ "$ep" -eq "$JAMMER_START_EP" ]; then
        log_info "Starting RF jammer on episode $ep..."
        ssh -q "${REMOTE_USER}@${REMOTE_HOST_JAMMER}"             "bash -c 'nohup python3 RF_Jammer.py -t $JAMMER_NOISE_AMP -g $JAMMER_TX_GAIN > jammer.log 2>&1 &'"             >> "$LOG_FILE" 2>&1
        if [ $? -ne 0 ]; then
            log_error "Failed to start jammer on episode $ep"
        else
            log_info "RF jammer started."
        fi
    fi

    # Trigger all agents synchronously — TX flowgraph already running so
    # move_tx_move.sh exits cleanly after sensing and transmitting
    log_only "Triggering agents..."
    for HOST in "${AGENT_HOSTS[@]}"; do
        log_info "  Agent @ $HOST sensing..."
        AGENT_OUTPUT=$(ssh -q "${REMOTE_USER}@${HOST}"             "bash -ic './move_tx_move.sh $ep'" 2>> "$LOG_FILE")
        if [ $? -ne 0 ]; then
            log_error "Failed to trigger agent @ $HOST on episode $ep"
        else
            SENSE_STR=$(echo "$AGENT_OUTPUT" | grep '^\[SENSING\]' | sed 's/\[SENSING\] //')
            log_info "  Agent @ $HOST sensing: $SENSE_STR"
            log_info "  Agent @ $HOST transmitting..."
            echo "$AGENT_OUTPUT" >> "$LOG_FILE"
        fi
    done
    log_info "  Agent @ $OWC_HOST sensing..."
    OWC_OUTPUT=$(ssh -q "${REMOTE_USER}@${OWC_HOST}"             "bash -ic '/home/ucanlab/ucan_TB/TB_Scripts/move_tx_move_optical.sh $ep'" 2>> "$LOG_FILE")
        if [ $? -ne 0 ]; then
            log_error "Failed to trigger agent @ $OWC_HOST on episode $ep"
        else
            SENSE_OWC_STR=$(echo "$OWC_OUTPUT" | grep '^\[SENSING\]' | sed 's/\[SENSING\] //')
            log_info "  Agent @ $OWC_HOST sensing: $SENSE_OWC_STR"
            log_info "  Agent @ $OWC_HOST transmitting..."
            echo "$OWC_OUTPUT" >> "$LOG_FILE"
        fi
    
    log_info "Waiting ${WAIT_ML_RX}s for ML Rx to receive transmission..."
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

    # ── Stop jammer if this is the stop episode ─────────────────────────────
    if [ -n "$JAMMER_STOP_EP" ] && [ "$ep" -eq "$JAMMER_STOP_EP" ]; then
        log_info "Stopping RF jammer after episode $ep..."
        ssh -q "${REMOTE_USER}@${REMOTE_HOST_JAMMER}"             "pkill -f RF_Jammer.py" >> "$LOG_FILE" 2>&1
        log_info "RF jammer stopped."
    fi

    log_info "======== Episode $ep complete ========"
done

# ── Clean up flowgraphs ──────────────────────────────────────────────────────
log_info "Cleaning up flowgraphs..."

# Kill TX flowgraph on each agent
for HOST in "${AGENT_HOSTS[@]}"; do
    ssh -q "${REMOTE_USER}@${HOST}"         "pkill -f SDR_RF_Hardware_01.py" >> "$LOG_FILE" 2>&1
    ssh -q "${REMOTE_USER}@${HOST}"         "pkill -f T4_STOP.py" >> "$LOG_FILE" 2>&1
    log_only "  TX flowgraph stopped on agent @ $HOST"
    log_only "  Emergency Button stopped on agent @ $HOST"
done

# Kill Rx flowgraph on ML
ssh -q "${REMOTE_USER}@${REMOTE_HOST_ML}"     "pkill -f Integrated_Comms_Rx.py" >> "$LOG_FILE" 2>&1
log_only "  Rx flowgraph stopped on ML"

# Kill jammer if still running (e.g. mission ended early)
ssh -q "${REMOTE_USER}@${REMOTE_HOST_JAMMER}" \
    "pkill -f RF_Jammer.py" >> "$LOG_FILE" 2>&1
log_only "  RF jammer stopped on jammer machine"

# Kill OWC flowgraph
if [ "$OWC_ENABLED" = true ]; then
    ssh -q "${REMOTE_USER}@${OWC_HOST}" \
        "pkill -f OWC_Push_Button.py" >> "$LOG_FILE" 2>&1
    log_only "  OWC flowgraph stopped on OWC agent @ $OWC_HOST"
fi

log_info "================================================"
log_info "Mission complete. Full log: $LOG_FILE"
log_info "================================================"
