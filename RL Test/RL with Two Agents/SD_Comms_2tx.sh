#!/bin/bash

TOP_DIR="/home/ucanlab/ucan_TB/TB_Scripts"

# Arguments
EPISODES=$1
ITERATIONS=$2
TX_NODE_A=${3:-183}
TX_NODE_B=${4:-181}
RX_NODE=${5:-182}
MODE_A=${6:-double}
MODE_B=${7:-single}
BETA=${8:-0.2}
ALPHA=${9:-0.5}
X_RF_INIT=${10:-0.5}
X_OW_INIT=${11:-0.7}

if [ -z "$EPISODES" ] || [ -z "$ITERATIONS" ]; then
    echo "Usage:"
    echo "$0 <episodes> <iterations> [tx_node_a] [tx_node_b] [rx_node] [mode_a] [mode_b] [beta] [alpha] [x_rf_init] [x_ow_init]"
    exit 1
fi

echo "----------------------------------------"
echo "RX Node: $RX_NODE"
echo "TX Nodes: $TX_NODE_A ($MODE_A), $TX_NODE_B ($MODE_B)"
echo "----------------------------------------"

echo "Starting node $TX_NODE_A"
ssh -n ucanlab@10.1.1.$TX_NODE_A \
    "python3 -u /home/ucanlab/ucan_TB/TB_Scripts/Integration_Flow_Graph.py -n $TX_NODE_A" \
    > tx_${TX_NODE_A}.log 2>&1 &

echo "Starting node $TX_NODE_B"
echo "----------------------------------------"
ssh -n ucanlab@10.1.1.$TX_NODE_B \
    "python3 -u /home/ucanlab/ucan_TB/TB_Scripts/T4_Comms_Pluto.py -n $TX_NODE_B" \
    > tx_${TX_NODE_B}.log 2>&1 &

echo "Waiting for flowgraphs..."
sleep 30

# --- Run RL Script ---
python3 /home/ucanlab/Downloads/Testbed_Controller-main/Testbed_Scripts/Configuration/Pi_Scripts/message_trigger_2tx.py \
    --episodes $EPISODES \
    --iterations $ITERATIONS \
    --tx_nodes $TX_NODE_A $TX_NODE_B \
    --rf_trigger_modes $MODE_A $MODE_B \
    --rx_node $RX_NODE \
    --beta $BETA \
    --alpha $ALPHA \
    --x_rf_init $X_RF_INIT \
    --x_ow_init $X_OW_INIT \

echo "----------------------------------------"
echo "Experiment complete"
echo "----------------------------------------"