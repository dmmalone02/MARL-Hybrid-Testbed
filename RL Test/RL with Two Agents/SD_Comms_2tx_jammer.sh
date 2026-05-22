#!/bin/bash

TOP_DIR="/home/ucanlab/ucan_TB/TB_Scripts"

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

JAMMER_NODE=${12:-105}
JAM_START_EP=${13:-4}
JAM_NOISE_AMP=${14:-10}
JAM_TX_GAIN=${15:-50}

if [ -z "$EPISODES" ] || [ -z "$ITERATIONS" ]; then
    echo "Usage:"
    echo "$0 <episodes> <iterations> [tx_node_a] [tx_node_b] [rx_node] [mode_a] [mode_b] [beta] [alpha] [x_rf_init] [x_ow_init] [jammer_node] [jam_start_ep] [jam_noise_amp] [jam_tx_gain]"
    exit 1
fi

echo "---------------------------------------"
echo "Starting dual TX RL experiment with jammer"
echo "TX A: $TX_NODE_A mode=$MODE_A"
echo "TX B: $TX_NODE_B mode=$MODE_B"
echo "RX:   $RX_NODE"
echo "JAM:  node=$JAMMER_NODE start_ep=$JAM_START_EP amp=$JAM_NOISE_AMP gain=$JAM_TX_GAIN"
echo "---------------------------------------"

sleep 3

echo "Starting TX flowgraph on node $TX_NODE_A..."
ssh -n ucanlab@10.1.1.$TX_NODE_A \
  "python3 -u $TOP_DIR/Integration_Flow_Graph.py -n $TX_NODE_A" \
  > tx_${TX_NODE_A}.log 2>&1 &

echo "Starting TX flowgraph on node $TX_NODE_B..."
ssh -n ucanlab@10.1.1.$TX_NODE_B \
  "python3 -u $TOP_DIR/T4_Comms_Pluto.py -n $TX_NODE_B" \
  > tx_${TX_NODE_B}.log 2>&1 &

echo "Waiting for flowgraphs to initialize..."
sleep 30

echo "Running RL script on TC..."
python3 /home/ucanlab/Downloads/Testbed_Controller-main/Testbed_Scripts/Configuration/Pi_Scripts/message_trigger_2tx_jammer.py \
  --episodes $EPISODES \
  --iterations $ITERATIONS \
  --tx_nodes $TX_NODE_A $TX_NODE_B \
  --rf_trigger_modes $MODE_A $MODE_B \
  --rx_node $RX_NODE \
  --beta $BETA \
  --alpha $ALPHA \
  --x_rf_init $X_RF_INIT \
  --x_ow_init $X_OW_INIT \
  --jammer_node $JAMMER_NODE \
  --jam_start_ep $JAM_START_EP \
  --jam_noise_amp $JAM_NOISE_AMP \
  --jam_tx_gain $JAM_TX_GAIN

echo "========================================"
echo "Experiment complete"
echo "========================================"