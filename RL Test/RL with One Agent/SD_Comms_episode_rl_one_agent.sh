#!/bin/bash

TOP_DIR="/home/ucanlab/ucan_TB/TB_Scripts"

EPISODES=$1
ITERATIONS=$2
TX_NODE=${3:-183}
RX_NODE=${4:-182}
BETA=${5:-0.2}
ALPHA=${6:-0.5}
X_RF_INIT=${7:-0.5}
X_OW_INIT=${8:-0.7}

if [ -z "$EPISODES" ] || [ -z "$ITERATIONS" ]; then
    echo "Usage: $0 <episodes> <iterations> [tx_node] [rx_node] [beta] [alpha] [x_rf_init] [x_ow_init]"
    exit 1
fi

#echo "========================================"
#echo "Starting RX evaluator on node $RX_NODE"
#echo "========================================"

#ssh ucanlab@10.1.1.$RX_NODE "
#    cd $TOP_DIR || exit 1
#    pkill -f rx_file_evaluator.py || true
#    nohup python3 rx_file_evaluator.py > rx_eval.log 2>&1 < /dev/null &
#    sleep 3
#    echo '--- process check ---'
#    ps aux | grep rx_file_evaluator.py | grep -v grep || exit 1
#    echo '--- port check ---'
#    ss -ltn | grep ':8082' || exit 1
#    echo '--- log tail ---'
#    tail -n 20 rx_eval.log || true
#"

#if [ $? -ne 0 ]; then
#    echo "ERROR: failed to start rx_file_evaluator.py on node $RX_NODE"
#    exit 1
#fi

#echo "========================================"
#echo "Checking evaluator from node 183"
#echo "========================================"

#ssh ucanlab@10.1.1.183 "
#    for i in 1 2 3 4 5 6 7 8 9 10
#    do
#        python3 -c \"from xmlrpc.client import ServerProxy; s=ServerProxy('http://10.1.1.$RX_NODE:8082/'); print(s.ping())\" && exit 0
#        sleep 1
#    done
#    exit 1
#"

#if [ $? -ne 0 ]; then
#    echo "ERROR: evaluator on node $RX_NODE is not reachable from node 183"
#    exit 1
#fi

echo "========================================"
echo "Starting RL experiment on node $TX_NODE"
echo "========================================"

ssh ucanlab@10.1.1.$TX_NODE "
    cd $TOP_DIR || exit 1
    python3 message_trigger_rl.py \
        --episodes $EPISODES \
        --iterations $ITERATIONS \
        --tx_node $TX_NODE \
        --rx_node $RX_NODE \
        --beta $BETA \
        --alpha $ALPHA \
        --x_rf_init $X_RF_INIT \
        --x_ow_init $X_OW_INIT
"