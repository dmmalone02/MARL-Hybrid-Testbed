#!/bin/bash

top_dir="/home/ucanlab/ucan_TB/TB_Scripts"

P_RF=$1
EPISODES=$2
ITERATIONS=$3
RX_NODE=${4:-182}

ssh ucanlab@10.1.1.183 "cd $top_dir && \
python3 message_trigger_xmlrpc_rand.py \
  --p_rf $P_RF \
  --episodes $EPISODES \
  --iterations $ITERATIONS \
  --node $RX_NODE"
