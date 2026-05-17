
#!/bin/bash

N=1

REMOTE_USER="ucanlab"
REMOTE_HOST_ML="10.1.1.100"
REMOTE_HOST_AGENT="10.1.1.120"

echo "Starting ML Rx flowgraph.."
echo ""
ssh "${REMOTE_USER}@${REMOTE_HOST_ML}" \
"bash -c 'nohup python3 /home/ucanlab/Mission_Leader/Integrated_Comms_Rx.py > rx.log 2>&1 &'"
sleep 10

for ((ep=1; ep<=N; ep++)); do
    echo "------- Starting Episode $ep -------------"
    echo "Starting Agent script"
    echo ""
    ssh "${REMOTE_USER}@${REMOTE_HOST_AGENT}" \
    "bash -ic './move_tx_move.sh $ep'"&
    sleep 40

    echo ""
    echo "Waiting for agent to finish sensing and tx to ML..."
    sleep 80

    echo "Starting ML script for location..."
    ssh "${REMOTE_USER}@${REMOTE_HOST_ML}" \
    "bash /home/ucanlab/Mission_Leader/ML_Script.sh $ep" &
    sleep 30
    
    echo "--------- Finished Episode $ep ---------"

done

echo "done"
