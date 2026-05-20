#!/bin/bash
set -e
# random_walk_tx.sh start <num_steps> <distance> <forward|backward|turn_left|turn_right|random>
#
# Examples:
#   ./random_walk_tx.sh start 3 0.5 forward
#   ./random_walk_tx.sh start 5 0.3 random

REMOTE_USER="ubuntu"
REMOTE_HOST="192.168.185.3"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="/home/ubuntu/tb4_motion_ws/install/setup.bash"
PKG_NAME="turtlebot4std"
NODE_NAME="random_walk_step"

COMMAND="$1"
NUM_STEPS="$2"
DISTANCE="$3"
ML_CMD="$4"

case "$COMMAND" in
    start)
        if [ -z "$NUM_STEPS" ] || [ -z "$DISTANCE" ] || [ -z "$ML_CMD" ]; then
            echo "Usage: $0 start <num_steps> <distance> <forward|backward|turn_left|turn_right|random>"
            exit 1
        fi

        echo "─────────────────────────────────────"
        echo "Sending to ${REMOTE_USER}@${REMOTE_HOST}"
        echo "  move:      $ML_CMD"
        echo "  num_steps: $NUM_STEPS"
        echo "  distance:  $DISTANCE m"
        echo "─────────────────────────────────────"

        ssh "${REMOTE_USER}@${REMOTE_HOST}" bash << EOF
            source ${ROS_SETUP}
            source ${WS_SETUP}
            ros2 run ${PKG_NAME} ${NODE_NAME} --ros-args \
                -p num_steps:=${NUM_STEPS} \
                -p distance:=${DISTANCE} \
                -p move:=${ML_CMD}
EOF
        ;;
    *)
        echo "Usage:"
        echo "  $0 start <num_steps> <distance> <forward|backward|turn_left|turn_right|random>"
        exit 1
        ;;
esac
