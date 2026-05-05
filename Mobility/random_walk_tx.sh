#!/bin/bash
set -e
# random_walk_tx.sh start <step num> <distance> <(forward, backward, left, right)>
# random_walk_tx.sh start <step num> <distance> <random>

REMOTE_USER="ubuntu"
REMOTE_HOST="192.168.185.3"

ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="/home/ubuntu/tb4_motion_ws/install/setup.bash"
PKG_NAME="turtlebot4std"

COMMAND="$1"
STEP_NUMBER="$2"
DISTANCE="$3"
ML_CMD="$4"

case "$COMMAND" in
    start)
        if [ -z "$STEP_NUMBER" ] || [ -z "$DISTANCE" ] || [ -z "$ML_CMD" ]; then
            echo "Usage: $0 start <step_number> <distance> <mission leader command>"
            exit 1
        fi
        echo "Sending random_walk_step to ${REMOTE_USER}@${REMOTE_HOST}"

        ssh "${REMOTE_USER}@${REMOTE_HOST}" "bash -ic '
            source ${ROS_SETUP} && \
            source ${WS_SETUP} && \
            ros2 run ${PKG_NAME} random_walk_step --ros-args \
            -p step_number:=\"${STEP_NUMBER}\" \
            -p distance:=\"${DISTANCE}\" \
            -p move:=\"${ML_CMD}\"
        '"
        ;;

    *)
        echo "Usage:"
        echo "$0 start <step num> <distance> <(forward, backward, left, right)>"
        echo "$0 start <step num> <distance> <random>"
        exit 1
        ;;
esac
