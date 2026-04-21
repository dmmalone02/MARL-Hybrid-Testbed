#!/usr/bin/env bash
#################################################################################################################
#################################################################################################################
##
##  Remote control script for TurtleBot motion commands.
##  Sends ROS2 motion commands to a remote robot over SSH.
##
##  Usage: ./move_agent.sh <command> [args...]
##    move_forward <distance> <speed>
##    rotate <angle_deg> <angular_speed>
##
##  Example:
##    ./move_agent.sh move_forward 0.5 0.15
##    ./move_agent.sh rotate 90 0.5
#################################################################################################################
#################################################################################################################

# ── Configuration ────────────────────────────────────────────────────────────
REMOTE_USER="${REMOTE_USER:-ubuntu}"
REMOTE_HOST="${REMOTE_HOST:-192.168.1.3}"
ROBOT_ID="${ROBOT_ID:-std}"                          # passed as -p xx:= to ROS nodes
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WS_SETUP="${WS_SETUP:-~/ros2_ws/install/setup.bash}"
PKG_NAME="${PKG_NAME:-your_package}"
# ─────────────────────────────────────────────────────────────────────────────

usage() {
    echo ""
    echo "Usage: $0 <command> [args]"
    echo ""
    echo "Commands:"
    echo "  move_forward <distance_m> <speed_m/s>"
    echo "      Example: $0 move_forward 0.5 0.15"
    echo ""
    echo "  rotate <angle_deg> <angular_speed_rad/s>"
    echo "      Example: $0 rotate 90 0.5"
    echo ""
    echo "Environment variables (override defaults):"
    echo "  REMOTE_USER   SSH username          (default: ubuntu)"
    echo "  REMOTE_HOST   Robot IP/hostname     (default: 192.168.185.3)"
    echo "  ROBOT_ID      Robot name suffix     (default: std)"
    echo "  ROS_SETUP     ROS setup.bash path   (default: /opt/ros/jazzy/setup.bash)"
    echo "  WS_SETUP      Workspace setup path  (default: ~/tb4_motion_ws/install/setup.bash)"
    echo "  PKG_NAME      ROS2 package name     (default: tb4_motion)"
    echo ""
    exit 1
}

# Guard: require at least one argument
if [ $# -eq 0 ]; then
    echo "ERROR: No command provided."
    usage
fi

COMMAND="$1"

case "$COMMAND" in
    move_forward)
        if [ $# -ne 3 ]; then
            echo "ERROR: move_forward requires exactly 2 arguments (distance, speed)"
            usage
        fi

        DISTANCE="$2"
        SPEED="$3"

        echo "Sending move_forward to ${REMOTE_USER}@${REMOTE_HOST}"
        echo "  robot=${ROBOT_ID}, distance=${DISTANCE} m, speed=${SPEED} m/s"

        ssh "${REMOTE_USER}@${REMOTE_HOST}" "bash -ic '
            source ${ROS_SETUP} && \
            source ${WS_SETUP} && \
            ros2 run ${PKG_NAME} move_forward --ros-args \
                -p xx:=${ROBOT_ID} \
                -p distance:=${DISTANCE} \
                -p speed:=${SPEED}'"
        ;;

    rotate)
        if [ $# -ne 3 ]; then
            echo "ERROR: rotate requires exactly 2 arguments (angle_deg, angular_speed)"
            usage
        fi

        ANGLE_DEG="$2"
        ANGULAR_SPEED="$3"

        echo "Sending rotate to ${REMOTE_USER}@${REMOTE_HOST}"
        echo "  robot=${ROBOT_ID}, angle=${ANGLE_DEG} deg, angular_speed=${ANGULAR_SPEED} rad/s"

        ssh "${REMOTE_USER}@${REMOTE_HOST}" "bash -ic '
            source ${ROS_SETUP} && \
            source ${WS_SETUP} && \
            ros2 run ${PKG_NAME} rotate --ros-args \
                -p xx:=${ROBOT_ID} \
                -p angle_deg:=${ANGLE_DEG} \
                -p angular_speed:=${ANGULAR_SPEED}'"
        ;;

    -h|--help|help)
        usage
        ;;

    *)
        echo "ERROR: Unknown command: $COMMAND"
        usage
        ;;
esac
