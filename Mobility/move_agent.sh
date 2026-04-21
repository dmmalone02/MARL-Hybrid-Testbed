COMMAND="$1"


case "$COMMAND" in
    move_forward)
        if [ $# -ne 3 ]; then 
             echo "ERROR: move_forward requires 2 arguments (distance, speed)"
             usage
        fi
        
        DISTANCE="$2"
        SPEED="$3"
        
        echo "Sending move_forward to ${REMOTE_USER}@${REMOTE_HOST}"
        echo "move_forward distance=${DISTANCE} m, speed=${SPEED} m/s"
        
        ssh "${REMOTE_USER}@${REMOTE_HOST}"  "bash -ic' 
             source ${ROS_SETUP} && \
             source ${WS_SETUP} && \
             ros2 run "$PKG_NAME" move_forward --ros-args \
                 -p distance:=${DISTANCE} \
                 -p speed:=${SPEED}'"
        ;;
        
    rotate)
        if [ $# -ne 3 ]; then 
             echo "ERROR: rotate requires 2 arguments (angular_deg, angular_speed)"
             usage
        fi
        
        ANGLE_DEG="$2"
        ANGULAR_SPEED="$3"
        
        echo "Sending rotate to ${REMOTE_USER}@${REMOTE_HOST}"
        echo "rotate angle_deg=${ANGLE_DEG} degrees, angular speed=${ANGULAR_SPEED} m/s"
       
        ssh "${REMOTE_USER}@${REMOTE_HOST}" "bash -ic '
              source ${ROS_SETUP} && \
              source ${WS_SETUP} && \
              ros2 run "$PKG_NAME" rotate --ros-args \
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
