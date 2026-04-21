#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool   

#################################################################################################################
#################################################################################################################
#
##  This node will publish a Twist message to the /tb_std/cmd_vel_unstamped topic to move the robot forward.  
#   The node subscribes to the /tb_std/odom topic to get the robot's current position and calculate the distance traveled.

#################################################################################################################
#################################################################################################################

# Command line input (on turtlebot): ros2 run your_package move_forward --ros-args -p distance:=0.5 -p speed:=0.15
def get_position(msg):
    return msg.pose.pose.position.x, msg.pose.pose.position.y
class MoveForward(Node):
    def __init__(self):
        super().__init__('move_forward')

        # Declare parameters (CLI configurable)
        self.declare_parameter('xx')  # default robot nameeither 01 (for TB Lite) or 02 (for TB Standard)
        self.declare_parameter('distance', 0.5)   # meters
        self.declare_parameter('speed', 0.1)      # m/s

        xx = self.get_parameter('xx').value
        self.target_distance = float(self.get_parameter('distance').value)
        self.speed = abs(float(self.get_parameter('speed').value))

        # ROS interfaces
        self.cmd_pub = self.create_publisher(Twist, f'/tb_{xx}/cmd_vel_unstamped', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/tb_{xx}/odom',
            self.odom_callback,
            10
        )

        # State variables
        self.start_position = None
        self.prev_position = None
        self.current_position = None
        self.distance_traveled = 0.0
        self.done_sent = False

        # Timer loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            f"Moving forward {self.target_distance} m at {self.speed} m/s"
        )

    def odom_callback(self, msg):
        x, y = get_position(msg)
        self.current_position = (x,y)

        if self.prev_position is None:
            self.start_position = (x, y)
            self.prev_position = (x, y)
            self.get_logger().info("Received first odometry reading.")
            return

        # Compute Euclidean distance from start
        dx = x - self.start_position[0]
        dy = y - self.start_position[1]
        step_distance = math.sqrt(dx**2 + dy**2)
        self.distance_traveled += step_distance
        self.prev_position = (x,y)

    def control_loop(self):
        if self.start_position is None or self.current_position is None:
            return  # wait for odometry

        cmd = Twist()

        if self.distance_traveled < self.target_distance:
            cmd.linear.x = self.speed
            slef.cmd_pub.publish(cmd)
        else:
            cmd.linear.x = 0.0
            self.cmd_pub.publish(cmd)

            if not self.done_sent:
                print("MOTION_DONE") 
                self.get_logger().info("Target distance reached. Stopping.")
                self.done_sent = True

            rclpy.shutdown()
            return
        


def main(args=None):
    rclpy.init(args=args)
    node = MoveForward()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
