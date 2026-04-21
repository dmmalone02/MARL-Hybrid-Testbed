import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from nav_msgs.msg import Odometry
#####################################################################
# Rotate robot by a specified angle (degrees)
#
# Command line input (CLI):
# ros2 run turtlebot4std rotate_angle \
#   --ros-args -p angle_deg:=90 -p angular_speed:=0.5
#####################################################################


def yaw_from_quat(q):
    """
    Convert quaternion → yaw (Z rotation)
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    """
    Compute shortest angular difference (wrap-safe)
    """
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))

class RotateAngle(Node):
    def __init__(self):
        super().__init__('rotate')

        # Parameters
        self.declare_parameter('xx')  # default robot name
        self.declare_parameter('angle_deg', 90.0)
        self.declare_parameter('angular_speed', 0.5)  # rad/s

        self.target_angle = math.radians(
            self.get_parameter('angle_deg').value
        )

        self.angular_speed = abs(
            self.get_parameter('angular_speed').value
        )

        # Direction (sign of rotation)
        self.direction = 1.0 if self.target_angle >= 0 else -1.0

        # ROS interfaces
        self.cmd_pub = self.create_publisher(
            Twist, '/tb_{xx}/cmd_vel_unstamped', 10
        )

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/tb_{xx}/odom',
            self.odom_callback,
            10
        )

        # State
        self.start_yaw = None
        self.current_yaw = None
        self.angle_traveled = 0.0
        self.done_sent = False

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f"Rotating {math.degrees(self.target_angle):.2f} deg "
            f"at {self.angular_speed:.2f} rad/s"
        )

    def odom_callback(self, msg):
        yaw = yaw_from_quat(msg.pose.pose.orientation)

        if self.start_yaw is None:
            self.start_yaw = yaw
            self.get_logger().info("Received first odometry reading.")

        self.current_yaw = yaw

        # Wrap-safe angle difference
        self.angle_traveled = angle_diff(yaw, self.start_yaw)

    def control_loop(self):
        if self.start_yaw is None or self.current_yaw is None:
            return

        cmd = Twist()

        if abs(self.angle_traveled) < abs(self.target_angle):
            cmd.angular.z = self.direction * self.angular_speed
            self.cmd_pub.publish(cmd)

        else:
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

            if not self.done_sent:
                print("MOTION_DONE")
                self.get_logger().info("Rotation complete.")
                self.done_sent = True

            rclpy.shutdown()
            return


def main(args=None):
    rclpy.init(args=args)
    node = RotateAngle()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
