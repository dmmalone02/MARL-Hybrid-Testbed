#!/usr/bin/env python3

import random
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from enum import Enum, auto


def get_position(msg):
    return msg.pose.pose.position.x, msg.pose.pose.position.y


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


class State(Enum):
    IDLE = auto()
    MOVE_FORWARD = auto()
    ROTATE = auto()
    MOVE_BACK_FWD = auto()
    PAUSE = auto()


moves = ["forward", "backward", "left", "right"]


class RandomWalkStep(Node):
    def __init__(self):
        super().__init__('random_walk_step')

        self.declare_parameter('distance', 0.5)
        self.declare_parameter('speed', 0.1)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('step_number', 1)
        self.declare_parameter('pause_sec', 0.5)
        self.declare_parameter('move','random')

        self.target_distance = abs(float(self.get_parameter('distance').value))
        self.speed = abs(float(self.get_parameter('speed').value))
        self.angular_speed = abs(float(self.get_parameter('angular_speed').value))
        self.step_number = int(self.get_parameter('step_number').value)
        self.pause_sec = float(self.get_parameter('pause_sec').value)

        self.cmd_pub = self.create_publisher(
            Twist,
            '/tb_02/cmd_vel_unstamped',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/tb_02/odom',
            self.odom_callback,
            10
        )

        self.current_position = None
        self.current_yaw = None

        self.start_position = None
        self.start_yaw = None
        self.distance_traveled = 0.0
        self.angle_traveled = 0.0

        self.target_angle = 0.0

        self.state = State.IDLE
        self.pause_remaining = 0.0
        self.steps_completed = 0

        self._in_backward = False

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f"Walk node started - {self.step_number} steps planned. Waiting for odometry..."
        )

    def odom_callback(self, msg: Odometry):
        x, y = get_position(msg)
        self.current_position = (x, y)
        self.current_yaw = yaw_from_quat(msg.pose.pose.orientation)

        if self.start_position is not None:
            dx = x - self.start_position[0]
            dy = y - self.start_position[1]
            self.distance_traveled = math.sqrt(dx ** 2 + dy ** 2)

        if self.start_yaw is not None:
            self.angle_traveled = angle_diff(self.current_yaw, self.start_yaw)

    def _reset_linear(self):
       if self.current_position is not None:
           self.start_position = self.current_position
           self.distance_traveled = 0.0

    def _reset_rotation(self, angle_rad: float):
        if self.current_yaw is not None:
            self.target_angle = angle_rad
            self.start_yaw = self.current_yaw
            self.angle_traveled = 0.0

    def start_forward(self):
        self.get_logger().info(f"FORWARD {self.target_distance}")
        self._reset_linear()
        self.state = State.MOVE_FORWARD

    def start_turn_left(self):
        self.get_logger().info("TURN LEFT")
        self._reset_rotation(math.radians(89.0))
        self.state = State.ROTATE

    def start_turn_right(self):
        self.get_logger().info("TURN RIGHT")
        self._reset_rotation(math.radians(-89.0))
        self.state = State.ROTATE

    def start_move_backward(self):
        self.get_logger().info("BACKWARD")
        self._reset_rotation(math.radians(math.pi))
        self.state = State.ROTATE
        self._in_backward = True

    def pick_move(self):
        move_param = self.get_parameter('move').value
        self.get_logger().info(f"move_param is: '{move_param}'")

        if move_param == 'random':
            move = random.choice(moves)
            self.get_logger().info(f"---- Picked move: {move} ----")
        elif move_param in moves:
            move = move_param
            self.get_logger().info(f"Using specified move {move}")

        else:
            self.get_logger().warn(f"Unknown move '{move_param}'")
            move = random.choice(moves)
            self.get_logger().info(f"Picked random move: {move}")

        if move == "forward":
            self.start_forward()
        elif move == "backward":
            self.start_move_backward()
        elif move == "left":
            self.start_turn_left()
        elif move == "right":
            self.start_turn_right()

    def _publish_linear(self, vx: float):
        cmd = Twist()
        cmd.linear.x = vx
        self.cmd_pub.publish(cmd)

    def _publish_angular(self, wz: float):
        cmd = Twist()
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        if self.current_position is None or self.current_yaw is None:
            return

        if self.state == State.IDLE:
            if self.steps_completed >= self.step_number:
                self.get_logger().info(
                    f"All {self.step_number} steps completed. Shutting down."
                )
                print("Walk Done.")
                rclpy.shutdown()
                return

            self.pick_move()
            return

        if self.state == State.PAUSE:
            self.pause_remaining -= 0.1
            if self.pause_remaining <= 0.0:
                self.state = State.IDLE
            return

        if self.state == State.MOVE_FORWARD:
            if self.start_position is None:
                self.r_reset_linear()

            if self.distance_traveled < self.target_distance:
                self._publish_linear(self.speed)
            else:
                self._stop()
                self.get_logger().info("Forward complete.")
                self._begin_pause()
            return

        if self.state == State.ROTATE:
            if self.start_yaw is None:
                self._reset_rotation(self.target_angle)

            if abs(self.angle_traveled) < abs(self.target_angle) - math.radians(2):
                direction = 1.0 if self.target_angle >= 0 else -1.0
                self._publish_angular(direction * self.angular_speed)
            else:
                self._stop()

                if self._in_backward:
                    self.get_logger().info("180 flip done, driving forward")
                    self._in_backward = False
                    self._reset_linear()
                    self.state = State.MOVE_BACK_FWD
                else:
                    self.get_logger().info("Rotation complete.")
                    self._reset_linear()
                    self.state = State.MOVE_FORWARD
            return

        if self.state == State.MOVE_BACK_FWD:
            if self.start_position is None:
                self._reset_linear()

            if self.distance_traveled < self.target_distance:
                self._publish_linear(self.speed)
            else:
                self._stop()
                self.get_logger().info("Backward complete.")
                self._begin_pause()
            return

    def _begin_pause(self):
        self.steps_completed += 1
        self.get_logger().info(
            f"Step {self.steps_completed}/{self.step_number} complete."
        )
        self.pause_remaining = self.pause_sec
        self.state = State.PAUSE

        self.start_position = None
        self.start_yaw = None
        self.distance_traveled = 0.0
        self.angle_traveled = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = RandomWalkStep()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
