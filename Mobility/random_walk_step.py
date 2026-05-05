#!/usr/bin/env python3
"""
random_walk.py  –  Turtlebot random walk node
================================================
Randomly selects one of four moves in sequence:
  • move_forward   – drive straight ahead
  • move_backward  – rotate 180°, then drive forward (stays facing new direction)
  • turn_left      – rotate +90 °
  • turn_right     – rotate -90 °

After each move completes the node waits a short pause, then picks the next move.

CLI launch:
  ros2 run <your_package> random_walk \
    --ros-args -p xx:=std -p distance:=0.5 -p speed:=0.1 \
               -p angular_speed:=0.5 -p pause_sec:=0.5 -p num_steps:=5
"""

import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from enum import Enum, auto


# ─────────────────────────────────────────────
#  Helpers
# ─────────────────────────────────────────────

def get_position(msg: Odometry):
    p = msg.pose.pose.position
    return p.x, p.y


def yaw_from_quat(q) -> float:
    """Quaternion → yaw (Z rotation)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a: float, b: float) -> float:
    """Shortest signed angular difference, wrap-safe."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


# ─────────────────────────────────────────────
#  State machine states
# ─────────────────────────────────────────────

class State(Enum):
    IDLE          = auto()   # waiting for first odom / between moves
    MOVE_FORWARD  = auto()   # driving forward
    ROTATE        = auto()   # rotating (left, right, or part of backward)
    MOVE_BACK_FWD = auto()   # driving forward after the 180° flip
    PAUSE         = auto()   # short pause between moves


# ─────────────────────────────────────────────
#  Move primitives (plain data classes)
# ─────────────────────────────────────────────

MOVES = ["forward", "backward", "turn_left", "turn_right"]


# ─────────────────────────────────────────────
#  Node
# ─────────────────────────────────────────────

class RandomWalkStep(Node):

    def __init__(self):
        super().__init__('random_walk_step')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('distance', 0.5)      # metres per forward move
        self.declare_parameter('speed', 0.1)         # m/s
        self.declare_parameter('angular_speed', 0.5) # rad/s
        self.declare_parameter('pause_sec', 0.5)     # seconds between moves
        self.declare_parameter('num_steps', 10)      # total moves before shutdown
        self.declare_parameter('move', 'random')     # forward/backward/turn_left/turn_right/random

        self.target_distance = abs(float(self.get_parameter('distance').value), 0.3)
        self.speed           = abs(float(self.get_parameter('speed').value), 0.3)
        self.angular_speed   = abs(float(self.get_parameter('angular_speed').value), 0.3)
        self.pause_sec       = float(self.get_parameter('pause_sec').value, 1)
        self.num_steps       = int(self.get_parameter('num_steps').value, 1)

        # ── ROS interfaces ───────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(
            Twist, f'/tb_02/cmd_vel_unstamped', 10)

        self.odom_sub = self.create_subscription(
            Odometry, f'/tb_02/odom', self.odom_callback, 10)

        # ── Odometry state ───────────────────────────────────────────────
        self.current_position = None   # (x, y)
        self.current_yaw      = None   # float radians

        # ── Motion tracking (reset each primitive) ───────────────────────
        self.start_position   = None
        self.start_yaw        = None
        self.distance_traveled = 0.0
        self.angle_traveled   = 0.0

        # Target for current primitive
        self.target_angle     = 0.0   # radians (signed)

        # ── State machine ────────────────────────────────────────────────
        self.state            = State.IDLE
        self.pause_remaining  = 0.0   # seconds
        self.steps_completed  = 0     # incremented after each full move

        # ── Timer (10 Hz) ────────────────────────────────────────────────
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"RandomWalk node started – {self.num_steps} steps planned. Waiting for odometry…")

    # ─────────────────────────────────────────
    #  Odometry callback
    # ─────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        x, y = get_position(msg)
        self.current_position = (x, y)
        self.current_yaw = yaw_from_quat(msg.pose.pose.orientation)

        # Update distance from start (if a linear primitive is active)
        if self.start_position is not None:
            dx = x - self.start_position[0]
            dy = y - self.start_position[1]
            self.distance_traveled = math.sqrt(dx ** 2 + dy ** 2)

        # Update angle from start (if a rotation primitive is active)
        if self.start_yaw is not None:
            self.angle_traveled = angle_diff(self.current_yaw, self.start_yaw)

    # ─────────────────────────────────────────
    #  Move initiators
    # ─────────────────────────────────────────

    def _reset_linear(self):
        """Snapshot start position for a forward drive."""
        self.start_position    = self.current_position
        self.distance_traveled = 0.0

    def _reset_rotation(self, angle_rad: float):
        """Snapshot start yaw for a rotation.  angle_rad is signed."""
        self.target_angle    = angle_rad
        self.start_yaw       = self.current_yaw
        self.angle_traveled  = 0.0

    def start_move_forward(self):
        self.get_logger().info(f"▶ FORWARD  {self.target_distance:.2f} m")
        self._reset_linear()
        self.state = State.MOVE_FORWARD

    def start_turn_left(self):
        self.get_logger().info("↺ TURN LEFT  90 °")
        self._reset_rotation(math.radians(90.0))
        self.state = State.ROTATE

    def start_turn_right(self):
        self.get_logger().info("↻ TURN RIGHT  90 °")
        self._reset_rotation(math.radians(-90.0))
        self.state = State.ROTATE

    def start_move_backward(self):
        """Backward = rotate 180° → drive forward (stays facing new direction)."""
        self.get_logger().info("◀ BACKWARD  (flip → drive)")
        self._reset_rotation(math.radians(179.0))
        self.state = State.ROTATE
        self._in_backward = True   # flag so ROTATE knows to drive next

    # ─────────────────────────────────────────
    #  Pick and start a random move
    # ─────────────────────────────────────────

    def _pick_move(self):
        move_param = self.get_parameter('move').value
        if move_param in MOVES:
            move = move_param
        else:
            print(f"Invalid move command: {move_param}")

        self.get_logger().info(
            f"─── Step {self.steps_completed + 1}/{self.num_steps}: {move} ───")
        if move == "forward":
            self.start_move_forward()
        elif move == "backward":
            self.start_move_backward()
        elif move == "turn_left":
            self.start_turn_left()
        elif move == "turn_right":
            self.start_turn_right()

    # ─────────────────────────────────────────
    #  Publish helpers
    # ─────────────────────────────────────────

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

    # ─────────────────────────────────────────
    #  Main control loop (10 Hz)
    # ─────────────────────────────────────────

    def control_loop(self):
        # ── Wait for first odometry ──────────────────────────────────────
        if self.current_position is None or self.current_yaw is None:
            return

        # ── IDLE: pick a move or shut down if steps exhausted ───────────
        if self.state == State.IDLE:
            if self.steps_completed >= self.num_steps:
                self.get_logger().info(
                    f"All {self.num_steps} steps complete. Shutting down.")
                print("WALK_DONE")
                rclpy.shutdown()
                return
            self._pick_move()
            return

        # ── PAUSE ────────────────────────────────────────────────────────
        if self.state == State.PAUSE:
            self.pause_remaining -= 0.1
            if self.pause_remaining <= 0.0:
                self.state = State.IDLE
            return

        # ── MOVE FORWARD ─────────────────────────────────────────────────
        if self.state == State.MOVE_FORWARD:
            if self.distance_traveled < self.target_distance:
                self._publish_linear(self.speed)
            else:
                self._stop()
                self.get_logger().info("✔ Forward complete.")
                self._begin_pause()
            return

        # ── ROTATE (turn_left / turn_right / backward phase 1) ──────────
        if self.state == State.ROTATE:
            if abs(self.angle_traveled) < abs(self.target_angle):
                direction = 1.0 if self.target_angle >= 0 else -1.0
                self._publish_angular(direction * self.angular_speed)
            else:
                self._stop()
                if getattr(self, '_in_backward', False):
                    # 180° flip done — now drive forward
                    self.get_logger().info("  ↳ 180° flip done → driving forward")
                    self._in_backward = False
                    self._reset_linear()
                    self.state = State.MOVE_BACK_FWD
                else:
                    # turn left/right — drive forward after rotating
                    self.get_logger().info("  ↳ Rotation done → driving forward")
                    self._reset_linear()
                    self.state = State.MOVE_FORWARD
            return

        # ── MOVE BACK FWD (backward: drive after 180° flip) ─────────────
        if self.state == State.MOVE_BACK_FWD:
            if self.distance_traveled < self.target_distance:
                self._publish_linear(self.speed)
            else:
                self._stop()
                self.get_logger().info("✔ Backward complete.")
                self._begin_pause()
            return

    def _begin_pause(self):
        self.steps_completed += 1
        self.get_logger().info(
            f"  Step {self.steps_completed}/{self.num_steps} complete.")
        self.pause_remaining = self.pause_sec
        self.state = State.PAUSE
        # Clear tracking so stale data doesn't bleed into next move
        self.start_position  = None
        self.start_yaw       = None
        self.distance_traveled = 0.0
        self.angle_traveled  = 0.0


# ─────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = RandomWalk()
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

