#!/usr/bin/env python3
"""
Standalone ROS2 node: subscribes to /cmd_vel and prints wheel velocities + angles.

Uses swerve drive inverse kinematics for 4-wheel independent steering.
Wheel order: front_left, rear_left, rear_right, front_right
"""

import math
import numpy as np
from numpy.linalg import pinv
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


# ================= ROBOT GEOMETRY =================
# Distance from robot center to wheel (meters)
# Adjust these for your robot
WHEEL_BASE_X = 0.15   # front/back distance from center
WHEEL_BASE_Y = 0.15   # left/right distance from center

# Wheel positions relative to robot center (x forward, y left)
# Order: front_left, rear_left, rear_right, front_right
WHEEL_POSITIONS = [
    ( WHEEL_BASE_X,  WHEEL_BASE_Y),  # front_left
    (-WHEEL_BASE_X,  WHEEL_BASE_Y),  # rear_left
    (-WHEEL_BASE_X, -WHEEL_BASE_Y),  # rear_right
    ( WHEEL_BASE_X, -WHEEL_BASE_Y),  # front_right
]

WHEEL_NAMES = ["front_left", "rear_left", "rear_right", "front_right"]

# Maximum wheel velocity (m/s) - for scaling if needed
MAX_WHEEL_VELOCITY = 0.5

# Timeout - zero outputs if no cmd_vel received (seconds)
CMD_VEL_TIMEOUT = 2.0

# Deadband - ignore values smaller than this
DEADBAND_LINEAR = 0.01   # m/s
DEADBAND_ANGULAR = 0.01  # rad/s
# ===================================================


@dataclass
class WheelCommand:
    name: str
    steering_angle: float  # radians
    drive_velocity: float  # m/s
    flipped: bool = False  # True if angle was flipped to reduce rotation


def apply_deadband(value: float, deadband: float) -> float:
    """Return 0 if value is within deadband, otherwise return value."""
    return 0.0 if abs(value) < deadband else value


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    angle = angle % (2 * math.pi)
    angle = (angle + 2 * math.pi) % (2 * math.pi)
    if angle > math.pi:
        angle -= 2 * math.pi
    return angle


def angle_difference(target: float, current: float) -> float:
    """Compute shortest signed angle from current to target."""
    diff = normalize_angle(target) - normalize_angle(current)
    if diff > math.pi:
        diff -= 2 * math.pi
    elif diff < -math.pi:
        diff += 2 * math.pi
    return diff


def optimize_wheel_angle(
    target_angle: float,
    target_velocity: float,
    current_angle: float
) -> Tuple[float, float, bool]:
    """
    Optimize steering angle to minimize rotation.

    If rotating >90°, flip angle by 180° and reverse velocity.

    Returns: (optimized_angle, optimized_velocity, was_flipped)
    """
    diff = angle_difference(target_angle, current_angle)

    if abs(diff) > math.pi / 2:
        # Flip: rotate to opposite direction, reverse drive
        flipped_angle = normalize_angle(target_angle + math.pi)
        return flipped_angle, -target_velocity, True
    else:
        return target_angle, target_velocity, False


class SwerveKinematics:
    """Inverse kinematics for 4-wheel swerve drive."""

    def __init__(self, wheel_positions: List[Tuple[float, float]]):
        """
        wheel_positions: list of (x, y) tuples for each wheel
        """
        self.wheel_positions = wheel_positions
        self.num_wheels = len(wheel_positions)

        # Build inverse kinematics matrix
        # Maps [vx, vy, omega] -> [w1_vx, w1_vy, w2_vx, w2_vy, ...]
        arr = []
        for x, y in wheel_positions:
            arr.append([1.0, 0.0, -y])  # vx component
            arr.append([0.0, 1.0,  x])  # vy component
        self.ik_matrix = np.array(arr)

    def compute_wheel_commands(
        self,
        vx: float,
        vy: float,
        omega: float,
        current_angles: List[float] = None,
        max_velocity: float = MAX_WHEEL_VELOCITY
    ) -> List[WheelCommand]:
        """
        Compute wheel steering angles and velocities from body motion.

        Args:
            vx: Forward velocity (m/s)
            vy: Lateral velocity (m/s, positive = left)
            omega: Angular velocity (rad/s, positive = CCW)
            current_angles: Current steering angles for optimization (radians)
            max_velocity: Maximum wheel velocity for scaling

        Returns:
            List of WheelCommand for each wheel
        """
        if current_angles is None:
            current_angles = [0.0] * self.num_wheels

        # If all inputs are zero, return zero angles and velocities
        if abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(omega) < 1e-6:
            return [
                WheelCommand(name=WHEEL_NAMES[i], steering_angle=0.0, drive_velocity=0.0)
                for i in range(self.num_wheels)
            ]

        body_state = np.array([vx, vy, omega])
        wheel_vectors = np.matmul(self.ik_matrix, body_state)

        # Calculate velocities and find scaling factor
        velocities = []
        max_scale = 1.0

        for i in range(self.num_wheels):
            wx = wheel_vectors[2 * i]
            wy = wheel_vectors[2 * i + 1]
            vel = math.sqrt(wx**2 + wy**2)
            velocities.append((wx, wy, vel))

            if vel > max_velocity:
                scale = vel / max_velocity
                if scale > max_scale:
                    max_scale = scale

        # Build wheel commands
        commands = []
        for i in range(self.num_wheels):
            wx, wy, vel = velocities[i]

            # Scale velocity if needed
            vel_scaled = vel / max_scale

            # Calculate steering angle
            if vel < 1e-6:
                # Near zero velocity - keep previous angle
                angle = current_angles[i]
                flipped = False
            else:
                raw_angle = math.atan2(wy, wx)
                angle, vel_scaled, flipped = optimize_wheel_angle(
                    raw_angle, vel_scaled, current_angles[i]
                )

            commands.append(WheelCommand(
                name=WHEEL_NAMES[i],
                steering_angle=normalize_angle(angle),
                drive_velocity=vel_scaled,
                flipped=flipped
            ))

        return commands


class CmdVelToWheelsNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_wheels")

        self.kinematics = SwerveKinematics(WHEEL_POSITIONS)
        self.print_count = 0

        # Track current wheel angles for optimization
        self.current_angles = [0.0] * len(WHEEL_POSITIONS)

        # Track last cmd_vel time for timeout
        self.last_cmd_time = None
        self.timed_out = False

        self.create_subscription(Twist, "/cmd_vel", self.cb_cmd_vel, 10)

        # Publishers for wheel commands
        self.pub_steering = self.create_publisher(Float64MultiArray, "/steering", 10)
        self.pub_drive = self.create_publisher(Float64MultiArray, "/drive", 10)

        # Timer to check for timeout
        self.create_timer(0.1, self.check_timeout)

        self.get_logger().info("cmd_vel_to_wheels node started")
        self.get_logger().info(f"Wheel positions: {WHEEL_POSITIONS}")
        self.get_logger().info(f"Timeout: {CMD_VEL_TIMEOUT}s, Deadband: linear={DEADBAND_LINEAR} angular={DEADBAND_ANGULAR}")
        self.get_logger().info("Publishing: /steering (rad), /drive (m/s)")
        print("\nWaiting for /cmd_vel messages...\n")
        print("Wheel angle optimization ENABLED (flips >90° rotations)\n")

    def cb_cmd_vel(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        self.timed_out = False

        # Apply deadband
        vx = apply_deadband(msg.linear.x, DEADBAND_LINEAR)
        vy = apply_deadband(msg.linear.y, DEADBAND_LINEAR)
        omega = apply_deadband(msg.angular.z, DEADBAND_ANGULAR)

        self.process_and_publish(vx, vy, omega)

    def check_timeout(self):
        """Check if cmd_vel has timed out and publish zeros if so."""
        if self.last_cmd_time is None:
            return

        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9

        if elapsed > CMD_VEL_TIMEOUT and not self.timed_out:
            self.timed_out = True
            self.get_logger().warn(f"cmd_vel timeout ({CMD_VEL_TIMEOUT}s) - stopping")
            self.process_and_publish(0.0, 0.0, 0.0)

    def process_and_publish(self, vx: float, vy: float, omega: float):
        """Compute wheel commands and publish."""
        commands = self.kinematics.compute_wheel_commands(
            vx, vy, omega, self.current_angles
        )

        # Update tracked angles
        for i, cmd in enumerate(commands):
            self.current_angles[i] = cmd.steering_angle

        # Publish steering angles (radians)
        steering_msg = Float64MultiArray()
        steering_msg.data = [cmd.steering_angle for cmd in commands]
        self.pub_steering.publish(steering_msg)

        # Publish drive velocities (m/s)
        drive_msg = Float64MultiArray()
        drive_msg.data = [cmd.drive_velocity for cmd in commands]
        self.pub_drive.publish(drive_msg)

        self.print_table(vx, vy, omega, commands)

    def print_table(self, vx: float, vy: float, omega: float, commands: List[WheelCommand]):
        self.print_count += 1

        # Print header every 20 lines
        if self.print_count % 20 == 1:
            print(f"\n{'─' * 78}")
            print(f"cmd_vel: vx={vx:+.3f} vy={vy:+.3f} omega={omega:+.3f}")
            print(f"{'─' * 78}")
            print(f"{'Wheel':<12} {'Angle (rad)':>12} {'Angle (deg)':>12} {'Velocity (m/s)':>16} {'Flip':>8}")
            print(f"{'─' * 78}")
        else:
            print(f"\ncmd_vel: vx={vx:+.3f} vy={vy:+.3f} omega={omega:+.3f}")

        for cmd in commands:
            angle_deg = math.degrees(cmd.steering_angle)
            flip_str = "FLIP" if cmd.flipped else ""
            print(
                f"{cmd.name:<12} "
                f"{cmd.steering_angle:>+12.4f} "
                f"{angle_deg:>+12.2f} "
                f"{cmd.drive_velocity:>+16.4f} "
                f"{flip_str:>8}"
            )


def main():
    rclpy.init()
    node = CmdVelToWheelsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
