import sys
import math
import threading

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QGridLayout,
    QLabel, QPushButton, QDial
)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Wheel order: [front_left, rear_left, rear_right, front_right]
WHEEL_NAMES = ["FL", "RL", "RR", "FR"]

# Maximum velocity in m/s
MAX_VELOCITY = 0.5


class SteeringGuiNode(Node):
    def __init__(self):
        super().__init__('steering_gui')
        self.steering_pub = self.create_publisher(Float64MultiArray, '/steering', 10)
        self.drive_pub = self.create_publisher(Float64MultiArray, '/drive', 10)
        self.get_logger().info('Steering GUI node started')

    def publish_commands(self, angles_deg, velocities, enabled):
        """Publish steering angles (rad) and velocities (m/s) for all wheels."""
        # Wheel order: [FL, RL, RR, FR] -> indices [0, 1, 2, 3]
        steering_msg = Float64MultiArray()
        drive_msg = Float64MultiArray()

        steering_data = []
        drive_data = []

        for i, name in enumerate(WHEEL_NAMES):
            if enabled[name]:
                angle_rad = math.radians(angles_deg)
                velocity = velocities
            else:
                angle_rad = 0.0
                velocity = 0.0

            steering_data.append(angle_rad)
            drive_data.append(velocity)

        steering_msg.data = steering_data
        drive_msg.data = drive_data

        self.steering_pub.publish(steering_msg)
        self.drive_pub.publish(drive_msg)


class MotorGui(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        # Track which wheels are enabled (True = enabled)
        self.wheel_enabled = {name: True for name in WHEEL_NAMES}
        self.current_angle = 0
        self.current_velocity = 0.0
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Swerve Steering Control")
        layout = QVBoxLayout()

        # === TOP DIAL: Steering Angle ±135° ===
        self.label_angle = QLabel("Steering Angle: 0°")
        self.label_angle.setAlignment(Qt.AlignCenter)
        self.dial_angle = QDial()
        self.dial_angle.setMinimum(-135)
        self.dial_angle.setMaximum(135)
        self.dial_angle.setValue(0)
        self.dial_angle.setNotchesVisible(True)
        self.dial_angle.setWrapping(False)
        self.dial_angle.valueChanged.connect(self.on_angle_change)

        layout.addWidget(self.label_angle)
        layout.addWidget(self.dial_angle)

        # === WHEEL ENABLE/DISABLE BUTTONS (2x2 grid) ===
        # Layout matches physical wheel arrangement:
        #   FL  FR
        #   RL  RR
        button_container = QWidget()
        button_layout = QGridLayout()
        button_layout.setSpacing(10)

        self.wheel_buttons = {}

        # Create buttons in wheel layout order
        # Row 0: FL (0,0), FR (0,1)
        # Row 1: RL (1,0), RR (1,1)
        button_positions = {
            "FL": (0, 0),
            "FR": (0, 1),
            "RL": (1, 0),
            "RR": (1, 1),
        }

        for name, (row, col) in button_positions.items():
            btn = QPushButton(f"{name}\nON")
            btn.setCheckable(True)
            btn.setChecked(True)
            btn.setMinimumSize(80, 60)
            btn.setStyleSheet(self.get_button_style(True))
            btn.clicked.connect(lambda checked, n=name: self.on_wheel_toggle(n))
            button_layout.addWidget(btn, row, col)
            self.wheel_buttons[name] = btn

        button_container.setLayout(button_layout)
        layout.addWidget(button_container)

        # === BOTTOM DIAL: Velocity ===
        self.label_velocity = QLabel("Velocity: 0.00 m/s")
        self.label_velocity.setAlignment(Qt.AlignCenter)
        self.dial_velocity = QDial()
        # Scale: -500 to +500 represents -0.5 to +0.5 m/s
        self.dial_velocity.setMinimum(-500)
        self.dial_velocity.setMaximum(500)
        self.dial_velocity.setValue(0)
        self.dial_velocity.setNotchesVisible(True)
        self.dial_velocity.setWrapping(False)
        self.dial_velocity.valueChanged.connect(self.on_velocity_change)

        layout.addWidget(self.label_velocity)
        layout.addWidget(self.dial_velocity)

        # === ZERO BUTTON ===
        self.zero_button = QPushButton("Zero All")
        self.zero_button.clicked.connect(self.on_zero)
        layout.addWidget(self.zero_button)

        self.setLayout(layout)

        # Timer to spin ROS2 and publish at 20 Hz
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.ros_spin)
        self.ros_timer.start(50)  # 20 Hz

    def get_button_style(self, enabled):
        if enabled:
            return """
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    font-weight: bold;
                    border-radius: 5px;
                }
                QPushButton:hover {
                    background-color: #45a049;
                }
            """
        else:
            return """
                QPushButton {
                    background-color: #f44336;
                    color: white;
                    font-weight: bold;
                    border-radius: 5px;
                }
                QPushButton:hover {
                    background-color: #da190b;
                }
            """

    def ros_spin(self):
        """Spin ROS2 and publish current commands."""
        rclpy.spin_once(self.ros_node, timeout_sec=0)
        self.publish_commands()

    def on_angle_change(self, value):
        self.current_angle = value
        self.label_angle.setText(f"Steering Angle: {value}°")

    def on_velocity_change(self, value):
        # Convert from dial units to m/s
        self.current_velocity = value / 1000.0
        self.label_velocity.setText(f"Velocity: {self.current_velocity:.2f} m/s")

    def on_wheel_toggle(self, wheel_name):
        # Toggle the wheel state
        self.wheel_enabled[wheel_name] = not self.wheel_enabled[wheel_name]
        enabled = self.wheel_enabled[wheel_name]

        btn = self.wheel_buttons[wheel_name]
        btn.setText(f"{wheel_name}\n{'ON' if enabled else 'OFF'}")
        btn.setStyleSheet(self.get_button_style(enabled))
        btn.setChecked(enabled)

    def publish_commands(self):
        """Publish steering and velocity commands via ROS2."""
        self.ros_node.publish_commands(
            self.current_angle,
            self.current_velocity,
            self.wheel_enabled
        )

    def on_zero(self):
        # Zero both dials
        self.dial_angle.setValue(0)
        self.dial_velocity.setValue(0)
        self.current_angle = 0
        self.current_velocity = 0.0


def main():
    rclpy.init()
    ros_node = SteeringGuiNode()

    app = QApplication(sys.argv)
    gui = MotorGui(ros_node)
    gui.show()

    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
