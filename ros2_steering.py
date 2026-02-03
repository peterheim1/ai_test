#!/usr/bin/env python3
"""
ROS2 Steering Controller for MKS-ESP32FOC

Subscribes:
  /drive_module_steering_angle_controller/commands (std_msgs/Float64MultiArray)
    data[0] = motor A angle (degrees)
    data[1] = motor B angle (degrees)

Sends serial commands to single ESP32 (right) controlling 2 motors.
"""

import serial
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


# ================= CONFIG =================
SERIAL_PORT = "/dev/brushless_right"
BAUDRATE = 115200

CMD_TIMEOUT_S = 0.5  # Zero steering if no command received
RECONNECT_INTERVAL_S = 2.0  # Retry connection interval
DEBUG = True  # Print values sent to ESP32
# ==========================================


class SteeringController(Node):
    def __init__(self):
        super().__init__("steering_controller")

        self.ser = None
        self.ser_lock = threading.Lock()
        self.last_reconnect_attempt = 0.0
        with self.ser_lock:
            self.connect_serial()

        self.last_cmd = [0.0, 0.0]
        self.last_cmd_time = 0.0
        self.lock = threading.Lock()
        self.last_debug_print = 0.0
        self.last_sent = [None, None]

        # Start serial read thread
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

        # Subscribe to steering commands
        self.create_subscription(
            Float64MultiArray,
            "/drive_module_steering_angle_controller/commands",
            self.cb_steering,
            10
        )

        # Timer to send commands at fixed rate
        self.create_timer(0.05, self.tick)  # 20 Hz

        self.get_logger().info("Steering controller started")
        self.get_logger().info(f"Port: {SERIAL_PORT}")

    def connect_serial(self):
        """Must be called with ser_lock held."""
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
            time.sleep(0.5)  # Wait for ESP32 to initialize after connection
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f"Connected to {SERIAL_PORT}")
            return True
        except Exception as e:
            self.get_logger().error(f"Could not open {SERIAL_PORT}: {e}")
            return False

    def try_reconnect(self):
        """Must be called with ser_lock held."""
        now = time.time()
        if now - self.last_reconnect_attempt < RECONNECT_INTERVAL_S:
            return
        self.last_reconnect_attempt = now
        self.get_logger().info("Attempting to reconnect...")
        self.connect_serial()

    def cb_steering(self, msg: Float64MultiArray):
        if len(msg.data) >= 2:
            with self.lock:
                self.last_cmd = [msg.data[0], msg.data[1]]
                self.last_cmd_time = time.time()

    def tick(self):
        now = time.time()

        with self.lock:
            stale = (now - self.last_cmd_time) > CMD_TIMEOUT_S
            if stale:
                angle_a = 0.0
                angle_b = 0.0
            else:
                angle_a = self.last_cmd[0]
                angle_b = self.last_cmd[1]

        with self.ser_lock:
            if not self.ser:
                self.try_reconnect()
                return

            try:
                cmd_a = f"A{angle_a:.2f}\n"
                cmd_b = f"B{angle_b:.2f}\n"
                self.ser.write(cmd_a.encode())
                self.ser.write(cmd_b.encode())
                if DEBUG:
                    changed = (self.last_sent[0] != angle_a or self.last_sent[1] != angle_b)
                    if changed or (now - self.last_debug_print) >= 1.0:
                        self.get_logger().info(f"Sent: A={angle_a:.3f} B={angle_b:.3f} rad")
                        self.last_debug_print = now
                        self.last_sent = [angle_a, angle_b]
            except OSError as e:
                self.get_logger().warn(f"Serial write error: {e}")
                self.ser = None

    def read_loop(self):
        """Read serial responses (for debugging/feedback)"""
        buf = ""

        while rclpy.ok():
            with self.ser_lock:
                ser = self.ser

            if ser:
                try:
                    if ser.in_waiting > 0:
                        data = ser.read(ser.in_waiting).decode(errors="ignore")
                        buf += data
                        while "\n" in buf:
                            line, buf = buf.split("\n", 1)
                            if line.strip():
                                self.get_logger().debug(f"ESP32: {line.strip()}")
                except OSError:
                    # Port disconnected, will be handled by tick()
                    buf = ""
                except Exception:
                    pass

            time.sleep(0.01)

    def destroy_node(self):
        # Zero steering on shutdown
        with self.ser_lock:
            try:
                if self.ser:
                    self.ser.write(b"A0\n")
                    self.ser.write(b"B0\n")
                    self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = SteeringController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
