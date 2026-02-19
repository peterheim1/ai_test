#!/usr/bin/env python3
"""
ROS2 Steering Controller for MKS-ESP32FOC

Subscribes:
  /drive_module_steering_angle_controller/commands (std_msgs/Float64MultiArray)
    data[0] = motor A angle (radians)
    data[1] = motor B angle (radians)

Sends serial commands to single ESP32 controlling 2 motors.
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

CMD_TIMEOUT_S = 10.0          # Zero steering if no command received
RECONNECT_INTERVAL_S = 2.0    # Retry connection interval
SEND_PERIOD_S = 0.1           # 10 Hz
DEBUG = True
# ==========================================


class SteeringController(Node):
    def __init__(self):
        super().__init__("steering_controller_right")

        self.ser = None
        self.ser_lock = threading.Lock()
        self.last_reconnect_attempt = 0.0

        self.last_cmd = [0.0, 0.0]
        self.last_cmd_time = 0.0
        self.cmd_lock = threading.Lock()

        self.last_sent = [None, None]
        self.last_debug_print = 0.0

        with self.ser_lock:
            self.connect_serial()

        # Serial read thread
        self.read_thread = threading.Thread(
            target=self.read_loop, daemon=True
        )
        self.read_thread.start()

        # ROS interfaces
        self.create_subscription(
            Float64MultiArray,
            "/steering",
            self.cb_steering,
            10,
        )

        self.create_timer(SEND_PERIOD_S, self.tick)

        self.get_logger().info("Steering controller started")
        self.get_logger().info(f"Serial port: {SERIAL_PORT}")

    # -----------------------------------------------------

    def connect_serial(self):
        """Must be called with ser_lock held."""
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

        try:
            self.ser = serial.Serial(
                SERIAL_PORT,
                BAUDRATE,
                timeout=0.1,
                write_timeout=0.05,
            )
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
        self.get_logger().warn("Attempting serial reconnect...")
        self.connect_serial()

    # -----------------------------------------------------

    def cb_steering(self, msg: Float64MultiArray):
        if len(msg.data) >= 2:
            with self.cmd_lock:
                self.last_cmd[0] = msg.data[3]
                self.last_cmd[1] = msg.data[2]
                self.last_cmd_time = time.time()

    # -----------------------------------------------------

    def tick(self):
        now = time.time()

        with self.cmd_lock:
            stale = (now - self.last_cmd_time) > CMD_TIMEOUT_S
            if stale:
                angle_a = 0.0
                angle_b = 0.0
            else:
                angle_a, angle_b = self.last_cmd

        # Do not spam identical commands
        if (
            angle_a == self.last_sent[0]
            and angle_b == self.last_sent[1]
        ):
            return

        packet = f"A{angle_a:.3f}\nB{angle_b:.3f}\n"

        with self.ser_lock:
            if not self.ser or not self.ser.is_open:
                self.try_reconnect()
                return

            try:
                t0 = time.time()
                self.ser.write(packet.encode())
                self.ser.flush()
                dt = time.time() - t0

                self.last_sent = [angle_a, angle_b]

                if DEBUG and (
                    dt > 0.02 or (now - self.last_debug_print) > 1.0
                ):
                    self.get_logger().info(
                        f"Sent A={angle_a:.3f} B={angle_b:.3f} "
                        f"(write {dt*1000:.1f} ms)"
                    )
                    self.last_debug_print = now

            except serial.SerialTimeoutException:
                self.get_logger().warn("Serial write timeout")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

            except OSError as e:
                self.get_logger().warn(f"Serial write error: {e}")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    # -----------------------------------------------------

    def read_loop(self):
        buf = ""

        while rclpy.ok():
            with self.ser_lock:
                ser = self.ser

            if ser and ser.is_open:
                try:
                    n = ser.in_waiting
                    if n > 0:
                        data = ser.read(n).decode(errors="ignore")
                        buf += data
                        while "\n" in buf:
                            line, buf = buf.split("\n", 1)
                            if line.strip():
                                self.get_logger().debug(
                                    f"ESP32: {line.strip()}"
                                )
                except Exception:
                    buf = ""

            time.sleep(0.02)

    # -----------------------------------------------------

    def destroy_node(self):
        with self.ser_lock:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.write(b"A0\nB0\n")
                    self.ser.flush()
                    self.ser.close()
            except Exception:
                pass
            self.ser = None

        super().destroy_node()


# =========================================================

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

