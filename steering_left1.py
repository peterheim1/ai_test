#!/usr/bin/env python3
"""
ROS2 Steering Controller for MKS-ESP32FOC

Subscribes:
  /drive_module_steering_angle_controller/commands (std_msgs/Float64MultiArray)
    data[0] = motor A angle (radians)
    data[1] = motor B angle (radians)

Publishes:
  /angle_left (std_msgs/Float64MultiArray)
    Encoder positions at 10 Hz, matching command array positions

Sends serial commands to single ESP32 controlling 2 motors.
"""

import serial
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


# ================= CONFIG =================
SERIAL_PORT = "/dev/brushless_left"
BAUDRATE = 115200

CMD_TIMEOUT_S = 10.0          # Zero steering if no command received
RECONNECT_INTERVAL_S = 2.0    # Retry connection interval
SEND_PERIOD_S = 0.1           # 10 Hz
DEBUG = True
# ==========================================


class SteeringController(Node):
    def __init__(self):
        super().__init__("steering_controller_left")

        self.ser = None
        self.ser_lock = threading.Lock()
        self.last_reconnect_attempt = 0.0

        self.last_cmd = [0.0, 0.0]
        self.last_cmd_time = 0.0
        self.cmd_lock = threading.Lock()

        self.last_sent = [None, None]
        self.last_debug_print = 0.0

        # Encoder feedback storage (A, B)
        self.encoder_angles = [0.0, 0.0]
        self.encoder_lock = threading.Lock()

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

        # Publisher for encoder feedback
        self.angle_pub = self.create_publisher(
            Float64MultiArray,
            "angle_left",
            10,
        )

        self.create_timer(SEND_PERIOD_S, self.tick)
        self.create_timer(0.1, self.publish_angles)  # 10 Hz

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
                self.last_cmd[0] = msg.data[0]
                self.last_cmd[1] = msg.data[1]
                self.last_cmd_time = time.time()

    # -----------------------------------------------------

    def publish_angles(self):
        """Publish encoder angles at 10 Hz."""
        msg = Float64MultiArray()
        # Match command array positions: index 0 = A, index 1 = B
        with self.encoder_lock:
            msg.data = [self.encoder_angles[0], self.encoder_angles[1], 0.0, 0.0]
        self.angle_pub.publish(msg)

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
                            line = line.strip()
                            if line:
                                self.parse_encoder_line(line)
                                self.get_logger().debug(f"ESP32: {line}")
                except Exception:
                    buf = ""

            time.sleep(0.02)

    def parse_encoder_line(self, line: str):
        """
        Parse encoder feedback line.
        Format: A0.0000    -0.0158    -0.0000    0.0000
                B0.0000    -0.0068    -0.0000    0.0000
        Extract the last value (4th column) as encoder position.
        """
        if not line or line[0] not in ('A', 'B'):
            return

        motor = line[0]
        try:
            # Split by whitespace, get last value
            parts = line[1:].split()
            if len(parts) >= 4:
                angle = float(parts[3])
                with self.encoder_lock:
                    if motor == 'A':
                        self.encoder_angles[0] = angle
                    else:
                        self.encoder_angles[1] = angle
        except (ValueError, IndexError):
            pass

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
