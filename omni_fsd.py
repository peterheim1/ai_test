#!/usr/bin/env python3
"""
ROS2 Combined Steering Controller for MKS-ESP32FOC

Controls both left and right ESP32 steering controllers in a single node.

Subscribes:
  /steering (std_msgs/Float64MultiArray)
    data[0] = front_left angle (radians)  -> Left ESP32 motor A
    data[1] = rear_left angle (radians)   -> Left ESP32 motor B
    data[2] = rear_right angle (radians)  -> Right ESP32 motor B
    data[3] = front_right angle (radians) -> Right ESP32 motor A

Publishes:
  /steering_feedback (std_msgs/Float64MultiArray)
    Encoder positions at 10 Hz, same order as commands
    data[0] = front_left encoder
    data[1] = rear_left encoder
    data[2] = rear_right encoder
    data[3] = front_right encoder

Serial format from ESP32:
  A0.0000    -0.0158    -0.0000    0.0000
  B0.0000    -0.0068    -0.0000    0.0000
  (last value is encoder position)
"""

import serial
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


# ================= CONFIG =================
SERIAL_PORT_LEFT = "/dev/brushless_left"
SERIAL_PORT_RIGHT = "/dev/brushless_right"
BAUDRATE = 115200

CMD_TIMEOUT_S = 10.0          # Zero steering if no command received
RECONNECT_INTERVAL_S = 2.0    # Retry connection interval
SEND_PERIOD_S = 0.1           # 10 Hz
PUBLISH_PERIOD_S = 0.1        # 10 Hz for encoder feedback
DEBUG = True
# ==========================================


class SteeringController(Node):
    def __init__(self):
        super().__init__("omni_steering_controller")

        # Serial connections
        self.ser_left = None
        self.ser_right = None
        self.ser_left_lock = threading.Lock()
        self.ser_right_lock = threading.Lock()
        self.last_reconnect_left = 0.0
        self.last_reconnect_right = 0.0

        # Command state
        # [front_left, rear_left, rear_right, front_right]
        self.last_cmd = [0.0, 0.0, 0.0, 0.0]
        self.last_cmd_time = 0.0
        self.cmd_lock = threading.Lock()

        self.last_sent_left = [None, None]   # [A, B] for left ESP32
        self.last_sent_right = [None, None]  # [A, B] for right ESP32
        self.last_debug_print = 0.0

        # Encoder feedback storage
        # [front_left, rear_left, rear_right, front_right]
        self.encoder_angles = [0.0, 0.0, 0.0, 0.0]
        self.encoder_lock = threading.Lock()

        # Connect to serial ports
        with self.ser_left_lock:
            self.connect_serial_left()
        with self.ser_right_lock:
            self.connect_serial_right()

        # Serial read threads
        self.read_thread_left = threading.Thread(
            target=self.read_loop_left, daemon=True
        )
        self.read_thread_right = threading.Thread(
            target=self.read_loop_right, daemon=True
        )
        self.read_thread_left.start()
        self.read_thread_right.start()

        # ROS interfaces
        self.create_subscription(
            Float64MultiArray,
            "/steering",
            self.cb_steering,
            10,
        )

        # Publisher for encoder feedback
        self.feedback_pub = self.create_publisher(
            Float64MultiArray,
            "/steering_feedback",
            10,
        )

        self.create_timer(SEND_PERIOD_S, self.tick)
        self.create_timer(PUBLISH_PERIOD_S, self.publish_feedback)

        self.get_logger().info("Omni steering controller started")
        self.get_logger().info(f"Left serial port: {SERIAL_PORT_LEFT}")
        self.get_logger().info(f"Right serial port: {SERIAL_PORT_RIGHT}")

    # -----------------------------------------------------
    # Serial Connection Management
    # -----------------------------------------------------

    def connect_serial_left(self):
        """Must be called with ser_left_lock held."""
        if self.ser_left:
            try:
                self.ser_left.close()
            except Exception:
                pass
            self.ser_left = None

        try:
            self.ser_left = serial.Serial(
                SERIAL_PORT_LEFT,
                BAUDRATE,
                timeout=0.1,
                write_timeout=0.05,
            )
            self.ser_left.reset_input_buffer()
            self.ser_left.reset_output_buffer()
            self.get_logger().info(f"Connected to {SERIAL_PORT_LEFT}")
            return True
        except Exception as e:
            self.get_logger().error(f"Could not open {SERIAL_PORT_LEFT}: {e}")
            return False

    def connect_serial_right(self):
        """Must be called with ser_right_lock held."""
        if self.ser_right:
            try:
                self.ser_right.close()
            except Exception:
                pass
            self.ser_right = None

        try:
            self.ser_right = serial.Serial(
                SERIAL_PORT_RIGHT,
                BAUDRATE,
                timeout=0.1,
                write_timeout=0.05,
            )
            self.ser_right.reset_input_buffer()
            self.ser_right.reset_output_buffer()
            self.get_logger().info(f"Connected to {SERIAL_PORT_RIGHT}")
            return True
        except Exception as e:
            self.get_logger().error(f"Could not open {SERIAL_PORT_RIGHT}: {e}")
            return False

    def try_reconnect_left(self):
        """Must be called with ser_left_lock held."""
        now = time.time()
        if now - self.last_reconnect_left < RECONNECT_INTERVAL_S:
            return
        self.last_reconnect_left = now
        self.get_logger().warn("Attempting left serial reconnect...")
        self.connect_serial_left()

    def try_reconnect_right(self):
        """Must be called with ser_right_lock held."""
        now = time.time()
        if now - self.last_reconnect_right < RECONNECT_INTERVAL_S:
            return
        self.last_reconnect_right = now
        self.get_logger().warn("Attempting right serial reconnect...")
        self.connect_serial_right()

    # -----------------------------------------------------
    # Command Handling
    # -----------------------------------------------------

    def cb_steering(self, msg: Float64MultiArray):
        if len(msg.data) >= 4:
            with self.cmd_lock:
                self.last_cmd[0] = msg.data[0]  # front_left
                self.last_cmd[1] = msg.data[1]  # rear_left
                self.last_cmd[2] = msg.data[2]  # rear_right
                self.last_cmd[3] = msg.data[3]  # front_right
                self.last_cmd_time = time.time()

    # -----------------------------------------------------
    # Encoder Feedback Publishing
    # -----------------------------------------------------

    def publish_feedback(self):
        """Publish encoder angles at 10 Hz."""
        msg = Float64MultiArray()
        with self.encoder_lock:
            msg.data = list(self.encoder_angles)
        self.feedback_pub.publish(msg)

    # -----------------------------------------------------
    # Main Tick - Send Commands
    # -----------------------------------------------------

    def tick(self):
        now = time.time()

        with self.cmd_lock:
            stale = (now - self.last_cmd_time) > CMD_TIMEOUT_S
            if stale:
                angles = [0.0, 0.0, 0.0, 0.0]
            else:
                angles = list(self.last_cmd)

        # Left ESP32: A = front_left (idx 0), B = rear_left (idx 1)
        left_a = angles[0]
        left_b = angles[1]

        # Right ESP32: A = front_right (idx 3), B = rear_right (idx 2)
        right_a = angles[3]
        right_b = angles[2]

        # Send to left ESP32
        if left_a != self.last_sent_left[0] or left_b != self.last_sent_left[1]:
            self.send_to_left(left_a, left_b, now)

        # Send to right ESP32
        if right_a != self.last_sent_right[0] or right_b != self.last_sent_right[1]:
            self.send_to_right(right_a, right_b, now)

    def send_to_left(self, angle_a, angle_b, now):
        packet = f"A{angle_a:.3f}\nB{angle_b:.3f}\n"

        with self.ser_left_lock:
            if not self.ser_left or not self.ser_left.is_open:
                self.try_reconnect_left()
                return

            try:
                t0 = time.time()
                self.ser_left.write(packet.encode())
                self.ser_left.flush()
                dt = time.time() - t0

                self.last_sent_left = [angle_a, angle_b]

                if DEBUG and (dt > 0.02 or (now - self.last_debug_print) > 1.0):
                    self.get_logger().info(
                        f"Left: A={angle_a:.3f} B={angle_b:.3f} ({dt*1000:.1f}ms)"
                    )
                    self.last_debug_print = now

            except serial.SerialTimeoutException:
                self.get_logger().warn("Left serial write timeout")
                try:
                    self.ser_left.close()
                except Exception:
                    pass
                self.ser_left = None

            except OSError as e:
                self.get_logger().warn(f"Left serial write error: {e}")
                try:
                    self.ser_left.close()
                except Exception:
                    pass
                self.ser_left = None

    def send_to_right(self, angle_a, angle_b, now):
        packet = f"A{angle_a:.3f}\nB{angle_b:.3f}\n"

        with self.ser_right_lock:
            if not self.ser_right or not self.ser_right.is_open:
                self.try_reconnect_right()
                return

            try:
                t0 = time.time()
                self.ser_right.write(packet.encode())
                self.ser_right.flush()
                dt = time.time() - t0

                self.last_sent_right = [angle_a, angle_b]

                if DEBUG and (dt > 0.02 or (now - self.last_debug_print) > 1.0):
                    self.get_logger().info(
                        f"Right: A={angle_a:.3f} B={angle_b:.3f} ({dt*1000:.1f}ms)"
                    )
                    self.last_debug_print = now

            except serial.SerialTimeoutException:
                self.get_logger().warn("Right serial write timeout")
                try:
                    self.ser_right.close()
                except Exception:
                    pass
                self.ser_right = None

            except OSError as e:
                self.get_logger().warn(f"Right serial write error: {e}")
                try:
                    self.ser_right.close()
                except Exception:
                    pass
                self.ser_right = None

    # -----------------------------------------------------
    # Serial Read Loops
    # -----------------------------------------------------

    def read_loop_left(self):
        buf = ""

        while rclpy.ok():
            with self.ser_left_lock:
                ser = self.ser_left

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
                                self.parse_encoder_line_left(line)
                                self.get_logger().debug(f"Left ESP32: {line}")
                except Exception:
                    buf = ""

            time.sleep(0.02)

    def read_loop_right(self):
        buf = ""

        while rclpy.ok():
            with self.ser_right_lock:
                ser = self.ser_right

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
                                self.parse_encoder_line_right(line)
                                self.get_logger().debug(f"Right ESP32: {line}")
                except Exception:
                    buf = ""

            time.sleep(0.02)

    def parse_encoder_line_left(self, line: str):
        """
        Parse encoder feedback from left ESP32.
        Format: A0.0000    -0.0158    -0.0000    0.0000
        Extract the last value (4th column) as encoder position.
        Left: A = front_left (idx 0), B = rear_left (idx 1)
        """
        if not line or line[0] not in ('A', 'B'):
            return

        motor = line[0]
        try:
            parts = line[1:].split()
            if len(parts) >= 4:
                angle = float(parts[3])
                with self.encoder_lock:
                    if motor == 'A':
                        self.encoder_angles[0] = angle  # front_left
                    else:
                        self.encoder_angles[1] = angle  # rear_left
        except (ValueError, IndexError):
            pass

    def parse_encoder_line_right(self, line: str):
        """
        Parse encoder feedback from right ESP32.
        Format: A0.0000    -0.0158    -0.0000    0.0000
        Extract the last value (4th column) as encoder position.
        Right: A = front_right (idx 3), B = rear_right (idx 2)
        """
        if not line or line[0] not in ('A', 'B'):
            return

        motor = line[0]
        try:
            parts = line[1:].split()
            if len(parts) >= 4:
                angle = float(parts[3])
                with self.encoder_lock:
                    if motor == 'A':
                        self.encoder_angles[3] = angle  # front_right
                    else:
                        self.encoder_angles[2] = angle  # rear_right
        except (ValueError, IndexError):
            pass

    # -----------------------------------------------------
    # Cleanup
    # -----------------------------------------------------

    def destroy_node(self):
        # Zero both ESP32s before shutting down
        with self.ser_left_lock:
            try:
                if self.ser_left and self.ser_left.is_open:
                    self.ser_left.write(b"A0\nB0\n")
                    self.ser_left.flush()
                    self.ser_left.close()
            except Exception:
                pass
            self.ser_left = None

        with self.ser_right_lock:
            try:
                if self.ser_right and self.ser_right.is_open:
                    self.ser_right.write(b"A0\nB0\n")
                    self.ser_right.flush()
                    self.ser_right.close()
            except Exception:
                pass
            self.ser_right = None

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
