#!/usr/bin/env python3
"""
ROS2 DDSM115 RS485 Driver with Tabular Console Output

Subscribes:
  /drive (std_msgs/Float64MultiArray)
    data[0..3] = wheel velocities in m/s (±0.3)

Drives:
  DDSM115 motors via RS485 (/dev/ttyUSB0)

Prints:
  ID | commanded m/s (after reverse) | RPM | current | temp | error
"""

import math
import struct
import time
from typing import List

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


# ================= USER CONFIG =================
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

# Incoming /drive order → motor IDs
MOTOR_IDS: List[int] = [2, 1, 3, 4]

# Reverse direction per motor (same order as MOTOR_IDS)
REVERSE: List[bool] = [True, True, False, False]

MAX_V_MPS = 0.3
WHEEL_RADIUS_M = 0.05     # <<< SET THIS CORRECTLY

SEND_HZ = 20.0
CMD_TIMEOUT_S = 0.25
TEMP_POLL_S = 1.0
# ===============================================


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# CRC-8/MAXIM
def crc8_maxim(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if (crc & 0x01) else (crc >> 1)
    return crc & 0xFF


def build_frame(payload9: bytes) -> bytes:
    return payload9 + bytes([crc8_maxim(payload9)])


def mps_to_rpm(v_mps: float, r: float) -> int:
    if r <= 0:
        return 0
    return int(round((v_mps / (2.0 * math.pi * r)) * 60.0))


# ---------- DDSM115 frames ----------

def frame_speed(mid: int, rpm: int) -> bytes:
    rpm = int(clamp(rpm, -330, 330))
    hi, lo = struct.pack(">h", rpm)
    payload = bytes([mid, 0x64, hi, lo, 0, 0, 0, 0, 0])
    return build_frame(payload)


def frame_query_other(mid: int) -> bytes:
    payload = bytes([mid, 0x74, 0, 0, 0, 0, 0, 0, 0])
    return build_frame(payload)


def decode_proto1(rx: bytes):
    return {
        "id": rx[0],
        "current": struct.unpack(">h", rx[2:4])[0],
        "rpm": struct.unpack(">h", rx[4:6])[0],
        "error": rx[8],
    }


def decode_proto2(rx: bytes):
    return {
        "temp": rx[6],
        "error": rx[8],
    }


class DDSM115DriveNode(Node):
    def __init__(self):
        super().__init__("ddsm115_drive_node")

        self.ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.02,
            write_timeout=0.2,
        )

        self.last_cmd = [0.0] * 4
        self.last_cmd_time = 0.0
        self.last_temp_poll = 0.0
        self.last_temp = ["--"] * 4
        self.print_count = 0

        self.create_subscription(Float64MultiArray, "/drive", self.cb_drive, 10)
        self.create_timer(1.0 / SEND_HZ, self.tick)

        print("\nDDSM115 RS485 Driver Started")
        print(f"Port: {PORT}  Baud: {BAUDRATE}")
        print(f"Motor IDs: {MOTOR_IDS}")
        print(f"Reverse:   {REVERSE}\n")

    def cb_drive(self, msg: Float64MultiArray):
        if len(msg.data) >= 4:
            self.last_cmd = list(msg.data[:4])
            self.last_cmd_time = time.time()

    def tick(self):
        now = time.time()
        stale = (now - self.last_cmd_time) > CMD_TIMEOUT_S
        v_raw = [0.0]*4 if stale else self.last_cmd

        rows = []

        for i, mid in enumerate(MOTOR_IDS):
            v = clamp(v_raw[i], -MAX_V_MPS, MAX_V_MPS)
            if REVERSE[i]:
                v = -v

            rpm_cmd = mps_to_rpm(v, WHEEL_RADIUS_M)

            self.ser.reset_input_buffer()
            self.ser.write(frame_speed(mid, rpm_cmd))
            self.ser.flush()

            rx = self.ser.read(10)
            if len(rx) == 10:
                fb = decode_proto1(rx)
                cur = fb["current"]
                rpm = fb["rpm"]
                err = fb["error"]
            else:
                cur = rpm = err = 0

            rows.append({
                "id": mid,
                "cmd": v,
                "rpm": rpm,
                "cur": cur,
                "temp": self.last_temp[i],
                "err": err,
            })

            time.sleep(0.001)

        # Periodic temperature poll
        if now - self.last_temp_poll > TEMP_POLL_S:
            self.last_temp_poll = now
            for i, mid in enumerate(MOTOR_IDS):
                self.ser.write(frame_query_other(mid))
                self.ser.flush()
                rx = self.ser.read(10)
                if len(rx) == 10:
                    self.last_temp[i] = decode_proto2(rx)["temp"]
                time.sleep(0.002)

        self.print_table(rows)

    def print_table(self, rows):
        self.print_count += 1
        if self.print_count % 20 == 1:
            print(
                "\nID   CMD(m/s)   RPM     CUR     TEMP(C)   ERR\n"
                "--   --------   ----    ----    -------   ---"
            )

        for r in rows:
            print(
                f"{r['id']:<4} "
                f"{r['cmd']:+.3f}     "
                f"{r['rpm']:+4d}    "
                f"{r['cur']:+4d}    "
                f"{str(r['temp']):<7}    "
                f"0x{r['err']:02X}"
            )

    def destroy_node(self):
        try:
            for mid in MOTOR_IDS:
                self.ser.write(frame_speed(mid, 0))
                time.sleep(0.02)
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = DDSM115DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

