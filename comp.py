#!/usr/bin/env python3
"""
ROS2 DDSM115 RS485 Driver (m/s -> RPM)

- Subscribes to /drive (Float64MultiArray)
  data[0..3] = wheel velocity in m/s

- Converts m/s -> RPM for DDSM115
- Applies per-motor reverse
- Prints clean tabular output
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
PORT = "/dev/motors"
BAUDRATE = 115200

# CONTROL ORDER (matches incoming /drive)
MOTOR_IDS: List[int] = [2, 1, 3, 4]

# Reverse direction per motor
REVERSE: List[bool] = [True, True, False, False]

# DISPLAY ORDER (swap motor 2 & 1 visually)
DISPLAY_ORDER: List[int] = [1, 0, 2, 3]

WHEEL_RADIUS_M = 0.05   # <<< SET THIS CORRECTLY

# Limits
MAX_MPS = 0.3           # clamp input m/s
MAX_RPM = 330           # DDSM115 speed limit

SEND_HZ = 20.0
CMD_TIMEOUT_S = 10.0
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


# m/s -> RPM
def mps_to_rpm(mps: float, radius: float) -> int:
    rad_s = mps / radius
    return int(round((rad_s / (2.0 * math.pi)) * 60.0))


# ---------- DDSM115 frames ----------

def frame_speed(mid: int, rpm: int) -> bytes:
    rpm = int(clamp(rpm, -MAX_RPM, MAX_RPM))
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
        "rpm_fb": struct.unpack(">h", rx[4:6])[0],
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

        print("\nDDSM115 RS485 Driver Started (m/s → RPM)")
        print(f"Control order : {MOTOR_IDS}")
        print(f"Display order : {[MOTOR_IDS[i] for i in DISPLAY_ORDER]}")
        print(f"Reverse flags : {REVERSE}\n")

    def cb_drive(self, msg: Float64MultiArray):
        if len(msg.data) >= 4:
            self.last_cmd = list(msg.data[:4])
            self.last_cmd_time = time.time()

    def tick(self):
        now = time.time()
        stale = (now - self.last_cmd_time) > CMD_TIMEOUT_S
        cmd_raw = [0.0] * 4 if stale else self.last_cmd

        rows = []

        for i, mid in enumerate(MOTOR_IDS):
            mps = clamp(cmd_raw[i], -MAX_MPS, MAX_MPS)
            if REVERSE[i]:
                mps = -mps

            rpm_cmd = mps_to_rpm(mps, WHEEL_RADIUS_M)
            rpm_cmd = clamp(rpm_cmd, -MAX_RPM, MAX_RPM)

            self.ser.reset_input_buffer()
            self.ser.write(frame_speed(mid, rpm_cmd))
            self.ser.flush()

            rx = self.ser.read(10)
            if len(rx) == 10:
                fb = decode_proto1(rx)
                cur = fb["current"]
                rpm_fb = fb["rpm_fb"]
                err = fb["error"]
            else:
                cur = rpm_fb = err = 0

            rows.append({
                "id": mid,
                "mps": mps,
                "rpm_cmd": rpm_cmd,
                "rpm_fb": rpm_fb,
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

        #self.print_table(rows)

    def print_table(self, rows):
        self.print_count += 1
        if self.print_count % 20 == 1:
            print(
                "\nID   CMD(m/s)  RPM_CMD  RPM_FB  CUR     TEMP(C)   ERR\n"
                "--   --------  -------  ------  ----    -------   ---"
            )

        for idx in DISPLAY_ORDER:
            r = rows[idx]
            print(
                f"{r['id']:<4} "
                f"{r['mps']:+.3f}      "
                f"{r['rpm_cmd']:+7d}  "
                f"{r['rpm_fb']:+6d}  "
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

