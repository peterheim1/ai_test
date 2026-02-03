#!/usr/bin/env python3
"""
DDSM115 RS485 ID Tool (Waveshare-compatible)

Implements:
- Scan motor IDs using Protocol 1 (0x64) with RPM=0 (reliable even when stopped)
- Set motor ID using the special AA 55 53 ID ... CRC frame
  IMPORTANT: send the ID-set frame FIVE times in a row (per Waveshare wiki)

Usage:
  Scan IDs 1..10:
    python3 ddsm115_id_tool.py --port /dev/ttyUSB0 --scan

  Scan IDs 1..253:
    python3 ddsm115_id_tool.py --port /dev/ttyUSB0 --scan --full

  Set motor ID to 3 (ONLY ONE MOTOR CONNECTED):
    python3 ddsm115_id_tool.py --port /dev/ttyUSB0 --set-id 3

Notes:
- RS485: 115200 8N1
- Frames are 10 bytes
- CRC is CRC-8/MAXIM over bytes [0..8], last byte is CRC.
"""

import argparse
import struct
import time
import serial


# ---------------- Serial settings ----------------
BAUDRATE = 115200
TIMEOUT_S = 0.05
WRITE_TIMEOUT_S = 0.2

DEFAULT_SCAN_MAX = 10  # default scan range 1..10


# ---------------- CRC-8/MAXIM ----------------
# Poly 0x31 reflected => 0x8C, init 0x00, refin/refout True, xorout 0x00
def crc8_maxim(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc & 0xFF


def open_port(port: str) -> serial.Serial:
    return serial.Serial(
        port=port,
        baudrate=BAUDRATE,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=TIMEOUT_S,
        write_timeout=WRITE_TIMEOUT_S,
    )


def build_frame(payload9: bytes) -> bytes:
    """payload9 is bytes[0..8]; returns 10-byte frame with CRC at byte[9]."""
    if len(payload9) != 9:
        raise ValueError("payload9 must be exactly 9 bytes")
    return payload9 + bytes([crc8_maxim(payload9)])


def read_frame10(ser: serial.Serial) -> bytes | None:
    data = ser.read(10)
    if len(data) != 10:
        return None
    if crc8_maxim(data[:9]) != data[9]:
        # CRC mismatch; treat as no valid reply
        return None
    return data


# ---------------- DDSM115 operations ----------------

def probe_id_via_speed0(ser: serial.Serial, motor_id: int) -> bool:
    """
    Probe a motor ID by sending Protocol 1 (0x64) with RPM=0.
    Motor replies with a 10-byte status frame if the ID exists.
    """
    # Protocol 1: [ID, 0x64, val_hi, val_lo, 0, 0, accel, brake, 0, crc]
    rpm = 0
    hi, lo = struct.pack(">h", rpm)  # big endian signed int16
    payload9 = bytes([motor_id, 0x64, hi, lo, 0x00, 0x00, 0x00, 0x00, 0x00])
    frame = build_frame(payload9)

    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()

    rx = read_frame10(ser)
    if rx is None:
        return False

    # rx[0] is the ID that replied
    print(f"✔ Motor found at ID {rx[0]}")
    return True


def scan_ids(ser: serial.Serial, max_id: int) -> list[int]:
    found: list[int] = []
    print(f"🔍 Scanning for motors (1..{max_id})...")
    for mid in range(1, max_id + 1):
        if probe_id_via_speed0(ser, mid):
            found.append(mid)
        time.sleep(0.01)
    if not found:
        print("❌ No motors found")
    return found


def set_motor_id_waveshare(ser: serial.Serial, new_id: int, repeats: int = 5) -> None:
    """
    Set motor ID using Waveshare special frame:

      AA 55 53 ID 00 00 00 00 00 CRC

    Must be sent 5 times in a row (per Waveshare wiki).
    No reply is expected.
    """
    if not (1 <= new_id <= 253):
        raise ValueError("new_id must be in range 1..253")

    payload9 = bytes([0xAA, 0x55, 0x53, new_id, 0x00, 0x00, 0x00, 0x00, 0x00])
    frame = build_frame(payload9)

    print(f"➡ Sending ID-set frame for ID={new_id} ({repeats} times)...")
    ser.reset_input_buffer()

    for i in range(repeats):
        ser.write(frame)
        ser.flush()
        time.sleep(0.03)  # small gap between repeats helps reliability

    print("✅ ID-set frames sent.")
    print("➡ Power-cycle the motor now (recommended), then scan to verify.")


# ---------------- CLI ----------------

def main() -> int:
    ap = argparse.ArgumentParser(description="DDSM115 ID tool (Waveshare AA55 ID set)")
    ap.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyUSB0 or COM5)")
    ap.add_argument("--scan", action="store_true", help=f"Scan IDs 1..{DEFAULT_SCAN_MAX}")
    ap.add_argument("--full", action="store_true", help="With --scan, scan 1..253")
    ap.add_argument("--set-id", type=int, metavar="NEW_ID", help="Set motor ID (ONLY ONE MOTOR CONNECTED)")
    ap.add_argument("--verify", action="store_true", help="After set-id, attempt to verify by probing the new ID")

    args = ap.parse_args()

    ser = open_port(args.port)
    try:
        if args.scan:
            max_id = 253 if args.full else DEFAULT_SCAN_MAX
            scan_ids(ser, max_id)
            return 0

        if args.set_id is not None:
            print("⚠ ONLY ONE MOTOR MUST BE CONNECTED to RS485 when setting ID.")
            time.sleep(0.8)

            set_motor_id_waveshare(ser, args.set_id, repeats=5)

            if args.verify:
                print("🔁 Verifying (probing new ID)...")
                ok = probe_id_via_speed0(ser, args.set_id)
                if not ok:
                    print("❌ Verification failed (likely needs power-cycle).")
                    return 2
                print("✅ Verified.")
            return 0

        ap.print_help()
        return 1

    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())

