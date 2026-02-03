#!/usr/bin/env python3
"""
Simple DDSM115 motor drive script

- Hard-wired to /dev/ttyUSB0
- Sends speed command at fixed rate
- CTRL+C stops the motor
"""

import serial
import struct
import time

# ================= USER SETTINGS =================
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
MOTOR_ID = 4
SPEED_RPM = 80        # range: -330 .. +330
SEND_HZ = 20          # command resend rate
# ================================================


# CRC-8/MAXIM (poly 0x31 reflected = 0x8C)
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


def build_speed_frame(motor_id: int, rpm: int) -> bytes:
    """
    Protocol 1 (0x64) speed command
    Frame:
      [ID, 0x64, hi, lo, 0, 0, accel, brake, 0, crc]
    """
    rpm = max(-330, min(330, rpm))
    hi, lo = struct.pack(">h", rpm)

    payload = bytes([
        motor_id,
        0x64,
        hi,
        lo,
        0x00,  # reserved
        0x00,  # reserved
        0x00,  # accel (0 = default)
        0x00,  # brake
        0x00,  # reserved
    ])

    return payload + bytes([crc8_maxim(payload)])


def main():
    print(f"Opening {PORT} @ {BAUDRATE}...")
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=0.05,
        write_timeout=0.2,
    )

    period = 1.0 / SEND_HZ
    frame = build_speed_frame(MOTOR_ID, SPEED_RPM)

    print(f"Driving motor ID {MOTOR_ID} at {SPEED_RPM} RPM")
    print("Press CTRL+C to stop")

    try:
        while True:
            t0 = time.time()
            ser.write(frame)
            ser.flush()

            # Optional: read and discard reply
            ser.read(10)

            dt = time.time() - t0
            time.sleep(max(0.0, period - dt))

    except KeyboardInterrupt:
        print("\nStopping motor...")

        stop_frame = build_speed_frame(MOTOR_ID, 0)
        for _ in range(5):
            ser.write(stop_frame)
            time.sleep(0.05)

    finally:
        ser.close()
        print("Port closed")


if __name__ == "__main__":
    main()

