#!/usr/bin/env python3
import serial
import time

# -------- CONFIG --------
PORT = "/dev/ttyUSB0"   # Linux example
# PORT = "COM5"         # Windows example
BAUD = 115200
# ------------------------

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except serial.SerialException as e:
        print(f"❌ Could not open serial port: {e}")
        return

    # ESP32 auto-reset on serial open
    time.sleep(2)
    print("✅ Serial connected")
    print("Enter: <angle_rad> <speed>   (example: 3.14 9)")
    print("Type q to quit\n")

    while True:
        try:
            line = input("> ").strip()
            if line.lower() in ("q", "quit", "exit"):
                break

            # Split on spaces or commas
            parts = line.replace(",", " ").split()
            if len(parts) != 2:
                print("⚠️  Please enter exactly two values: angle speed")
                continue

            angle = float(parts[0])
            speed = float(parts[1])

            # Send commands exactly as SimpleFOC Commander expects
            ser.write(f"D{angle}\n".encode("utf-8"))
            time.sleep(0.05)  # small gap helps reliability
            ser.write(f"S{speed}\n".encode("utf-8"))

            print(f"➡ Sent: D{angle}  S{speed}")

        except ValueError:
            print("⚠️  Invalid numbers")
        except KeyboardInterrupt:
            break

    ser.close()
    print("🔌 Serial closed")

if __name__ == "__main__":
    main()

