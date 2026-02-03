import sys
import serial
import threading
import time

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QDial
)

# === EDIT SERIAL PORT IF NEEDED ===
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

class MotorGui(QWidget):
    def __init__(self):
        super().__init__()
        self.ser = None
        self.init_ui()
        self.connect_serial()
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

    def init_ui(self):
        self.setWindowTitle("BLDC Motor Position Control")
        layout = QVBoxLayout()

        # Motor A dial and label
        self.labelA = QLabel("Motor A: 0.0°")
        self.dialA = QDial()
        self.dialA.setMinimum(-180)
        self.dialA.setMaximum(180)
        self.dialA.setNotchesVisible(True)
        self.dialA.valueChanged.connect(self.on_changeA)

        # Motor B dial and label
        self.labelB = QLabel("Motor B: 0.0°")
        self.dialB = QDial()
        self.dialB.setMinimum(-180)
        self.dialB.setMaximum(180)
        self.dialB.setNotchesVisible(True)
        self.dialB.valueChanged.connect(self.on_changeB)

        # Button to zero motors
        self.zero_button = QPushButton("Zero Motors")
        self.zero_button.clicked.connect(self.on_zero)

        # Arrange elements
        layout.addWidget(self.labelA)
        layout.addWidget(self.dialA)
        layout.addWidget(self.labelB)
        layout.addWidget(self.dialB)
        layout.addWidget(self.zero_button)

        self.setLayout(layout)

        # Timer to refresh labels if needed
        self.timer = QTimer()
        self.timer.timeout.connect(self.redraw)
        self.timer.start(100)  # 10 Hz refresh

    def connect_serial(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
            print(f"Connected to serial {SERIAL_PORT}")
        except Exception as e:
            print("Could not open serial:", e)

    def on_changeA(self, value):
        # Send simple CLI command: e.g. "A45.0\n"
        if self.ser:
            cmd = f"A{value}\n"
            self.ser.write(cmd.encode())

    def on_changeB(self, value):
        if self.ser:
            cmd = f"B{value}\n"
            self.ser.write(cmd.encode())

    def on_zero(self):
        # Zero both dials and send commands
        self.dialA.setValue(0)
        self.dialB.setValue(0)
        if self.ser:
            self.ser.write(b"A0\n")
            self.ser.write(b"B0\n")

    def read_loop(self):
        """ Continuously read serial input for current positions """
        buffer = ""
        while True:
            if self.ser and self.ser.in_waiting > 0:
                try:
                    data = self.ser.read(self.ser.in_waiting).decode(errors="ignore")
                    buffer += data
                    # Process complete lines
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        self.parse_line(line.strip())
                except Exception:
                    pass
            time.sleep(0.01)

    def parse_line(self, line):
        # Example parse: "A: 45.2"
        parts = line.split()
        if len(parts) >= 2:
            if parts[0].startswith("A"):
                try:
                    angle = float(parts[-1])
                    self.labelA.setText(f"Motor A: {angle:.1f}°")
                except:
                    pass
            elif parts[0].startswith("B"):
                try:
                    angle = float(parts[-1])
                    self.labelB.setText(f"Motor B: {angle:.1f}°")
                except:
                    pass

    def redraw(self):
        # Could update something else regularly if needed
        pass

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = MotorGui()
    gui.show()
    sys.exit(app.exec_())

