# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This repository contains the control stack for a 4-wheel independent drive / 4-wheel steering (swerve) robot running ROS 2 Jazzy on Ubuntu 24.04. Currently includes Python tools for DDSM115 motor control; full architecture is under development.

## Target Architecture (from FSD.md)

**Hardware:**
- 4 wheel drive motors on single RS485 bus (`/dev/motors`)
- 2 ESP32 steering controllers (SimpleFOC) via serial:
  - `/dev/brushless_left` (left_front, left_rear)
  - `/dev/brushless_right` (right_front, right_rear)
- Teensy 3.5 publishing battery state, dock state, heading angle

**ROS 2 Node Structure (Option B - recommended):**
1. `cmd_vel_to_wheels` - kinematics node (cmd_vel → wheel_cmd), pure math
2. `wheel_hw_driver` - RS485 + ESP32 serial I/O, publishes wheel_state
3. `odom_estimator` - subscribes wheel_state + heading, publishes odometry/TF
4. `system_status` - diagnostics aggregation

**Timing Targets:**
- Control loop latency: 10-50ms
- cmd_vel / wheel_cmd / wheel_state / odom: 20-50 Hz
- RS485 poll loop: 50-100 Hz (sequential polling)

**Message Conventions:**
- Wheel/motor commands: `std_msgs/Float64MultiArray`
- Wheel order: front_left, rear_left, rear_right, front_right
- Battery: `sensor_msgs/BatteryState`

**Reference Implementation:**
- Python swerve controller: https://github.com/pvandervelde/zinger_swerve_controller
- Steering firmware (MKS ESP32 FOC): https://github.com/makerbase-motor/MKS-ESP32FOC

## Hardware Protocol

**DDSM115 Motor Protocol (RS485, 115200 8N1):**
- All frames are 10 bytes with CRC-8/MAXIM checksum (poly 0x31 reflected = 0x8C)
- Protocol 1 (0x64): Speed command - `[ID, 0x64, rpm_hi, rpm_lo, 0, 0, accel, brake, 0, CRC]`
- Protocol 2 (0x74): Query temperature/status
- ID-set frame: `[0xAA, 0x55, 0x53, new_id, 0, 0, 0, 0, 0, CRC]` - must send 5 times, power-cycle after
- RPM range: -330 to +330

## Running the Tools

```bash
# Drive a single motor (edit constants in file for ID/speed)
python3 ddsm115_drive.py

# Scan for motors on the bus
python3 ddsm115_id_tool.py --port /dev/ttyUSB0 --scan

# Scan full ID range (1-253)
python3 ddsm115_id_tool.py --port /dev/ttyUSB0 --scan --full

# Set motor ID (only one motor connected!)
python3 ddsm115_id_tool.py --port /dev/ttyUSB0 --set-id 3 --verify

# ROS2 driver (subscribes to /drive topic)
python3 ros2_drive.py
```

## ROS2 Driver Configuration

`ros2_drive.py` subscribes to `/drive` (Float64MultiArray) with 4 wheel velocities in m/s (range ±0.3). Configuration constants at top of file:
- `MOTOR_IDS`: Maps array indices to motor IDs (default: [2, 1, 3, 4])
- `REVERSE`: Per-motor direction inversion
- `WHEEL_RADIUS_M`: Set correctly for velocity-to-RPM conversion

## udev Rules

The `agents/` directory contains a script and template for creating persistent USB device symlinks. Several `.rules` files in the root are examples for various devices (RPLidar, RealSense, ESP32, etc.).

```bash
# Install the CH340/CH9102 udev rule
./agents/install_udev_rule.sh
```
