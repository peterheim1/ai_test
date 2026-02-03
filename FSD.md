# Functional Specification Document (FSD)
## Mobile Robot Control Stack (ROS 2 Jazzy, Ubuntu 24.04)

### Purpose
Define a reliable, maintainable software architecture for a 4‑wheel independent drive/4‑wheel steering robot using:
- RS485 (single bus) wheel motor controllers
- Two ESP32 steering controllers (SimpleFOC)
- Teensy 3.5 publishing battery state, dock state, and heading angle
- ROS 2 `cmd_vel` input (linear X/Y, angular Z)

The system targets 10–50 ms control‑loop latency and prioritizes reliability, diagnosability, and fault isolation.

---

## System Overview

### Hardware
- 4 independent wheel drive motors on a single RS485 bus (USB dongle to host)
- 4 steering actuators controlled by 2 ESP32 boards (serial)
- Teensy 3.5 publishes:
  - battery state
  - dock state
  - heading angle

### Software Platform
- Ubuntu 24.04
- ROS 2 Jazzy
- MCUs use normal serial (no micro‑ROS)

---

## Design Goals
- Stable and reliable control
- Clear separation of concerns
- Easy restart/recovery of subsystems
- Built‑in diagnostics and health reporting
- Predictable timing on single RS485 bus

---

## Architecture Options

### Option A: Single C++ Monolith
One large program performs:
- `cmd_vel` -> wheel kinematics
- serial I/O for RS485 + ESP32 + Teensy
- odometry calculation and TF
- diagnostics

**Pros**
- Simple deployment (one binary)
- Minimal inter‑process overhead

**Cons**
- Poor fault isolation: a crash stops all control
- Harder to debug and test subsystems
- Risk of blocking serial I/O affecting kinematics and odom
- Diagnostics intertwined with control logic

**Recommended only if** team wants fastest initial prototype and accepts reduced reliability.

---

### Option B: Multiple ROS 2 Nodes (Recommended)
Split into smaller, focused nodes with clear topics:

1) **`cmd_vel_to_wheels not here` (kinematics node)**
   - Sub: `geometry_msgs/Twist` (`cmd_vel`)
   - Pub: `wheel_cmd` (per‑wheel angles + velocities)
   - Pure math, no serial I/O

2) **`wheel_hw_driver` (hardware I/O node)**
   - Serial I/O to RS485 motor controllers and ESP32 steering controllers
   - Sub: `wheel_cmd`
   - Pub: `wheel_state` (measured angles/vels + status)
   - Handles retries, CRC errors, timeouts, rate limiting

3) **`odom_estimator not here`**
   - Sub: `wheel_state` + heading (Teensy)
   - Pub: `nav_msgs/Odometry`, TF
   - Optional fusion with IMU if added later

4) **`system_status` (optional)**
   - Sub: battery/dock/MCU heartbeats
   - Pub: `diagnostics` + aggregated system state
   
5) **`ESP32` (hardware)**  
   - ESP32 steering controllers serial address
   - dev/brushless_left
   -left_front
   -left rear
   - dev/brueshless_right
   - right_rear 
   - right_front

**Pros**
- Fault isolation (restart one node without stopping others)
- Easier testing and debugging
- Clear interfaces and ROS tooling support
- Diagnostics are simpler and more modular

**Cons**
- Slightly more setup (multiple nodes)
- Requires message/interface definitions

**Recommended for reliability and long‑term maintainability.**

---

## Diagnostics (Required)
Each node must publish `diagnostic_msgs/DiagnosticArray` with:

**Bus/Serial Health**
- RS485 timeouts and retries
- CRC error counts
- Average bus loop time and jitter

**MCU Health**
- Last response timestamp per wheel/steering MCU
- Heartbeat freshness for Teensy/ESP32

**Control Health**
- Age of last `cmd_vel`
- Saturation flags (commanded vs achievable limits)
- Command‑to‑measurement error thresholds

**System Health**
- Battery state age/validity
- Dock state age/validity
- Node heartbeat timestamps

---

## Timing and Rates (Suggested)
- `cmd_vel` input: 20–50 Hz
- `wheel_cmd` output: 20–50 Hz
- RS485 poll loop: 50–100 Hz (single bus, sequential wheel polling)
- `wheel_state` publish: 20–50 Hz
- `odom` publish: 20–50 Hz

---

## RS485 Single‑Bus Considerations
- serial address is /dev/motors
- All RS485 traffic centralized in `wheel_hw_driver`
- Sequential polling of 4 motor controllers in fixed loop
- Use:
  - device addressing
  - sequence numbers
  - CRC
  - per‑device timeout tracking
- Avoid multiple processes touching the RS485 bus

---

## ROS 2 Interfaces (Placeholder)
Define message formats for wheel/motor commands and feedback, and for battery/dock state.

**Wheel + motor commands/states**
- Message type: `std_msgs/msg/Float64MultiArray`
- Payload content: agreed ordering for angles/velocities per wheel (document order in node params or a separate interface doc)
- wheel and motor joints order is front_left rear_left rear_right front_right

**Battery and docking**
- Message type: `sensor_msgs/msg/BatteryState` (standard)

---

## Recommendation
Adopt **Option B (multiple ROS 2 nodes)** for reliability, easier diagnostics, and fault isolation.

If desired, nodes can still run in a single process using a multi‑node executor, while preserving separation of concerns.

---

## Initial Controller Implementation (Python)
Use a Python-based test controller until hardware is proven stable, then migrate to C++.

**Reference controller (Python):**
```
https://github.com/pvandervelde/zinger_swerve_controller
```

---

## Open Items / Next Steps
- Confirm final message definitions (`wheel_cmd`, `wheel_state`)
- Define serial protocol framing (addressing, CRC, heartbeat)
- Add integration tests for bus timeouts and reconnection

---

## Hardware References
Steering motors (ESP32 SimpleFOC firmware/project):
```
https://github.com/makerbase-motor/MKS-ESP32FOC
```

Wheel motor RS485/USB interface reference:
```
https://www.waveshare.com/wiki/USB_TO_RS485
```
