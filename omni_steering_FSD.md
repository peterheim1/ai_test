# Functional Specification Document: Omni Steering Controller
## cmd_vel_to_wheels.py

### Purpose

Convert ROS 2 `cmd_vel` (Twist) messages into individual wheel steering angles and drive velocities for a 4-wheel independent steering (swerve) robot.

---

### System Overview

```
                    ┌─────────────────────────┐
   /cmd_vel ───────►│   cmd_vel_to_wheels     │───────► /steering (rad)
 (Twist: vx,vy,ω)   │                         │───────► /drive (m/s)
                    │  - Inverse kinematics   │
                    │  - Angle optimization   │───────► Terminal output
                    └─────────────────────────┘
```

**Input:**
- `/cmd_vel` (geometry_msgs/Twist): Body velocity command
  - `linear.x`: Forward velocity (m/s)
  - `linear.y`: Lateral velocity (m/s, positive = left)
  - `angular.z`: Rotational velocity (rad/s, positive = CCW)

**Outputs:**
- `/steering` (std_msgs/Float64MultiArray): Wheel angles in radians
- `/drive` (std_msgs/Float64MultiArray): Wheel velocities in m/s
- Terminal: Human-readable table

**Wheel Order:** `[front_left, rear_left, rear_right, front_right]`

---

### Robot Geometry

The robot is modeled as a rectangle with wheels at each corner:

```
        +X (forward)
           ▲
           │
    FL ────┼──── FR      FL = front_left  (+x, +y)
     │     │     │       RL = rear_left   (-x, +y)
     │  (0,0)    │       RR = rear_right  (-x, -y)
     │     │     │       FR = front_right (+x, -y)
    RL ────┼──── RR
           │
    +Y ◄───┘ (left)
```

**Configuration Parameters:**
```python
WHEEL_BASE_X = 0.15       # Distance from center to front/rear wheels (m)
WHEEL_BASE_Y = 0.15       # Distance from center to left/right wheels (m)
MAX_WHEEL_VELOCITY = 0.5  # Maximum wheel speed (m/s)

CMD_VEL_TIMEOUT = 2.0     # Zero outputs if no cmd_vel received (seconds)
DEADBAND_LINEAR = 0.01    # Ignore linear velocities below this (m/s)
DEADBAND_ANGULAR = 0.01   # Ignore angular velocities below this (rad/s)
```

---

### Inverse Kinematics

#### Theory

For a rigid body, the velocity at any point is the sum of:
1. The body's linear velocity
2. The tangential velocity from rotation

For wheel `i` at position `(x_i, y_i)`:

```
wheel_vx = vx - ω × y_i
wheel_vy = vy + ω × x_i
```

The wheel's drive velocity and steering angle:

```
velocity = √(wheel_vx² + wheel_vy²)
angle = atan2(wheel_vy, wheel_vx)
```

#### Matrix Form

The inverse kinematics matrix maps body state to wheel velocity vectors:

```
┌ w1_vx ┐   ┌ 1  0  -y1 ┐   ┌ vx ┐
│ w1_vy │   │ 0  1   x1 │   │    │
│ w2_vx │   │ 1  0  -y2 │   │ vy │
│ w2_vy │ = │ 0  1   x2 │ × │    │
│ w3_vx │   │ 1  0  -y3 │   │ ω  │
│ w3_vy │   │ 0  1   x3 │   └────┘
│ w4_vx │   │ 1  0  -y4 │
└ w4_vy ┘   └ 0  1   x4 ┘
```

#### Velocity Scaling

If any wheel exceeds `MAX_WHEEL_VELOCITY`, all wheels are scaled proportionally to maintain the motion profile while respecting motor limits.

---

### Wheel Angle Optimization

#### Problem

When transitioning between commands, a wheel might need to rotate more than 90°. For example, going from forward motion (0°) to rotate-in-place (135°).

#### Solution

If the required steering rotation exceeds 90°:
1. Flip the steering angle by 180°
2. Reverse the drive velocity

This ensures wheels never rotate more than 90° between commands.

#### Algorithm

```python
def optimize_wheel_angle(target_angle, target_velocity, current_angle):
    diff = angle_difference(target_angle, current_angle)

    if abs(diff) > π/2:
        # Flip angle and reverse velocity
        return (target_angle + π), (-target_velocity), True
    else:
        return target_angle, target_velocity, False
```

#### Example: Rotate in Place

Starting from wheels at 0°, commanding pure rotation:

| Wheel | Raw Angle | Rotation | Optimized | Rotation | Velocity |
|-------|-----------|----------|-----------|----------|----------|
| front_left | +135° | 135° | **-45°** | 45° | **reversed** |
| rear_left | +45° | 45° | +45° | 45° | normal |
| rear_right | +135° | 135° | **-45°** | 45° | **reversed** |
| front_right | +45° | 45° | +45° | 45° | normal |

Maximum steering rotation reduced from 135° to 45°.

---

### Instantaneous Center of Rotation (ICR)

The ICR is the point around which the robot rotates. All wheel velocity vectors are perpendicular to lines drawn from each wheel to the ICR.

**Calculation from cmd_vel:**
```
ICR_x = -vy / ω
ICR_y = vx / ω
```

**Special Cases:**
- Pure translation (ω = 0): ICR at infinity, all wheels parallel
- Pure rotation (vx = vy = 0): ICR at robot center (0, 0)
- Arc motion: ICR at finite distance from robot
- **Zero command (vx = vy = ω = 0):** All wheels return to 0° steering, 0 velocity

#### Zero Command Handling

When `cmd_vel` is all zeros, wheels return to forward-facing (0°) with zero velocity:

```python
if abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(omega) < 1e-6:
    return [
        WheelCommand(name=WHEEL_NAMES[i], steering_angle=0.0, drive_velocity=0.0)
        for i in range(self.num_wheels)
    ]
```

This ensures the robot returns to a known state when stopped, rather than holding the previous steering angles.

---

### Safety Features

#### Timeout

If no `cmd_vel` message is received for `CMD_VEL_TIMEOUT` seconds (default: 2.0), the node automatically publishes zero steering and velocity:

```python
def check_timeout(self):
    elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9

    if elapsed > CMD_VEL_TIMEOUT and not self.timed_out:
        self.timed_out = True
        self.process_and_publish(0.0, 0.0, 0.0)
```

This prevents the robot from continuing to move if the control source disconnects or crashes.

#### Deadband

Small input values are treated as zero to reduce jitter from noisy controllers:

```python
def apply_deadband(value: float, deadband: float) -> float:
    return 0.0 if abs(value) < deadband else value

# Applied to incoming cmd_vel:
vx = apply_deadband(msg.linear.x, DEADBAND_LINEAR)    # 0.01 m/s
vy = apply_deadband(msg.linear.y, DEADBAND_LINEAR)    # 0.01 m/s
omega = apply_deadband(msg.angular.z, DEADBAND_ANGULAR)  # 0.01 rad/s
```

Values below the deadband threshold are snapped to zero, preventing small unintended movements.

---

### Code Structure

```
cmd_vel_to_wheels.py
│
├── Configuration
│   ├── WHEEL_BASE_X, WHEEL_BASE_Y - Robot geometry (m)
│   ├── WHEEL_POSITIONS - List of (x, y) tuples for each wheel
│   ├── MAX_WHEEL_VELOCITY - Speed limit (m/s)
│   ├── CMD_VEL_TIMEOUT - Safety timeout (seconds)
│   └── DEADBAND_LINEAR, DEADBAND_ANGULAR - Input filtering
│
├── Data Structures
│   └── WheelCommand (name, steering_angle, drive_velocity, flipped)
│
├── Utility Functions
│   ├── apply_deadband() - Snap small values to zero
│   ├── normalize_angle() - Constrain angle to [-π, π]
│   ├── angle_difference() - Shortest signed angle between two angles
│   └── optimize_wheel_angle() - Flip logic to minimize rotation
│
├── SwerveKinematics Class
│   ├── __init__() - Build inverse kinematics matrix
│   └── compute_wheel_commands() - Convert (vx, vy, ω) to wheel commands
│
└── CmdVelToWheelsNode Class (ROS 2)
    ├── __init__() - Setup subscribers/publishers/timers
    ├── cb_cmd_vel() - Handle incoming Twist, apply deadband
    ├── check_timeout() - Monitor for cmd_vel timeout (safety)
    ├── process_and_publish() - Compute and publish wheel commands
    └── print_table() - Terminal output
```

---

### ROS 2 Interface

#### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Body velocity command |

#### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/steering` | std_msgs/Float64MultiArray | Wheel angles [FL, RL, RR, FR] in radians |
| `/drive` | std_msgs/Float64MultiArray | Wheel velocities [FL, RL, RR, FR] in m/s |

---

### Usage

#### Run the Node
```bash
python3 cmd_vel_to_wheels.py
```

#### Test Commands
```bash
# Forward motion
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}"

# Strafe left
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {y: 0.2}}"

# Rotate in place
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}"

# Arc motion (forward + turn)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}"
```

#### Monitor Outputs
```bash
ros2 topic echo /steering
ros2 topic echo /drive
```

---

### Sample Output

```
──────────────────────────────────────────────────────────────────────────────
cmd_vel: vx=+0.200 vy=+0.000 omega=+0.300
──────────────────────────────────────────────────────────────────────────────
Wheel        Angle (rad)  Angle (deg)  Velocity (m/s)     Flip
──────────────────────────────────────────────────────────────────────────────
front_left       +0.2187       +12.53           +0.2101
rear_left        -0.2187       -12.53           +0.2101
rear_right       -0.2187       -12.53           +0.2101
front_right      +0.2187       +12.53           +0.2101
```

---

### Dependencies

- Python 3
- NumPy
- ROS 2 (rclpy, geometry_msgs, std_msgs)

```bash
# Ubuntu 24.04 with ROS 2 Jazzy
sudo apt install python3-numpy ros-jazzy-rclpy
```

---

### How to Recreate

1. **Define robot geometry** - Measure wheel positions relative to center

2. **Build inverse kinematics matrix** - For each wheel at (x, y):
   ```python
   row_vx = [1.0, 0.0, -y]
   row_vy = [0.0, 1.0,  x]
   ```

3. **Compute wheel vectors** - Multiply matrix by [vx, vy, ω]

4. **Convert to polar** - `velocity = √(vx² + vy²)`, `angle = atan2(vy, vx)`

5. **Add angle optimization** - If rotation >90°, flip angle and reverse velocity

6. **Add safety features:**
   - **Deadband** - Snap small inputs to zero to reduce jitter
   - **Timeout** - Zero outputs if no cmd_vel received for N seconds

7. **Wrap in ROS 2 node** - Subscribe to `/cmd_vel`, publish to `/steering` and `/drive`

---

### References

- [Swerve Drive Kinematics (Chief Delphi)](https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383)
- [Zinger Swerve Controller (Python reference)](https://github.com/pvandervelde/zinger_swerve_controller)
- ROS 2 Jazzy Documentation
