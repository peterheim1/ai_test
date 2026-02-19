# Closed-Loop Stepper Steering Controller — Requirements Specification

## 1. Overview

Replace the existing `main.cpp` open-loop stepper steering firmware with a clean,
closed-loop PID position controller for 4 steering wheels. The ESP32 reads absolute
angle from AS5600 magnetic encoders, computes PID error, and drives steppers at a
corrected speed to reach the commanded angle.

**Platform:** ESP32 dev board, PlatformIO, Arduino framework

## 2. Scope

### In Scope
- 4-channel closed-loop stepper position control (step/dir)
- 4x AS5600 encoder reading via TCA9548A I2C multiplexer
- Serial command interface (tab-separated, compatible with existing ROS 2 host)
- Serial telemetry (position feedback, status, faults)
- PID tuning via serial commands with EEPROM persistence
- Calibration offsets stored in EEPROM
- Communication watchdog with return-to-center timeout
- Stall detection with all-stop e-stop behavior

### Out of Scope (removed from old firmware)
- Drive motor I2C forwarding (SetTarget to addresses 4-7)
- Battery voltage ADC reading
- Dock pin sensing
- I2C slave RequestData

## 3. Hardware Interface

### 3.1 Stepper Pins (step/dir)
| Motor | Step Pin | Dir Pin |
|-------|----------|---------|
| S0    | 17       | 5       |
| S1    | 19       | 18      |
| S2    | 33       | 32      |
| S3    | 26       | 25      |

### 3.2 AS5600 Encoders (I2C via TCA9548A at 0x70)
| Motor | AS5600 Object | TCA9548A Channel |
|-------|---------------|------------------|
| S0    | stpB          | 0                |
| S1    | stpA          | 2                |
| S2    | stpD          | 3                |
| S3    | stpC          | 1                |

### 3.3 Constants
- Steps per revolution: 12835
- Steering range: +/- 135 degrees from center
- Calibration strategy: AS5600 offset ensures 0-deg steering maps to ~180-deg raw
  angle, so +/-135 deg never crosses the 360/0 wrap point

## 4. Functional Requirements

### FR-1: PID Position Control
- Each motor runs an independent PID loop at **50 Hz** (20 ms period)
- **Independent PID gains per motor** (Kp, Ki, Kd stored separately for each of 4 motors)
- **Per-motor direction inversion flags** for both stepper and encoder (configurable, stored in EEPROM)
- **Input:** target angle (degrees, from serial) vs. current angle (degrees, from AS5600)
- **Output:** step rate (steps/second) and direction
- Step pulse generation via ESP32 hardware timers (not MobaTools)
- PID output clamped to a configurable maximum step rate
- When error is below a configurable dead-band, output zero (hold position, reduce noise)
- Steps per revolution (12835) already accounts for gearing and microstepping (hardwired on drivers)

### FR-2: Encoder Reading
- Read all 4 AS5600 sensors each PID cycle via TCA9548A multiplexer
- Convert raw 12-bit angle to degrees using AS5600_RAW_TO_DEGREES
- Subtract stored calibration offset to get steering-relative angle
- Validate readings: if I2C read fails, flag that channel as unhealthy

### FR-3: Serial Command Protocol (host → ESP32)
Maintain backward compatibility with existing tab-separated format:

| Command | Format | Description |
|---------|--------|-------------|
| `s`     | `s<TAB>S0<TAB>S1<TAB>S2<TAB>S3\n` | Set target angles (degrees) for 4 motors |
| `c`     | `c\n` | Calibrate: read current AS5600 positions, store as zero offsets in EEPROM |
| `p`     | `p<TAB>motor<TAB>Kp<TAB>Ki<TAB>Kd\n` | Set PID gains for a specific motor (0-3), save to EEPROM |
| `P`     | `P<TAB>Kp<TAB>Ki<TAB>Kd\n` | Set PID gains for ALL motors at once, save to EEPROM |
| `d`     | `d<TAB>motor<TAB>stepDir<TAB>encDir\n` | Set direction inversion for motor (0-3): stepDir/encDir = 1 or -1 |
| `q`     | `q\n` | Query current PID gains, calibration offsets, and direction flags for all motors |
| `r`     | `r\n` | Reset fault state, re-enable motors after e-stop |
| `e`     | `e\n` | Emergency stop: immediately disable all steppers |

### FR-4: Serial Telemetry (ESP32 → host)
Publish at 50 Hz (same rate as PID loop):

```
a<TAB>pos0<TAB>pos1<TAB>pos2<TAB>pos3<TAB>tgt0<TAB>tgt1<TAB>tgt2<TAB>tgt3<TAB>status\n
```

- `pos0-3`: current angle in degrees (from AS5600)
- `tgt0-3`: current target angle in degrees
- `status`: bitmask or string indicating fault state (OK, STALL, ESTOP, I2C_ERR)

### FR-5: Stall Detection & E-Stop
- If any motor's position error exceeds a threshold (configurable, e.g. 15 degrees)
  for longer than a configurable duration (e.g. 500 ms), declare a **stall fault**
- On stall fault: **stop ALL 4 steppers** immediately, report fault over serial
- Motors remain disabled until host sends reset command (`r`)
- **Single global enable pin** (pin TBD) controls all stepper drivers — future
  implementation placeholder. For now, e-stop = stop generating pulses. Pin definition
  and enable/disable logic will be added in a later revision.

### FR-6: Communication Watchdog
- If no valid serial command is received for a configurable timeout (default: 2 seconds),
  command all wheels to return to center (0 degrees)
- Use a controlled ramp (not instant snap) to return to center
- Report watchdog timeout in telemetry status field

### FR-7: EEPROM Persistence
Store in ESP32 Preferences (NVS):
- Calibration offsets for each motor (4x float)
- PID gains per motor: Kp, Ki, Kd (4x3 = 12 floats)
- Direction inversion flags per motor: stepper dir, encoder dir (4x2 = 8 int8)
- Stall threshold (degrees) and stall timeout (ms)
- Magic byte / version for validity checking

Load on boot. If no valid data found, use compile-time defaults.

### FR-8: Auto-Zero on Boot
- On startup, read AS5600 values
- Apply stored EEPROM calibration offsets
- Set initial target position to 0 degrees (center)
- Do NOT move steppers until first command received (hold at wherever wheels are)

## 5. Non-Functional Requirements

### NFR-1: Code Structure
- Use arrays/structs for motor data — NO copy-paste for 4 motors
- Separate concerns into logical sections/files:
  - `main.cpp` — setup/loop, top-level orchestration
  - Motor/PID module (struct-based, operates on motor index)
  - Encoder module (TCA9548A + AS5600 reading)
  - Serial command parser
  - Config/EEPROM module
- No blocking delays in the main loop
- Non-blocking serial parsing (replace Messenger library)

### NFR-2: Safety
- All PID outputs clamped to safe ranges
- Stall detection cannot be disabled
- Watchdog timeout cannot be set below 500 ms
- E-stop command always processed immediately (highest priority)

### NFR-3: Performance
- PID loop must complete within 20 ms for all 4 motors
- I2C reads for 4 encoders (through mux) estimated at ~4 ms total
- Remaining 16 ms budget for PID computation and serial I/O

### NFR-4: Debuggability
- Serial output parseable by existing ROS 2 host code
- PID gains readable via `q` command
- Status field in telemetry clearly indicates fault conditions

## 6. Libraries

### Keep
- `AS5600` — encoder reading
- `TCA9548A` — I2C multiplexer
- `Wire` — I2C (built-in)
- `Preferences` — ESP32 NVS/EEPROM (built-in)

### Remove
- `MobaTools` — replaced by hardware timer step generation
- `Messenger` — replaced by simple non-blocking serial parser

## 7. Default Configuration Values

| Parameter | Default | Unit | Per-motor? |
|-----------|---------|------|------------|
| PID Kp | 10.0 | steps/s per degree | Yes |
| PID Ki | 0.5 | steps/s per degree-second | Yes |
| PID Kd | 1.0 | steps/s per degree/second | Yes |
| Stepper direction | 1 | 1 or -1 | Yes |
| Encoder direction | 1 | 1 or -1 | Yes |
| PID dead-band | 0.5 | degrees | No (global) |
| Max step rate | 2000 | steps/second | No (global) |
| Stall error threshold | 15.0 | degrees | No (global) |
| Stall timeout | 500 | ms | No (global) |
| Watchdog timeout | 2000 | ms | No (global) |
| PID loop rate | 50 | Hz | No (global) |
| Steps per revolution | 12835 | steps (includes gearing + microsteps) | No (global) |
| Max steering angle | 135 | degrees | No (global) |
| Enable pin | TBD | GPIO (future) | No (global) |

*Note: PID defaults are starting points — will need tuning on real hardware per motor.*

## 8. Resolved Questions

1. **Per-motor direction inversion:** YES — both stepper and encoder directions are
   independently configurable per motor, stored in EEPROM, settable via `d` command.
2. **Per-motor PID gains:** YES — independent Kp/Ki/Kd per motor, stored in EEPROM.
3. **Enable pin:** Single global enable pin for all stepper drivers. Pin TBD,
   implementation deferred to a later revision. E-stop currently = stop pulses.
4. **Gear ratio:** 12835 steps/rev already includes gearing and microstepping
   (hardwired on the stepper drivers). No additional ratio needed in firmware.

## 9. Next Steps

After requirements approval:
1. `/sc:design` — Detailed software architecture (structs, state machine, timer config)
2. `/sc:implement` — Write the new firmware
3. Test with serial monitor before connecting to ROS 2 host
