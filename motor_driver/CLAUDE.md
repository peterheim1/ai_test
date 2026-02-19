# CLAUDE.md — Stepper Steering Controller Firmware

## Project Overview

Closed-loop PID position controller for 4 steering wheels on a swerve-drive robot. ESP32 reads absolute angle from AS5600 magnetic encoders via TCA9548A I2C mux, computes PID error, and drives steppers at corrected speed using AccelStepper.

**Platform:** ESP32 dev board, PlatformIO, Arduino Core 3.x (pioarduino), framework: Arduino

## Current State

- Firmware is a single file: `src/main.cpp`
- Full rewrite from old MobaTools/Messenger-based open-loop code to closed-loop PID
- Builds clean with zero warnings on Arduino Core 3.3.6 (IDF 5.5.0)
- Known fix applied: `enableOutputs()` called in `stepper_init()` to re-init GPIO after HAL startup (global AccelStepper constructors run before ESP32 HAL is ready on Core 3.x)
- Stall detection is progress-based (checks if motor position is actually changing, not just absolute error)

## Build & Flash

```bash
# Build
pio run

# Flash
pio run -t upload

# Serial monitor
pio device monitor

# Erase NVS (to reset saved config to compile-time defaults)
pio run -t erase
# then reflash
```

## Hardware Pin Mapping

| Motor | Step Pin | Dir Pin | TCA9548A Ch | Old AS5600 name |
|-------|----------|---------|-------------|-----------------|
| 0     | 17       | 5       | 0           | stpB            |
| 1     | 19       | 18      | 2           | stpA            |
| 2     | 33       | 32      | 3           | stpD            |
| 3     | 26       | 25      | 1           | stpC            |

- TCA9548A I2C mux at address 0x70
- I2C: 400 kHz
- Steps per revolution: 12835 (includes gearing + microstepping)
- Steering range: +/- 135 degrees

## Serial Protocol (115200 baud)

All commands are newline-terminated. Fields separated by spaces or tabs.

| Cmd | Format | Description |
|-----|--------|-------------|
| `s` | `s S0 S1 S2 S3` | Set 4 target angles (degrees, +/-135 max) |
| `S` | `S maxRate stallMs wdogMs` | Set system config (saved to NVS) |
| `c` | `c` | Calibrate: store current encoder positions as zero offsets |
| `p` | `p M Kp Ki Kd` | Set PID gains for motor M (0-3), save NVS |
| `P` | `P Kp Ki Kd` | Set PID gains for all motors, save NVS |
| `d` | `d M stepDir encDir` | Set direction flags for motor M (1 or -1), save NVS |
| `q` | `q` | Query all config |
| `r` | `r` | Reset from ESTOP -> RUNNING (targets = current positions) |
| `e` | `e` | Emergency stop: stop all motors immediately |

### Telemetry Output (50 Hz)

```
a\tpos0\tpos1\tpos2\tpos3\ttgt0\ttgt1\ttgt2\ttgt3\tstatus\r\n
```

Status values: `OK`, `STALL`, `ESTOP`, `I2C_ERR`, `WDOG` (pipe-separated if multiple)

## Architecture

### State Machine

`STATE_INIT` -> `STATE_RUNNING` -> `STATE_ESTOP` / `STATE_WATCHDOG`

- **RUNNING**: PID active, stall/watchdog checks running
- **WATCHDOG**: no command for wdogMs, ramps targets to 0 at 45 deg/s. New `s` command -> back to RUNNING
- **ESTOP**: all motors stopped. Requires `r` to reset
- Boot: targets = current encoder positions (no movement until first `s` command)

### Code Organization (all in main.cpp)

1. Constants, enums, structs (MotorConfig, MotorState, SystemConfig)
2. Config/NVS: `config_set_defaults()`, `config_load()`, `config_save()`, `config_save_motor()`
3. Encoder: `encoder_init()`, `encoder_read_all()`
4. PID: `pid_compute()`, `pid_reset()`, `pid_reset_all()`
5. Stepper: `stepper_init()`, `stepper_stop_all()`, `stepper_update()`
6. Safety: `safety_enter_estop()`, `safety_check_stall()`, `safety_check_watchdog()`, `safety_ramp_to_center()`
7. Serial: `serial_poll()`, `serial_dispatch()`, `cmd_*` handlers
8. Telemetry: `telemetry_send()`
9. `setup()` and `loop()`

### Main Loop

```
loop():
  1. serial_poll()                      — non-blocking char-by-char parser
  2. steppers[i].runSpeed() x4          — MUST call every iteration (AccelStepper timing)
  3. if 20ms elapsed (50 Hz):
     - encoder_read_all()
     - state machine: PID + safety checks
     - telemetry_send()
```

### AccelStepper Usage

Uses **continuous speed mode** only (non-blocking):
- `setSpeed(float)` — PID sets desired step rate (signed = direction)
- `runSpeed()` — called every loop, generates one pulse if timing is right (~5us per call)
- Does NOT use `moveTo()`, `run()`, or `runToPosition()` (those block)

## Default Configuration

| Parameter | Default | Unit |
|-----------|---------|------|
| PID Kp | 10.0 | steps/s per degree |
| PID Ki | 0.5 | steps/s per degree-second |
| PID Kd | 1.0 | steps/s per degree/second |
| Dead-band | 0.5 | degrees |
| Max step rate | 4000 | steps/sec (~112 deg/s) |
| Stall threshold | 15.0 | degrees |
| Stall timeout | 2000 | ms |
| Watchdog timeout | 5000 | ms |
| Center ramp rate | 45.0 | deg/s |
| Stepper direction | 1 | per motor |
| Encoder direction | 1 | per motor |
| Cal offset | 180.0 | degrees raw |

## NVS Storage

Namespace: `"steer"`. Magic: `0x53545231`, version: `1`. If magic/version mismatch on boot, compile-time defaults are used. All `p`, `P`, `d`, `c`, `S` commands auto-save to NVS.

## Libraries

- **AccelStepper** (via lib_deps, waspinator/AccelStepper@^1.64)
- **AS5600** (in lib/)
- **TCA9548A** (in lib/)
- **Wire, Preferences** (built-in)

Removed: MobaTools, Messenger

## Requirements

Full requirements spec: `REQUIREMENTS.md`
