// Closed-Loop Stepper Steering Controller
// PID position control for 4 steering wheels using AS5600 encoders + AccelStepper
//
// Hardware: ESP32, 4x stepper (step/dir), 4x AS5600 via TCA9548A I2C mux
// Serial protocol: tab-separated commands at 115200 baud

#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <TCA9548A.h>
#include <AS5600.h>
#include <Preferences.h>

// ============================================================================
// Constants
// ============================================================================

static const int NUM_MOTORS = 4;
static const int STEPS_PER_REV = 12835;
static const float STEPS_PER_DEGREE = STEPS_PER_REV / 360.0f;
static const int MAX_STEERING_DEG = 135;
static const unsigned long PID_INTERVAL_MS = 20;  // 50 Hz

// Pin mapping: [motor index] -> {step, dir, mux channel}
static const uint8_t STEP_PINS[NUM_MOTORS] = {17, 19, 33, 26};
static const uint8_t DIR_PINS[NUM_MOTORS]  = { 5, 18, 32, 25};
static const uint8_t MUX_CHANNELS[NUM_MOTORS] = {0, 2, 3, 1};

// NVS
static const char* NVS_NAMESPACE = "steer";
static const uint32_t NVS_MAGIC = 0x53545231;  // "STR1"
static const uint8_t NVS_VERSION = 1;

// ============================================================================
// Enums & Flags
// ============================================================================

enum SystemState : uint8_t {
    STATE_INIT,
    STATE_RUNNING,
    STATE_ESTOP,
    STATE_WATCHDOG
};

enum FaultFlags : uint8_t {
    FAULT_NONE    = 0,
    FAULT_STALL   = (1 << 0),
    FAULT_ESTOP   = (1 << 1),
    FAULT_I2C_ERR = (1 << 2),
    FAULT_WATCHDOG = (1 << 3)
};

// ============================================================================
// Data Structures
// ============================================================================

struct MotorConfig {
    float kp;
    float ki;
    float kd;
    float calOffset;   // raw AS5600 degrees at steering center
    int8_t stepDir;    // 1 or -1
    int8_t encDir;     // 1 or -1
};

struct MotorState {
    float currentAngle;    // degrees from center (encoder-derived)
    float targetAngle;     // degrees from center (from serial command)
    bool  encoderHealthy;
    // PID state
    float integral;
    float prevError;
    float output;
    // Stall tracking (progress-based)
    unsigned long stallStartMs;
    bool stallTimerActive;
    float stallLastAngle;  // position when stall timer started
};

struct SystemConfig {
    float deadbandDeg;
    float maxStepRate;
    float stallThreshDeg;
    unsigned long stallTimeoutMs;
    unsigned long watchdogTimeoutMs;
    float centerRampRate;  // degrees per second for watchdog ramp
};

struct SerialParser {
    char buf[128];
    uint8_t pos;
    bool overflow;
};

// ============================================================================
// Compile-time Defaults
// ============================================================================

static const MotorConfig DEFAULT_MOTOR_CONFIG = {
    10.0f,   // kp
    0.5f,    // ki
    1.0f,    // kd
    180.0f,  // calOffset (center ~180 deg raw)
    1,       // stepDir
    1        // encDir
};

static const SystemConfig DEFAULT_SYS_CONFIG = {
    0.5f,    // deadbandDeg
    4000.0f, // maxStepRate (steps/sec)
    15.0f,   // stallThreshDeg
    2000,    // stallTimeoutMs
    5000,    // watchdogTimeoutMs
    45.0f    // centerRampRate (deg/s)
};

// ============================================================================
// Global Instances
// ============================================================================

static MotorConfig config[NUM_MOTORS];
static MotorState  motors[NUM_MOTORS];
static SystemConfig sysConfig;

static AccelStepper steppers[NUM_MOTORS] = {
    AccelStepper(AccelStepper::DRIVER, STEP_PINS[0], DIR_PINS[0]),
    AccelStepper(AccelStepper::DRIVER, STEP_PINS[1], DIR_PINS[1]),
    AccelStepper(AccelStepper::DRIVER, STEP_PINS[2], DIR_PINS[2]),
    AccelStepper(AccelStepper::DRIVER, STEP_PINS[3], DIR_PINS[3])
};

static AS5600 encoders[NUM_MOTORS];
static TCA9548A mux(0x70);
static Preferences prefs;
static SerialParser parser = {{0}, 0, false};

static SystemState systemState = STATE_INIT;
static uint8_t faultFlags = FAULT_NONE;
static unsigned long lastCommandMs = 0;
static unsigned long lastPidMs = 0;

// ============================================================================
// Config / NVS
// ============================================================================

static void config_set_defaults() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        config[i] = DEFAULT_MOTOR_CONFIG;
    }
    sysConfig = DEFAULT_SYS_CONFIG;
}

static void config_load() {
    prefs.begin(NVS_NAMESPACE, true);  // read-only

    uint32_t magic = prefs.getUInt("magic", 0);
    uint8_t version = prefs.getUChar("version", 0);

    if (magic != NVS_MAGIC || version != NVS_VERSION) {
        prefs.end();
        Serial.println("NVS: no valid config, using defaults");
        config_set_defaults();
        return;
    }

    char key[16];
    for (int i = 0; i < NUM_MOTORS; i++) {
        snprintf(key, sizeof(key), "m%d_kp", i);
        config[i].kp = prefs.getFloat(key, DEFAULT_MOTOR_CONFIG.kp);
        snprintf(key, sizeof(key), "m%d_ki", i);
        config[i].ki = prefs.getFloat(key, DEFAULT_MOTOR_CONFIG.ki);
        snprintf(key, sizeof(key), "m%d_kd", i);
        config[i].kd = prefs.getFloat(key, DEFAULT_MOTOR_CONFIG.kd);
        snprintf(key, sizeof(key), "m%d_cal", i);
        config[i].calOffset = prefs.getFloat(key, DEFAULT_MOTOR_CONFIG.calOffset);
        snprintf(key, sizeof(key), "m%d_sdir", i);
        config[i].stepDir = prefs.getChar(key, DEFAULT_MOTOR_CONFIG.stepDir);
        snprintf(key, sizeof(key), "m%d_edir", i);
        config[i].encDir = prefs.getChar(key, DEFAULT_MOTOR_CONFIG.encDir);
    }

    sysConfig.deadbandDeg = prefs.getFloat("deadband", DEFAULT_SYS_CONFIG.deadbandDeg);
    sysConfig.maxStepRate = prefs.getFloat("maxrate", DEFAULT_SYS_CONFIG.maxStepRate);
    sysConfig.stallThreshDeg = prefs.getFloat("stalldeg", DEFAULT_SYS_CONFIG.stallThreshDeg);
    sysConfig.stallTimeoutMs = prefs.getULong("stallms", DEFAULT_SYS_CONFIG.stallTimeoutMs);
    sysConfig.watchdogTimeoutMs = prefs.getULong("wdogms", DEFAULT_SYS_CONFIG.watchdogTimeoutMs);
    sysConfig.centerRampRate = prefs.getFloat("ramprate", DEFAULT_SYS_CONFIG.centerRampRate);

    prefs.end();
    Serial.println("NVS: config loaded");
}

static void config_save() {
    prefs.begin(NVS_NAMESPACE, false);  // read-write
    prefs.putUInt("magic", NVS_MAGIC);
    prefs.putUChar("version", NVS_VERSION);

    char key[16];
    for (int i = 0; i < NUM_MOTORS; i++) {
        snprintf(key, sizeof(key), "m%d_kp", i);
        prefs.putFloat(key, config[i].kp);
        snprintf(key, sizeof(key), "m%d_ki", i);
        prefs.putFloat(key, config[i].ki);
        snprintf(key, sizeof(key), "m%d_kd", i);
        prefs.putFloat(key, config[i].kd);
        snprintf(key, sizeof(key), "m%d_cal", i);
        prefs.putFloat(key, config[i].calOffset);
        snprintf(key, sizeof(key), "m%d_sdir", i);
        prefs.putChar(key, config[i].stepDir);
        snprintf(key, sizeof(key), "m%d_edir", i);
        prefs.putChar(key, config[i].encDir);
    }

    prefs.putFloat("deadband", sysConfig.deadbandDeg);
    prefs.putFloat("maxrate", sysConfig.maxStepRate);
    prefs.putFloat("stalldeg", sysConfig.stallThreshDeg);
    prefs.putULong("stallms", sysConfig.stallTimeoutMs);
    prefs.putULong("wdogms", sysConfig.watchdogTimeoutMs);
    prefs.putFloat("ramprate", sysConfig.centerRampRate);

    prefs.end();
}

static void config_save_motor(int idx) {
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putUInt("magic", NVS_MAGIC);
    prefs.putUChar("version", NVS_VERSION);

    char key[16];
    snprintf(key, sizeof(key), "m%d_kp", idx);
    prefs.putFloat(key, config[idx].kp);
    snprintf(key, sizeof(key), "m%d_ki", idx);
    prefs.putFloat(key, config[idx].ki);
    snprintf(key, sizeof(key), "m%d_kd", idx);
    prefs.putFloat(key, config[idx].kd);
    snprintf(key, sizeof(key), "m%d_cal", idx);
    prefs.putFloat(key, config[idx].calOffset);
    snprintf(key, sizeof(key), "m%d_sdir", idx);
    prefs.putChar(key, config[idx].stepDir);
    snprintf(key, sizeof(key), "m%d_edir", idx);
    prefs.putChar(key, config[idx].encDir);

    prefs.end();
}

// ============================================================================
// Encoder
// ============================================================================

static void encoder_init() {
    mux.begin(Wire);
    for (int i = 0; i < NUM_MOTORS; i++) {
        mux.closeAll();
        mux.openChannel(MUX_CHANNELS[i]);
        encoders[i].begin();
        encoders[i].setDirection(AS5600_CLOCK_WISE);

        bool magnet = encoders[i].detectMagnet();
        bool tooWeak = encoders[i].magnetTooWeak();
        bool tooStrong = encoders[i].magnetTooStrong();
        Serial.printf("  Motor %d (mux ch %d): magnet=%s", i, MUX_CHANNELS[i],
                       magnet ? "OK" : "NONE");
        if (tooWeak) Serial.print(" TOO_WEAK");
        if (tooStrong) Serial.print(" TOO_STRONG");
        Serial.println();

        motors[i].encoderHealthy = magnet && !tooWeak && !tooStrong;
    }
    mux.closeAll();
}

static void encoder_read_all() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        mux.closeAll();
        mux.openChannel(MUX_CHANNELS[i]);

        uint16_t raw = encoders[i].rawAngle();
        int err = encoders[i].lastError();

        if (err != AS5600_OK) {
            motors[i].encoderHealthy = false;
            faultFlags |= FAULT_I2C_ERR;
        } else {
            motors[i].encoderHealthy = true;
            float rawDeg = raw * AS5600_RAW_TO_DEGREES;
            motors[i].currentAngle = (rawDeg - config[i].calOffset) * config[i].encDir;
        }
    }
    mux.closeAll();
}

// ============================================================================
// PID
// ============================================================================

static float pid_compute(int idx, float dt) {
    float error = motors[idx].targetAngle - motors[idx].currentAngle;

    // Dead-band: if close enough, hold position
    if (fabsf(error) < sysConfig.deadbandDeg) {
        motors[idx].integral = 0.0f;
        motors[idx].prevError = 0.0f;
        motors[idx].output = 0.0f;
        return 0.0f;
    }

    // Proportional
    float pTerm = config[idx].kp * error;

    // Integral with anti-windup
    motors[idx].integral += error * dt;
    float iTerm = config[idx].ki * motors[idx].integral;
    float iMax = sysConfig.maxStepRate / (config[idx].ki > 0.001f ? config[idx].ki : 1.0f);
    motors[idx].integral = constrain(motors[idx].integral, -iMax, iMax);
    iTerm = config[idx].ki * motors[idx].integral;

    // Derivative
    float derivative = (error - motors[idx].prevError) / dt;
    float dTerm = config[idx].kd * derivative;
    motors[idx].prevError = error;

    // Sum and clamp
    float output = pTerm + iTerm + dTerm;
    output = constrain(output, -sysConfig.maxStepRate, sysConfig.maxStepRate);

    // Apply stepper direction inversion
    output *= config[idx].stepDir;

    motors[idx].output = output;
    return output;
}

static void pid_reset(int idx) {
    motors[idx].integral = 0.0f;
    motors[idx].prevError = 0.0f;
    motors[idx].output = 0.0f;
}

static void pid_reset_all() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        pid_reset(i);
    }
}

// ============================================================================
// Stepper
// ============================================================================

static void stepper_init() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        // Re-enable outputs now that Arduino HAL is initialized
        // (global constructors run before HAL init on ESP32 Core 3.x)
        steppers[i].enableOutputs();
        steppers[i].setMaxSpeed(sysConfig.maxStepRate);
        steppers[i].setSpeed(0);
    }
}

static void stepper_stop_all() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        steppers[i].setSpeed(0);
    }
}

static void stepper_update(int idx, float speedStepsPerSec) {
    steppers[idx].setSpeed(speedStepsPerSec);
}

// ============================================================================
// Safety
// ============================================================================

static void safety_enter_estop() {
    stepper_stop_all();
    pid_reset_all();
    systemState = STATE_ESTOP;
    faultFlags |= FAULT_ESTOP;
    Serial.println("ESTOP");
}

static void safety_check_stall() {
    unsigned long now = millis();
    static const float PROGRESS_THRESH = 1.0f;  // must move at least 1 deg to show progress

    for (int i = 0; i < NUM_MOTORS; i++) {
        float error = fabsf(motors[i].targetAngle - motors[i].currentAngle);
        if (error > sysConfig.stallThreshDeg) {
            if (!motors[i].stallTimerActive) {
                // Start tracking: record current position
                motors[i].stallTimerActive = true;
                motors[i].stallStartMs = now;
                motors[i].stallLastAngle = motors[i].currentAngle;
            } else {
                // Check if motor is making progress
                float moved = fabsf(motors[i].currentAngle - motors[i].stallLastAngle);
                if (moved > PROGRESS_THRESH) {
                    // Motor is moving, reset stall timer
                    motors[i].stallStartMs = now;
                    motors[i].stallLastAngle = motors[i].currentAngle;
                } else if (now - motors[i].stallStartMs >= sysConfig.stallTimeoutMs) {
                    // No progress for stallTimeoutMs — truly stalled
                    faultFlags |= FAULT_STALL;
                    Serial.printf("STALL on motor %d (error=%.1f, no progress)\n", i, error);
                    safety_enter_estop();
                    return;
                }
            }
        } else {
            motors[i].stallTimerActive = false;
        }
    }
}

static void safety_check_watchdog() {
    if (millis() - lastCommandMs > sysConfig.watchdogTimeoutMs) {
        if (systemState == STATE_RUNNING) {
            systemState = STATE_WATCHDOG;
            faultFlags |= FAULT_WATCHDOG;
            Serial.println("WATCHDOG: ramping to center");
        }
    }
}

static void safety_ramp_to_center(float dt) {
    float maxDelta = sysConfig.centerRampRate * dt;  // max degrees this tick
    bool allCentered = true;
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (fabsf(motors[i].targetAngle) > 0.1f) {
            allCentered = false;
            if (motors[i].targetAngle > 0) {
                motors[i].targetAngle -= min(maxDelta, motors[i].targetAngle);
            } else {
                motors[i].targetAngle += min(maxDelta, -motors[i].targetAngle);
            }
        } else {
            motors[i].targetAngle = 0.0f;
        }
    }
    (void)allCentered;
}

// ============================================================================
// Serial Protocol
// ============================================================================

// Forward declarations
static void serial_dispatch(const char* line);
static void cmd_set_targets(const char* args);
static void cmd_calibrate();
static void cmd_pid_single(const char* args);
static void cmd_pid_all(const char* args);
static void cmd_direction(const char* args);
static void cmd_query();
static void cmd_reset();
static void cmd_estop();
static void cmd_sysconfig(const char* args);

static void serial_poll() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (parser.pos > 0) {
                parser.buf[parser.pos] = '\0';
                if (!parser.overflow) {
                    serial_dispatch(parser.buf);
                } else {
                    Serial.println("ERR: line too long");
                }
                parser.pos = 0;
                parser.overflow = false;
            }
        } else {
            if (parser.pos < sizeof(parser.buf) - 1) {
                parser.buf[parser.pos++] = c;
            } else {
                parser.overflow = true;
            }
        }
    }
}

static void serial_dispatch(const char* line) {
    // Any valid command resets watchdog
    lastCommandMs = millis();

    char cmd = line[0];
    const char* args = (line[1] == '\t') ? &line[2] : &line[1];

    switch (cmd) {
        case 's': cmd_set_targets(args); break;
        case 'c': cmd_calibrate(); break;
        case 'p': cmd_pid_single(args); break;
        case 'P': cmd_pid_all(args); break;
        case 'd': cmd_direction(args); break;
        case 'q': cmd_query(); break;
        case 'r': cmd_reset(); break;
        case 'e': cmd_estop(); break;
        case 'S': cmd_sysconfig(args); break;
        default:
            Serial.printf("ERR: unknown cmd '%c'\n", cmd);
            break;
    }
}

static void cmd_set_targets(const char* args) {
    float t[NUM_MOTORS];
    int n = sscanf(args, "%f\t%f\t%f\t%f", &t[0], &t[1], &t[2], &t[3]);
    if (n != NUM_MOTORS) {
        Serial.printf("ERR: s expects 4 floats, got %d\n", n);
        return;
    }
    for (int i = 0; i < NUM_MOTORS; i++) {
        t[i] = constrain(t[i], (float)-MAX_STEERING_DEG, (float)MAX_STEERING_DEG);
        motors[i].targetAngle = t[i];
    }
    if (systemState == STATE_WATCHDOG) {
        systemState = STATE_RUNNING;
        faultFlags &= ~FAULT_WATCHDOG;
        Serial.println("WATCHDOG cleared");
    }
}

static void cmd_calibrate() {
    encoder_read_all();
    for (int i = 0; i < NUM_MOTORS; i++) {
        mux.closeAll();
        mux.openChannel(MUX_CHANNELS[i]);
        uint16_t raw = encoders[i].rawAngle();
        config[i].calOffset = raw * AS5600_RAW_TO_DEGREES;
        Serial.printf("  Motor %d: calOffset = %.2f\n", i, config[i].calOffset);
    }
    mux.closeAll();
    config_save();
    // Re-read with new offsets
    encoder_read_all();
    // Set targets to current positions to prevent jump
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].targetAngle = motors[i].currentAngle;
    }
    pid_reset_all();
    Serial.println("Calibration saved");
}

static void cmd_pid_single(const char* args) {
    int m;
    float kp, ki, kd;
    int n = sscanf(args, "%d\t%f\t%f\t%f", &m, &kp, &ki, &kd);
    if (n != 4 || m < 0 || m >= NUM_MOTORS) {
        Serial.println("ERR: p expects motor(0-3) Kp Ki Kd");
        return;
    }
    config[m].kp = kp;
    config[m].ki = ki;
    config[m].kd = kd;
    pid_reset(m);
    config_save_motor(m);
    Serial.printf("PID motor %d: Kp=%.3f Ki=%.3f Kd=%.3f (saved)\n", m, kp, ki, kd);
}

static void cmd_pid_all(const char* args) {
    float kp, ki, kd;
    int n = sscanf(args, "%f\t%f\t%f", &kp, &ki, &kd);
    if (n != 3) {
        Serial.println("ERR: P expects Kp Ki Kd");
        return;
    }
    for (int i = 0; i < NUM_MOTORS; i++) {
        config[i].kp = kp;
        config[i].ki = ki;
        config[i].kd = kd;
        pid_reset(i);
    }
    config_save();
    Serial.printf("PID all: Kp=%.3f Ki=%.3f Kd=%.3f (saved)\n", kp, ki, kd);
}

static void cmd_direction(const char* args) {
    int m, sd, ed;
    int n = sscanf(args, "%d\t%d\t%d", &m, &sd, &ed);
    if (n != 3 || m < 0 || m >= NUM_MOTORS) {
        Serial.println("ERR: d expects motor(0-3) stepDir encDir");
        return;
    }
    config[m].stepDir = (sd >= 0) ? 1 : -1;
    config[m].encDir = (ed >= 0) ? 1 : -1;
    config_save_motor(m);
    Serial.printf("Dir motor %d: stepDir=%d encDir=%d (saved)\n",
                   m, config[m].stepDir, config[m].encDir);
}

static void cmd_query() {
    Serial.println("== Config ==");
    for (int i = 0; i < NUM_MOTORS; i++) {
        Serial.printf("  M%d: Kp=%.3f Ki=%.3f Kd=%.3f cal=%.2f sdir=%d edir=%d\n",
                       i, config[i].kp, config[i].ki, config[i].kd,
                       config[i].calOffset, config[i].stepDir, config[i].encDir);
    }
    Serial.printf("  deadband=%.2f maxRate=%.0f stallDeg=%.1f stallMs=%lu wdogMs=%lu rampRate=%.1f\n",
                   sysConfig.deadbandDeg, sysConfig.maxStepRate,
                   sysConfig.stallThreshDeg, sysConfig.stallTimeoutMs,
                   sysConfig.watchdogTimeoutMs, sysConfig.centerRampRate);
}

static void cmd_reset() {
    if (systemState == STATE_ESTOP) {
        // Set targets to current positions to prevent jump
        encoder_read_all();
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].targetAngle = motors[i].currentAngle;
        }
        pid_reset_all();
        faultFlags = FAULT_NONE;
        systemState = STATE_RUNNING;
        lastCommandMs = millis();
        Serial.println("Reset: RUNNING");
    } else {
        Serial.println("Not in ESTOP, ignoring reset");
    }
}

static void cmd_estop() {
    safety_enter_estop();
}

static void cmd_sysconfig(const char* args) {
    float maxRate;
    unsigned long stallMs, wdogMs;
    int n = sscanf(args, "%f %lu %lu", &maxRate, &stallMs, &wdogMs);
    if (n != 3) {
        Serial.println("ERR: S expects maxRate stallMs wdogMs");
        return;
    }
    if (stallMs < 200) stallMs = 200;
    if (wdogMs < 500) wdogMs = 500;
    if (maxRate < 100.0f) maxRate = 100.0f;
    if (maxRate > 10000.0f) maxRate = 10000.0f;

    sysConfig.maxStepRate = maxRate;
    sysConfig.stallTimeoutMs = stallMs;
    sysConfig.watchdogTimeoutMs = wdogMs;

    // Apply new max rate to AccelStepper instances
    for (int i = 0; i < NUM_MOTORS; i++) {
        steppers[i].setMaxSpeed(sysConfig.maxStepRate);
    }

    config_save();
    Serial.printf("SysConfig: maxRate=%.0f stallMs=%lu wdogMs=%lu (saved)\n",
                   sysConfig.maxStepRate, sysConfig.stallTimeoutMs, sysConfig.watchdogTimeoutMs);
}

// ============================================================================
// Telemetry
// ============================================================================

static void telemetry_send() {
    Serial.print("a");
    for (int i = 0; i < NUM_MOTORS; i++) {
        Serial.print('\t');
        Serial.print(motors[i].currentAngle, 2);
    }
    for (int i = 0; i < NUM_MOTORS; i++) {
        Serial.print('\t');
        Serial.print(motors[i].targetAngle, 2);
    }
    Serial.print('\t');

    // Status string
    bool first = true;
    if (faultFlags == FAULT_NONE) {
        Serial.print("OK");
    } else {
        if (faultFlags & FAULT_STALL) {
            if (!first) Serial.print('|');
            Serial.print("STALL");
            first = false;
        }
        if (faultFlags & FAULT_ESTOP) {
            if (!first) Serial.print('|');
            Serial.print("ESTOP");
            first = false;
        }
        if (faultFlags & FAULT_I2C_ERR) {
            if (!first) Serial.print('|');
            Serial.print("I2C_ERR");
            first = false;
        }
        if (faultFlags & FAULT_WATCHDOG) {
            if (!first) Serial.print('|');
            Serial.print("WDOG");
            first = false;
        }
    }
    Serial.print("\r\n");
}

// ============================================================================
// Setup & Loop
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("== Stepper Steering Controller ==");

    Wire.begin();
    Wire.setClock(400000);  // 400 kHz I2C

    config_set_defaults();
    config_load();

    Serial.println("Initializing encoders...");
    encoder_init();

    Serial.println("Initializing steppers...");
    stepper_init();

    // Read initial positions
    encoder_read_all();
    // Clear any I2C error from init reads
    faultFlags &= ~FAULT_I2C_ERR;

    // Set initial targets to current positions (no movement until first command)
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].targetAngle = motors[i].currentAngle;
        motors[i].stallTimerActive = false;
    }
    pid_reset_all();

    lastCommandMs = millis();
    lastPidMs = millis();
    systemState = STATE_RUNNING;

    Serial.println("Ready");
}

void loop() {
    // 1. Non-blocking serial input
    serial_poll();

    // 2. Call runSpeed() for all steppers every iteration
    for (int i = 0; i < NUM_MOTORS; i++) {
        steppers[i].runSpeed();
    }

    // 3. PID cycle at 50 Hz
    unsigned long now = millis();
    if (now - lastPidMs >= PID_INTERVAL_MS) {
        float dt = (now - lastPidMs) / 1000.0f;
        lastPidMs = now;

        // Clear transient I2C error flag each cycle
        faultFlags &= ~FAULT_I2C_ERR;

        // Read all encoders
        encoder_read_all();

        switch (systemState) {
            case STATE_RUNNING:
                safety_check_watchdog();
                for (int i = 0; i < NUM_MOTORS; i++) {
                    float spd = pid_compute(i, dt);
                    stepper_update(i, spd);
                }
                safety_check_stall();
                break;

            case STATE_WATCHDOG:
                safety_ramp_to_center(dt);
                for (int i = 0; i < NUM_MOTORS; i++) {
                    float spd = pid_compute(i, dt);
                    stepper_update(i, spd);
                }
                safety_check_stall();
                break;

            case STATE_ESTOP:
                // Motors already stopped, do nothing
                break;

            case STATE_INIT:
                // Should not reach here after setup
                break;
        }

        telemetry_send();
    }
}
