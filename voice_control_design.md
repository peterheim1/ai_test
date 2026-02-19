# Robbie Voice Control - Architecture Design

## 1. Architecture Overview

The voice control system adds a single Python process on the GPU PC that bridges the ESPHome voice satellite (ReSpeaker Lite) to the existing ROS 2 control stack. It does **not** modify any existing nodes.

```
                            ┌─────────────────────────────────────────────────────────┐
                            │  GPU PC                                                  │
                            │                                                          │
┌──────────────┐  ESPHome   │  ┌──────────────────────────────────────────────────┐    │
│ ReSpeaker    │  Native    │  │  robbie_voice_server.py                          │    │
│ Lite         │  API       │  │                                                  │    │
│              │◄──────────►│  │  ┌─────────┐  ┌─────────┐  ┌──────────────┐     │    │
│ micro_wake_  │  (TCP)     │  │  │ ESPHome │  │ Audio   │  │ Fast-Path    │     │    │
│ word         │            │  │  │ Client  │──│ Buffer  │──│ Stop Detect  │──┐  │    │
│ "ok nabu"    │            │  │  │         │  │         │  │              │  │  │    │
│              │            │  │  └─────────┘  └────┬────┘  └──────────────┘  │  │    │
│ LED + Spkr   │            │  │                    │                         │  │    │
└──────────────┘            │  │               ┌────▼────┐                    │  │    │
                            │  │               │ Faster- │                    │  │    │
                            │  │               │ Whisper │                    │  │    │
                            │  │               │ (CUDA)  │                    │  │    │
                            │  │               └────┬────┘                    │  │    │
                            │  │                    │                         │  │    │
                            │  │               ┌────▼────┐                    │  │    │
                            │  │               │ Intent  │                    │  │    │
                            │  │               │ Classif │                    │  │    │
                            │  │               └────┬────┘                    │  │    │
                            │  │                    │                         │  │    │
                            │  │          ┌─────────┼─────────┐              │  │    │
                            │  │          │         │         │              │  │    │
                            │  │     ┌────▼──┐ ┌───▼───┐ ┌───▼────┐        │  │    │
                            │  │     │ ROS 2 │ │Ollama │ │ Error  │        │  │    │
                            │  │     │Publish│ │Mistral│ │Handler │        │  │    │
                            │  │     └───┬───┘ └───┬───┘ └───┬────┘        │  │    │
                            │  │         │         │         │              │  │    │
                            │  │         │    ┌────▼─────────▼──┐          │  │    │
                            │  │         │    │  Piper TTS      │          │  │    │
                            │  │         │    │  (male voice)   │          │  │    │
                            │  │         │    └────────┬────────┘          │  │    │
                            │  │         │             │                    │  │    │
                            │  │         │    ┌────────▼────────┐          │  │    │
                            │  │         │    │ TTS → ESPHome   │          │  │    │
                            │  │         │    │ speaker stream  │          │  │    │
                            │  │         │    └─────────────────┘          │  │    │
                            │  │         │                                 │  │    │
                            │  │    ┌────▼──────────────────┐             │  │    │
                            │  │    │ Console Log (stdout)  │◄────────────┘  │    │
                            │  │    └───────────────────────┘                │    │
                            │  └──────────────────────────────────────────────┘    │
                            │         │                                            │
                            └─────────┼────────────────────────────────────────────┘
                                      │ ROS 2 topics
                                      ▼
                            ┌───────────────────┐
                            │ Existing ROS 2    │
                            │ cmd_vel_to_wheels  │
                            │ ros2_drive         │
                            │ ros2_steering      │
                            └───────────────────┘
```

---

## 2. ROS 2 Interface Design

### 2.1 New Topics Published by Voice Server

| Topic | Type | Hz | Description |
|---|---|---|---|
| `/voice/intent` | `std_msgs/String` | on event | JSON-encoded intent (see §2.2) |
| `/voice/stop` | `std_msgs/Empty` | on event | Fast-path emergency stop (no payload needed) |
| `/voice/tts_text` | `std_msgs/String` | on event | Text being spoken (for logging/display nodes) |
| `/cmd_vel` | `geometry_msgs/Twist` | on event | For `goto` navigation commands (if nav stack present) |
| `/head/position` | `std_msgs/Float64MultiArray` | on event | `[pan, tilt]` for `look` commands |

### 2.2 Intent Message Format (JSON on `/voice/intent`)

All intents are published as JSON strings on `/voice/intent`. This allows any downstream node to subscribe and react.

```json
{
  "timestamp": "2026-02-16T14:32:05.123",
  "utterance": "go to the kitchen",
  "intent": "goto",
  "confidence": 0.95,
  "params": {
    "location": "kitchen",
    "x": 2.16,
    "y": 4.53,
    "yaw_deg": 137.33
  }
}
```

Intent-specific params:

```yaml
# look
params: { direction: "left", pan: 1.2, tilt: 0.0 }

# goto
params: { location: "kitchen", x: 2.16, y: 4.53, yaw_deg: 137.33 }

# point_to
params: { target: "the table" }

# stop_all
params: {}

# query_status
params: { subject: "battery" }

# dock / undock / cal
params: {}

# list_topics
params: {}

# list_node_params
params: { node_name: "move_group" }

# general_question
params: { original: "what is the capital of France" }

# unknown
params: { original: "blinkify the wotsit" }
```

### 2.3 Topics Subscribed by Voice Server

| Topic | Type | Purpose |
|---|---|---|
| `/battery_state` | `sensor_msgs/BatteryState` | Answer `query_status` battery questions |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Answer `query_status` error questions |

### 2.4 Integration with Existing Nodes

The voice server does **not** replace or modify existing nodes. It integrates by publishing to existing topics:

```
Voice "go to kitchen" → publishes nav goal (future nav stack)
Voice "look left"     → publishes /head/position [1.2, 0.0]
Voice "stop"          → publishes /voice/stop (fast-path)
                        + publishes /cmd_vel with all zeros
                        + publishes /drive with all zeros
```

For `stop_all`, the voice server publishes to **multiple topics simultaneously** to ensure all motion ceases:
- `/voice/stop` (for any node monitoring emergency stop)
- `/cmd_vel` with `Twist()` (all zeros) to stop kinematics
- `/drive` with `[0, 0, 0, 0]` to directly stop motors as backup

---

## 3. ESPHome Configuration Design

### 3.1 ESPHome YAML for ReSpeaker Lite

```yaml
# robbie_voice_satellite.yaml
# Flash to ReSpeaker Lite (XIAO ESP32-S3) via ESPHome

esphome:
  name: robbie-voice
  friendly_name: "Robbie Voice Satellite"

esp32:
  board: seeed_xiao_esp32s3
  framework:
    type: esp-idf

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  manual_ip:
    static_ip: 192.168.1.50       # ← set to your robot's static IP
    gateway: 192.168.1.1
    subnet: 255.255.255.0

api:
  encryption:
    key: !secret api_encryption_key
  # aioesphomeapi connects here (port 6053 default)

logger:
  level: INFO

i2s_audio:
  - id: i2s_input
    i2s_lrclk_pin: GPIO7       # ReSpeaker Lite mic pins
    i2s_bclk_pin: GPIO8
  - id: i2s_output
    i2s_lrclk_pin: GPIO44      # ReSpeaker Lite speaker pins
    i2s_bclk_pin: GPIO43

microphone:
  - platform: i2s_audio
    id: mic
    i2s_audio_id: i2s_input
    adc_type: external
    i2s_din_pin: GPIO9
    pdm: false
    bits_per_sample: 16bit
    channel: left

speaker:
  - platform: i2s_audio
    id: spkr
    i2s_audio_id: i2s_output
    dac_type: external
    i2s_dout_pin: GPIO42
    mode: mono

micro_wake_word:
  model: okay_nabu
  on_wake_word_detected:
    - light.turn_on:
        id: status_led
        effect: "listening_pulse"
    - voice_assistant.start:

voice_assistant:
  microphone: mic
  speaker: spkr
  use_wake_word: true
  on_tts_end:
    - light.turn_off:
        id: status_led

light:
  - platform: esp32_rmt_led_strip
    id: status_led
    pin: GPIO1                   # ReSpeaker Lite onboard LED
    num_leds: 1
    rgb_order: GRB
    chipset: WS2812
    effects:
      - pulse:
          name: "listening_pulse"
          transition_length: 300ms
          update_interval: 300ms
          min_brightness: 20%
          max_brightness: 100%
```

**Notes:**
- Pin assignments are for the ReSpeaker Lite (XIAO ESP32-S3) — verify against your specific board revision
- The `api:` section enables the Native API that `aioesphomeapi` connects to
- `micro_wake_word` runs the "okay_nabu" model on-device
- On wake word detection: LED pulses blue, voice assistant pipeline starts
- The voice assistant streams audio to whatever client is connected via the API
- TTS audio sent back from the client is played on the speaker automatically

### 3.2 Network Configuration

| Device | IP | Port | Role |
|---|---|---|---|
| ReSpeaker Lite | `192.168.1.50` | 6053 (API) | Voice satellite |
| GPU PC | `192.168.1.100` | — | Voice server (client) |

---

## 4. Python Server Design

### 4.1 Module Structure

```
voice_control/
├── robbie_voice_server.py      # Main entry point, asyncio event loop
├── esphome_client.py           # aioesphomeapi connection + voice pipeline handler
├── stt_engine.py               # Faster-Whisper wrapper
├── intent_classifier.py        # Text → intent mapping
├── stop_detector.py            # Fast-path keyword spotter
├── tts_engine.py               # Piper TTS wrapper
├── llm_client.py               # Ollama/Mistral client
├── ros2_dispatcher.py          # rclpy publisher for all ROS 2 topics
├── console_logger.py           # Simple stdout logging
├── config/
│   ├── locations.yaml          # Named locations with coords + aliases
│   ├── intents.yaml            # Intent definitions + example phrases
│   ├── head_positions.yaml     # Direction → pan/tilt mapping
│   └── voice_config.yaml       # Server settings (IPs, model paths, thresholds)
└── requirements.txt
```

### 4.2 Class Design

```
┌──────────────────────────────────────────────────────────────┐
│ RobbieVoiceServer (robbie_voice_server.py)                   │
│                                                              │
│ - Owns asyncio event loop                                    │
│ - Initializes all components                                 │
│ - Runs rclpy in a background thread                          │
│ - Coordinates pipeline: audio → STT → intent → dispatch      │
│                                                              │
│ Methods:                                                     │
│   async run()                                                │
│   async handle_pipeline(audio_data: bytes) → str             │
│   shutdown()                                                 │
└──────────┬───────────────────────────────────────────────────┘
           │ uses
           ▼
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│ ESPHomeClient    │  │ STTEngine        │  │ StopDetector     │
│                  │  │                  │  │                  │
│ connect(ip,port) │  │ __init__(model)  │  │ check(audio)→   │
│ on_voice_start() │  │ transcribe(pcm)  │  │   bool           │
│ on_audio_data()  │  │   → str          │  │                  │
│ send_tts(audio)  │  │                  │  │ KEYWORDS =       │
│                  │  │ Uses CUDA        │  │  [stop,halt,     │
│ Uses             │  │ Keeps model      │  │   freeze]        │
│ aioesphomeapi    │  │ loaded in VRAM   │  │                  │
└──────────────────┘  └──────────────────┘  └──────────────────┘

┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│ IntentClassifier │  │ TTSEngine        │  │ LLMClient        │
│                  │  │                  │  │                  │
│ __init__(config) │  │ __init__(voice)  │  │ __init__(model)  │
│ classify(text)   │  │ synthesize(text) │  │ ask(question)    │
│  → Intent        │  │  → bytes (PCM)   │  │  → str           │
│                  │  │                  │  │                  │
│ Loads intents    │  │ Piper male voice │  │ ollama.chat()    │
│ .yaml + aliases  │  │ Returns PCM      │  │ Mistral 7B       │
│                  │  │ audio bytes      │  │ Streaming resp   │
└──────────────────┘  └──────────────────┘  └──────────────────┘

┌──────────────────┐  ┌──────────────────┐
│ ROS2Dispatcher   │  │ ConsoleLogger    │
│                  │  │                  │
│ __init__()       │  │ log_wake()       │
│ publish_intent() │  │ log_utterance()  │
│ publish_stop()   │  │ log_intent()     │
│ publish_head()   │  │ log_response()   │
│ publish_nav()    │  │ log_error()      │
│                  │  │                  │
│ rclpy Node       │  │ Print to stdout  │
│ runs in thread   │  │ with timestamps  │
└──────────────────┘  └──────────────────┘
```

### 4.3 Intent Data Model

```python
@dataclass
class Intent:
    name: str               # "goto", "look", "stop_all", etc.
    utterance: str          # Raw transcription
    confidence: float       # 0.0 - 1.0
    params: dict            # Intent-specific params
    response_text: str      # What to speak back via TTS
```

### 4.4 Threading Model

```
┌─────────────────────────────────────────────────┐
│  Main Thread (asyncio)                          │
│                                                 │
│  - aioesphomeapi connection                     │
│  - Audio receive callbacks                      │
│  - Pipeline orchestration                       │
│  - TTS audio send-back                          │
│  - Console logging                              │
│                                                 │
│  Calls into STT/TTS/LLM via                    │
│  asyncio.run_in_executor() for blocking ops     │
└─────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────┐
│  ROS 2 Thread (daemon)                          │
│                                                 │
│  - rclpy.spin() in background thread            │
│  - Publishers created on init                   │
│  - publish_*() methods are thread-safe          │
│  - Subscribes to /battery_state, /diagnostics   │
└─────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────┐
│  Executor Pool (ThreadPoolExecutor)             │
│                                                 │
│  - STT transcription (GPU-bound)                │
│  - TTS synthesis (GPU-bound)                    │
│  - LLM queries (GPU-bound, longer running)      │
└─────────────────────────────────────────────────┘
```

---

## 5. Pipeline Sequence Diagrams

### 5.1 Normal Command (e.g. "go to the kitchen")

```
User          ESP32           VoiceServer        STT         Classifier    Dispatcher    TTS         ESP32
 │              │                  │               │              │             │           │           │
 │ "ok nabu"    │                  │               │              │             │           │           │
 │─────────────►│                  │               │              │             │           │           │
 │              │ wake detected    │               │              │             │           │           │
 │              │ LED pulse blue   │               │              │             │           │           │
 │              │ pipeline_start   │               │              │             │           │           │
 │              │─────────────────►│               │              │             │           │           │
 │              │                  │ log: WAKE     │              │             │           │           │
 │ "go to the   │                  │               │              │             │           │           │
 │  kitchen"    │                  │               │              │             │           │           │
 │─────────────►│ audio chunks     │               │              │             │           │           │
 │              │─────────────────►│ buffer audio  │              │             │           │           │
 │              │─────────────────►│ + check stop  │              │             │           │           │
 │              │─────────────────►│ (not stop)    │              │             │           │           │
 │              │                  │               │              │             │           │           │
 │ (silence)    │ VAD end          │               │              │             │           │           │
 │              │─────────────────►│               │              │             │           │           │
 │              │                  │ transcribe()  │              │             │           │           │
 │              │                  │──────────────►│              │             │           │           │
 │              │                  │  "go to the   │              │             │           │           │
 │              │                  │◄──────────────│              │             │           │           │
 │              │                  │   kitchen"    │              │             │           │           │
 │              │                  │               │              │             │           │           │
 │              │                  │ classify()    │              │             │           │           │
 │              │                  │──────────────────────────►   │             │           │           │
 │              │                  │  goto(kitchen)│              │             │           │           │
 │              │                  │◄──────────────────────────   │             │           │           │
 │              │                  │               │              │             │           │           │
 │              │                  │ log: "go to the kitchen" → goto(kitchen)  │           │           │
 │              │                  │               │              │             │           │           │
 │              │                  │ publish intent + nav goal    │             │           │           │
 │              │                  │─────────────────────────────►│             │           │           │
 │              │                  │               │              │             │           │           │
 │              │                  │ synthesize("navigating to the kitchen")    │           │           │
 │              │                  │────────────────────────────────────────────►│           │           │
 │              │                  │               │              │             │ PCM audio  │           │
 │              │                  │◄────────────────────────────────────────────│           │           │
 │              │  TTS audio       │               │              │             │           │           │
 │              │◄─────────────────│               │              │             │           │           │
 │ (speaker)    │                  │               │              │             │           │           │
 │◄─────────────│ LED off          │               │              │             │           │           │
```

### 5.2 Fast-Path Stop

```
User          ESP32           VoiceServer        StopDetect    Dispatcher
 │              │                  │                  │             │
 │ "ok nabu"    │                  │                  │             │
 │─────────────►│ pipeline_start   │                  │             │
 │              │─────────────────►│                  │             │
 │ "stop!"      │ audio chunk 1    │                  │             │
 │─────────────►│─────────────────►│ check("stop")   │             │
 │              │                  │─────────────────►│             │
 │              │                  │  MATCH!          │             │
 │              │                  │◄─────────────────│             │
 │              │                  │                  │             │
 │              │                  │ IMMEDIATE publish_stop()       │
 │              │                  │────────────────────────────────►│
 │              │                  │                  │             │ /voice/stop
 │              │                  │                  │             │ /cmd_vel zeros
 │              │                  │                  │             │ /drive zeros
 │              │                  │                  │             │
 │              │                  │ log: FAST STOP   │             │
 │              │  TTS "stopping"  │                  │             │
 │              │◄─────────────────│                  │             │
 │ (speaker)    │                  │                  │             │
 │◄─────────────│                  │                  │             │
```

**Fast-path timing budget:**
```
Audio arrives at server:           0ms
Stop keyword detection:          ~50ms  (simple pattern match on small buffer)
ROS 2 publish:                   ~5ms
────────────────────────────────────
Total (server-side):            ~55ms
+ WiFi latency:                ~10-20ms
+ ESP32 pipeline start:        ~50ms
────────────────────────────────────
Total end-to-end:             ~120-130ms  (well under 500ms target)
```

### 5.3 General Question (LLM path)

```
User          ESP32           VoiceServer        STT       Classifier    LLM         TTS         ESP32
 │ "what's the  │ audio            │               │            │           │           │           │
 │  capital of  │─────────────────►│               │            │           │           │           │
 │  France"     │                  │ transcribe()  │            │           │           │           │
 │              │                  │──────────────►│            │           │           │           │
 │              │                  │◄──────────────│            │           │           │           │
 │              │                  │ classify()    │            │           │           │           │
 │              │                  │──────────────────────────►│            │           │           │
 │              │                  │ general_question           │           │           │           │
 │              │                  │◄──────────────────────────│            │           │           │
 │              │                  │               │            │           │           │           │
 │              │                  │ ask("what's the capital of France")    │           │           │
 │              │                  │────────────────────────────────────────►│           │           │
 │              │                  │ "The capital of France is Paris"       │           │           │
 │              │                  │◄────────────────────────────────────────│           │           │
 │              │                  │               │            │           │           │           │
 │              │                  │ synthesize("The capital of France is Paris")       │           │
 │              │                  │────────────────────────────────────────────────────►│           │
 │              │  TTS audio       │               │            │           │           │           │
 │              │◄─────────────────│               │            │           │           │           │
 │◄─────────────│                  │               │            │           │           │           │
```

---

## 6. Intent Classifier Design

### 6.1 Classification Strategy

A **rule-based classifier** with fuzzy matching. No ML model needed for a fixed vocabulary.

```
Input text
    │
    ▼
┌───────────────┐    match     ┌──────────────┐
│ Exact keyword │───────────►  │ Return intent │
│ patterns      │              └──────────────┘
└───────┬───────┘
        │ no match
        ▼
┌───────────────┐    match     ┌──────────────┐
│ Fuzzy match   │───────────►  │ Return intent │
│ (fuzzywuzzy)  │              │ + confidence  │
└───────┬───────┘              └──────────────┘
        │ no match (score < 60)
        ▼
┌───────────────┐    looks     ┌──────────────┐
│ Question      │─ like Q? ──► │ general_      │
│ detector      │              │ question      │
└───────┬───────┘              └──────────────┘
        │ no
        ▼
┌──────────────┐
│ unknown      │
└──────────────┘
```

### 6.2 Pattern Rules (loaded from intents.yaml)

```yaml
# config/intents.yaml

stop_all:
  priority: 1  # checked first
  patterns:
    - "^stop$"
    - "^halt$"
    - "^freeze$"
    - "stop everything"
    - "emergency stop"
  response: "stopping"

look:
  patterns:
    - "look (left|right|up|down|forward|straight|ahead)"
    - "turn your head (left|right|up|down)"
    - "look straight ahead"
  param_extract:
    direction:
      "left": "left"
      "right": "right"
      "up": "up"
      "down": "down"
      "forward|straight|ahead|front": "forward"
  response: "looking {direction}"

goto:
  patterns:
    - "(go|navigate|move|drive) to (the )?(.*)"
  param_extract:
    location: "$3"  # capture group 3 matched against locations.yaml aliases
  response: "navigating to {location}"

dock:
  patterns:
    - "^dock$"
    - "go dock"
    - "start docking"
    - "go charge"
  response: "docking"

undock:
  patterns:
    - "^undock$"
    - "leave the dock"
    - "undock yourself"
  response: "undocking"

cal:
  patterns:
    - "^calibrate$"
    - "^reset$"
    - "^cal$"
  response: "calibrating"

point_to:
  patterns:
    - "point (at|to|towards) (.*)"
  param_extract:
    target: "$2"
  response: "pointing at {target}"

query_status:
  patterns:
    - "(what|how).*battery"
    - "any errors"
    - "what room"
    - "are you docked"
    - "how are you"
    - "list nodes"
  param_extract:
    subject:
      "battery|charge|power": "battery"
      "error|fault|problem": "errors"
      "node": "nodes"
      "room|where": "room"
      "dock": "dock"
      "default": "general"
  response: null  # response generated from queried data

list_topics:
  patterns:
    - "(list|show|what) topics"
  response: null  # response generated from ros2 topic list

list_node_params:
  patterns:
    - "(show|list|what) param.* (for|of) (\\w+)"
  param_extract:
    node_name: "$3"
  response: null  # response generated from ros2 param list

general_question:
  detect_as_question: true  # fallback: if text looks like a question
  response: null  # response from LLM
```

### 6.3 Location Resolution

```yaml
# config/locations.yaml

locations:
  charging_station:
    x: 1.0
    y: 0.0
    yaw_deg: 0.0
    aliases: ["charger", "charge", "charging", "charging station"]

  foyer:
    x: 3.7
    y: 1.37
    yaw_deg: 271.0
    aliases: ["hallway", "entrance", "front door"]

  tv:
    x: -1.35
    y: 3.17
    yaw_deg: 87.89
    aliases: ["television", "lounge", "living room"]

  kitchen:
    x: 2.16
    y: 4.53
    yaw_deg: 137.33
    aliases: []

  dining:
    x: 4.10
    y: 6.92
    yaw_deg: 133.45
    aliases: ["dining room", "dining table"]
```

Location matching: exact match on name or aliases, then fuzzy match (threshold 80%).

---

## 7. Fast-Path Stop Detector Design

The stop detector runs **in parallel** with audio buffering, checking each audio chunk as it arrives.

### 7.1 Strategy

Use **Faster-Whisper in streaming mode with a tiny buffer** (~0.5s of audio). Run the `tiny` model (very fast) on just the first chunk to check for stop words.

Alternative (simpler): Use a dedicated keyword spotter like `openwakeword` configured with "stop", "halt", "freeze" as additional wake words. This avoids running Whisper twice.

**Recommended approach**: Run Faster-Whisper `tiny` model on the first 0.5s audio chunk. If transcription contains stop keywords → immediate dispatch. Otherwise, continue buffering for full transcription with the `small` model.

```python
# stop_detector.py (pseudocode)

STOP_KEYWORDS = {"stop", "halt", "freeze", "emergency"}

class StopDetector:
    def __init__(self, whisper_model_tiny):
        self.model = whisper_model_tiny  # tiny model, ~75ms inference

    async def check(self, audio_chunk: bytes) -> bool:
        """Check first ~0.5s of audio for stop keywords."""
        text = self.model.transcribe(audio_chunk)
        words = set(text.lower().split())
        return bool(words & STOP_KEYWORDS)
```

### 7.2 VRAM Note

The `tiny` model for stop detection uses ~0.1GB additional VRAM. Updated budget:

| Model | VRAM |
|---|---|
| Faster-Whisper tiny (stop detect) | ~0.1 GB |
| Faster-Whisper small (full STT) | ~1.0 GB |
| Piper TTS | ~0.2 GB |
| Mistral 7B Q4 | ~5.0 GB |
| **Total** | **~6.3 GB** |

---

## 8. TTS Response Design

### 8.1 Response Templates

Each intent has a response template. Some are static, some are dynamic:

```python
RESPONSES = {
    "stop_all": "stopping",
    "look": "looking {direction}",
    "goto": "navigating to {location}",
    "dock": "docking",
    "undock": "undocking",
    "cal": "calibrating",
    "point_to": "pointing at {target}",
    "unknown": "I heard '{utterance}' but I don't know that command",
    # query_status, list_topics, list_node_params, general_question → dynamic
}
```

### 8.2 Dynamic Responses

For `query_status`:
```python
# Read from /battery_state topic
"battery is at {percentage} percent"
"no errors reported" / "there are {n} errors: {details}"
"you are in the {room}" / "I'm not sure which room"
"I am docked" / "I am not docked"
```

For `general_question`:
```python
# Send to Ollama Mistral 7B with a system prompt:
SYSTEM_PROMPT = """You are Robbie, a helpful robot assistant.
Give brief, conversational answers (1-2 sentences max).
If you don't know, say so."""
```

### 8.3 Audio Format

Piper outputs 16-bit PCM at the voice model's native rate (typically 22050 Hz). ESPHome voice assistant expects audio in the format configured for the speaker component. May need resampling to match.

---

## 9. Console Log Design

Simple timestamp-prefixed log lines to stdout:

```
[14:32:01] WAKE  "ok nabu" detected
[14:32:03] HEAR  "go to the kitchen"
[14:32:03] INTENT goto → kitchen (x=2.16, y=4.53, yaw=137.33°)
[14:32:03] PUB   /voice/intent, /nav_goal
[14:32:04] TTS   "navigating to the kitchen"
[14:32:04] ─────────────────────────────────
[14:32:10] WAKE  "ok nabu" detected
[14:32:11] HEAR  "stop"
[14:32:11] FAST  stop_all → /voice/stop, /cmd_vel, /drive
[14:32:11] TTS   "stopping"
[14:32:11] ─────────────────────────────────
[14:32:20] WAKE  "ok nabu" detected
[14:32:22] HEAR  "what is the capital of France"
[14:32:22] INTENT general_question
[14:32:23] LLM   "The capital of France is Paris."
[14:32:24] TTS   "The capital of France is Paris."
[14:32:24] ─────────────────────────────────
[14:32:30] WAKE  "ok nabu" detected
[14:32:32] HEAR  "blinkify the wotsit"
[14:32:32] INTENT unknown
[14:32:33] TTS   "I heard 'blinkify the wotsit' but I don't know that command"
[14:32:33] ─────────────────────────────────
```

---

## 10. Configuration File Schemas

### 10.1 voice_config.yaml

```yaml
# config/voice_config.yaml

esphome:
  host: "192.168.1.50"
  port: 6053
  encryption_key: "your-key-here"  # matches ESPHome api.encryption.key

stt:
  model: "small"           # faster-whisper model size
  language: "en"
  device: "cuda"

stop_detector:
  model: "tiny"            # fast model for stop keyword spotting
  keywords: ["stop", "halt", "freeze", "emergency"]

tts:
  model: "en_US-lessac-medium"   # Piper male voice
  speaker_id: null
  output_sample_rate: 22050

llm:
  model: "mistral"         # Ollama model name
  host: "http://localhost:11434"
  system_prompt: |
    You are Robbie, a helpful robot assistant.
    Give brief, conversational answers (1-2 sentences max).
    If you don't know, say so.
  max_tokens: 100

vad:
  silence_threshold_ms: 1500
  max_duration_ms: 5000

ros2:
  node_name: "robbie_voice"
  topics:
    intent: "/voice/intent"
    stop: "/voice/stop"
    tts_text: "/voice/tts_text"
    cmd_vel: "/cmd_vel"
    drive: "/drive"
    head_position: "/head/position"
```

### 10.2 head_positions.yaml

```yaml
# config/head_positions.yaml

head:
  left:    [1.2, 0.0]     # [pan, tilt]
  right:   [-1.2, 0.0]
  up:      [0.0, -0.5]
  down:    [0.0, 0.5]
  forward: [0.0, 0.0]
  front:   [0.0, 0.0]     # alias for forward
```

---

## 11. Error Handling Design

| Error | Handling |
|---|---|
| ESP32 disconnects | Log error, retry connection every 5s, publish `/voice/status` offline |
| STT returns empty | Treat as `unknown`, TTS "I didn't catch that" |
| STT timeout (>3s) | Cancel transcription, TTS "sorry, processing took too long" |
| Ollama unreachable | TTS "I can't answer questions right now", log error |
| ROS 2 publish fails | Log error, still send TTS response |
| Unknown location | TTS "I don't know where {location} is", log |

---

## 12. Startup Sequence

```
1. Load config files (voice_config.yaml, locations.yaml, intents.yaml, head_positions.yaml)
2. Initialize ROS 2 node (rclpy) in background thread
3. Load Faster-Whisper models (tiny + small) → CUDA
4. Load Piper TTS model
5. Verify Ollama is running (health check)
6. Connect to ESPHome device via aioesphomeapi
7. Register voice pipeline handler
8. Log "Robbie Voice Server ready" to console
9. Enter main asyncio loop
```

---

## 13. Dependencies & Versions

```
# requirements.txt
aioesphomeapi>=24.0
faster-whisper>=1.0.0
piper-tts>=1.2.0
webrtcvad>=2.0.10
ollama>=0.3.0
pyyaml>=6.0
rclpy  # from ROS 2 Jazzy
std_msgs  # from ROS 2
geometry_msgs  # from ROS 2
sensor_msgs  # from ROS 2
diagnostic_msgs  # from ROS 2
```

---

## 14. Next Steps

1. **`/sc:workflow`** - Generate ordered implementation plan
2. **`/sc:implement`** - Build components in order:
   - ESPHome YAML config + flash to ReSpeaker
   - voice_config.yaml + locations.yaml + intents.yaml
   - esphome_client.py (verify audio streaming works)
   - stt_engine.py (verify transcription works)
   - intent_classifier.py + stop_detector.py
   - ros2_dispatcher.py
   - tts_engine.py + llm_client.py
   - robbie_voice_server.py (wire it all together)
   - Console logger
   - End-to-end testing
