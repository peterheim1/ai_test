# Robbie Voice Control - Requirements Specification

## 1. Project Goal

A fully local, low-latency voice control system for the Robbie robot. A ReSpeaker Lite (XIAO ESP32-S3) running **stock ESPHome firmware** with `micro_wake_word` listens for "ok nabu", then streams audio via the **ESPHome Native API** to a Python server on a GPU PC. The server transcribes speech with Faster-Whisper, classifies intent, dispatches commands to ROS 2, and streams Piper TTS audio back to the ESP32 speaker. A fast-path ensures "stop/halt" commands are dispatched with minimal latency.

---

## 2. System Overview

```
┌─────────────────────────┐  ESPHome Native API   ┌────────────────────────────────────┐
│  ReSpeaker Lite         │ ◄────────────────────► │  GPU PC (RTX 3060)                 │
│  (stock ESPHome FW)     │  (aioesphomeapi)       │                                    │
│                         │                         │  Python server:                    │
│  micro_wake_word        │  audio chunks ───────► │   1. aioesphomeapi client          │
│  "ok nabu" on-device    │                         │   2. Faster-Whisper STT (small)    │
│  LED flash on wake      │  TTS audio ◄────────── │   3. Intent classifier             │
│  Speaker playback       │                         │   4. Fast-path stop detector       │
│                         │                         │   5. Command dispatcher → ROS 2    │
│  No custom firmware     │                         │   6. Piper TTS (male voice)        │
│  needed                 │                         │   7. Mistral 7B via Ollama         │
│                         │                         │   8. Simple console log             │
└─────────────────────────┘                         └────────────────────────────────────┘
                                                                  │
                                                                  ▼ ROS 2 topics
                                                       ┌──────────────────────┐
                                                       │  Robot Control Nodes │
                                                       └──────────────────────┘
```

### Key Decision: Stock ESPHome Firmware + aioesphomeapi

The ReSpeaker Lite already supports ESPHome with the voice assistant pipeline. Instead of custom firmware, we use:

- **ESP32 side**: Stock ESPHome firmware with `micro_wake_word` ("ok nabu"), voice assistant component, LED automation, speaker output. Configured via ESPHome YAML.
- **PC side**: Python server using `aioesphomeapi` library to connect to the ESP32 as a voice pipeline handler - the same protocol Home Assistant uses, but with our own processing.

**Benefits**: No firmware development. OTA updates via ESPHome. Proven audio pipeline. LED and speaker already integrated. The `aioesphomeapi` library is pip-installable and well-maintained.

---

## 3. Functional Requirements

### FR-1: Wake Word Detection (on ESP32)
- Wake word: **"ok nabu"** via ESPHome `micro_wake_word` component (runs on-device)
- On detection: flash onboard RGB LED blue
- On detection: signal voice pipeline start via ESPHome Native API
- Idle state: no audio transmitted, minimal power

### FR-2: Audio Streaming (ESP32 → PC)
- Protocol: **ESPHome Native API** (protobuf over TCP, handled by `aioesphomeapi`)
- Format: 16-bit PCM, 16kHz mono (ESPHome voice assistant default)
- Streaming begins on wake word detection
- VAD: server-side silence detection (~1.5s silence → stop)
- Hard timeout: 5 seconds max recording per utterance

### FR-3: Speech-to-Text
- Engine: **Faster-Whisper** (small or base model) with CUDA on RTX 3060
- Input: PCM audio buffer from ESPHome voice stream
- Output: raw transcription text string
- Expected latency: <500ms for short commands

### FR-4: Fast-Path Stop Detection
- **Before full STT processing**, check incoming audio for stop keywords
- Use a lightweight detector (keyword spotting or very fast small Whisper pass)
- Keywords: "stop", "halt", "freeze"
- On detection: **immediately** publish `stop_all` to ROS 2 (skip full pipeline)
- Target: <500ms from speech to stop command dispatched

### FR-5: Intent Classification
- Map transcribed text to one of the defined intents
- Matching: keyword/pattern matching with fuzzy matching for aliases
- Must resolve location aliases (e.g. "living room" → `tv`)
- Must resolve direction synonyms (e.g. "straight ahead" → `forward`)
- Single intent per utterance (no compound commands)

### FR-6: Defined Intents

| Intent | Params | Examples |
|---|---|---|
| `look` | `direction`: left/right/up/down/forward | "look left", "look straight ahead" |
| `goto` | `location`: from known locations | "go to the kitchen", "navigate to the charger" |
| `point_to` | `target`: freeform string | "point at the table" |
| `stop_all` | none | "stop", "halt", "freeze" (fast-pathed) |
| `query_status` | `subject`: battery/errors/nodes/room/dock/general | "battery level", "any errors" |
| `dock` | none | "dock", "go charge" |
| `undock` | none | "undock", "leave the dock" |
| `cal` | none | "calibrate", "reset" |
| `list_topics` | none | "show topics", "list topics" |
| `list_node_params` | `node_name` | "show params for move_group" |
| `general_question` | `original`: full text | "what is the capital of France" |
| `unknown` | `original`: full text | anything unrecognized |

### FR-7: Known Locations

| Name | Aliases | x | y | yaw_deg |
|---|---|---|---|---|
| `charging_station` | charger, charge, charging | 1.0 | 0.0 | 0.0 |
| `foyer` | hallway, entrance, front door | 3.7 | 1.37 | 271.0 |
| `tv` | television, lounge, living room | -1.35 | 3.17 | 87.89 |
| `kitchen` | *(none)* | 2.16 | 4.53 | 137.33 |
| `dining` | dining room, dining table | 4.10 | 6.92 | 133.45 |

### FR-8: Head Movement Targets

| Direction | Pan, Tilt |
|---|---|
| left | 1.2, 0.0 |
| right | -1.2, 0.0 |
| up | 0.0, -0.5 |
| down | 0.0, 0.5 |
| forward/front | 0.0, 0.0 |

### FR-9: Response - Text-to-Speech
- Engine: **Piper TTS** (male voice) running on GPU PC
- TTS audio streamed back to ESP32 via ESPHome Native API voice pipeline response
- ESP32 plays audio through onboard speaker (handled by ESPHome)
- Used for: command confirmations, status answers, error messages, LLM responses

### FR-10: Response - Console Display
- Simple scrolling log on GPU PC terminal
- Each line shows: timestamp, transcribed text, classified intent, response
- No TUI framework needed - simple print-based log

### FR-11: Error Handling
- On `unknown` intent: speak back what was heard and that it wasn't understood
- Example TTS response: "I heard 'blinkify the wotsit' but I don't know that command"
- Log error in console

### FR-12: General Questions
- Route `general_question` intents to **Mistral 7B** via Ollama (local on GPU PC)
- Speak the LLM's response back via Piper TTS to ESP32 speaker
- Display question and answer in console

### FR-13: Command Dispatch
- Classified intents published to appropriate ROS 2 topics via Python script
- Message format: `std_msgs/Float64MultiArray` where applicable
- Wheel order convention: front_left, rear_left, rear_right, front_right

---

## 4. Non-Functional Requirements

| ID | Requirement | Target |
|---|---|---|
| NFR-1 | End-to-end latency (end of speech → command dispatched) | < 2 seconds |
| NFR-2 | Fast-path stop latency (end of "stop" → command dispatched) | < 500ms |
| NFR-3 | STT inference time | < 500ms on RTX 3060 |
| NFR-4 | TTS generation time | < 500ms for short responses |
| NFR-5 | All processing local, no cloud | 100% offline capable |
| NFR-6 | Network | Static IPs, same WiFi LAN |
| NFR-7 | Wake word false-positive rate | Low (tunable in ESPHome config) |
| NFR-8 | ESP32 firmware | Stock ESPHome (no custom firmware) |
| NFR-9 | Multi-user | Multiple family members, no enrollment needed |
| NFR-10 | LLM VRAM usage | Mistral 7B ~5GB, leaves room for Whisper + Piper |

---

## 5. User Stories

1. **As a user**, I say "ok nabu" and see the LED flash blue, so I know Robbie is listening.
2. **As a user**, I say "ok nabu, go to the kitchen" and the robot navigates within 2 seconds.
3. **As a user**, I say "ok nabu, look left" and the head pans left.
4. **As a user**, I say "ok nabu, stop" and all motion halts within 500ms (fast-path).
5. **As a user**, I say "ok nabu, what's the battery level" and hear a spoken response from the robot.
6. **As a user**, I say something unrecognized and hear what was heard plus an error message.
7. **As a user**, I ask "ok nabu, what's the capital of France" and hear an LLM answer.
8. **As a user**, I see all voice interactions logged in a simple console on the GPU PC.
9. **As a family member**, I can use voice commands without any setup.
10. **As a user**, everything works fully offline.

---

## 6. Component Breakdown

| # | Component | Runs On | Technology |
|---|---|---|---|
| 1 | Wake word + voice pipeline | XIAO ESP32-S3 | ESPHome `micro_wake_word` + `voice_assistant` |
| 2 | LED feedback | XIAO ESP32-S3 | ESPHome `light` automation |
| 3 | Speaker playback | XIAO ESP32-S3 | ESPHome `speaker` component (I2S) |
| 4 | ESPHome API client | GPU PC | `aioesphomeapi` (Python, pip) |
| 5 | VAD (server-side) | GPU PC | Silero VAD or webrtcvad |
| 6 | Fast-path stop detector | GPU PC | Keyword spotting on audio stream |
| 7 | STT engine | GPU PC | Faster-Whisper (small), CUDA |
| 8 | Intent classifier | GPU PC | Python, regex + fuzzy matching |
| 9 | Location resolver | GPU PC | YAML config, alias lookup |
| 10 | Command dispatcher | GPU PC | Python → ROS 2 topic publisher |
| 11 | TTS engine | GPU PC | Piper TTS, male voice |
| 12 | Local LLM | GPU PC | Mistral 7B via Ollama |
| 13 | Console display | GPU PC | Simple Python logging to stdout |

---

## 7. Data Flow per Utterance

```
 1. User says "ok nabu"
 2. ESP32 micro_wake_word detects → LED flashes blue
 3. ESP32 signals voice pipeline start via ESPHome Native API
 4. aioesphomeapi client receives pipeline start event
 5. ESP32 streams audio chunks via Native API
 6. [PARALLEL] Fast-path: check audio for "stop/halt/freeze"
    → If detected: immediately publish stop_all, skip remaining steps
 7. VAD detects ~1.5s silence → signal end of speech
 8. Faster-Whisper transcribes → "go to the living room"
 9. Intent classifier → {intent: "goto", location: "tv"}
10. Console logs: [14:32:05] "go to the living room" → goto(tv)
11. Dispatcher publishes to ROS 2 topic
12. Piper TTS generates "navigating to the living room" (male voice)
13. TTS audio streamed back to ESP32 via Native API
14. ESP32 plays audio on speaker
15. Robot begins navigation
```

---

## 8. GPU VRAM Budget (RTX 3060 12GB)

| Model | Est. VRAM |
|---|---|
| Faster-Whisper small | ~1 GB |
| Piper TTS | ~0.2 GB |
| Mistral 7B (Q4) | ~5 GB |
| **Total** | **~6.2 GB** |
| **Headroom** | **~5.8 GB** |

---

## 9. ESPHome Configuration (ESP32 side)

The ReSpeaker Lite needs an ESPHome YAML config with:
- `micro_wake_word` component configured for "ok nabu"
- `voice_assistant` component (pipeline handled externally)
- `i2s_audio` for microphone input and speaker output
- `light` component for onboard RGB LED with wake word automation
- WiFi with static IP
- Native API enabled (for `aioesphomeapi` connection from GPU PC)

No custom C/C++ firmware. All configuration via ESPHome YAML.

---

## 10. Configuration Files (GPU PC side)

- `locations.yaml` - Named locations with coordinates and aliases
- `intents.yaml` - Intent definitions, params, and example phrases
- `head_positions.yaml` - Head direction to pan/tilt mapping
- `voice_config.yaml` - ESP32 IP, Whisper model size, Piper voice model, VAD thresholds

---

## 11. Python Dependencies (GPU PC)

```
aioesphomeapi          # ESPHome Native API client
faster-whisper         # STT with CUDA support
piper-tts              # Local TTS (male voice)
webrtcvad / silero-vad # Voice activity detection
ollama                 # Mistral 7B client
rclpy                  # ROS 2 Python client
pyyaml                 # Config file parsing
```

---

## 12. Resolved Questions

| Question | Resolution |
|---|---|
| Firmware approach | Stock ESPHome - no custom firmware |
| Piper voice | Male voice |
| Console style | Simple scrolling log |
| Stop command | Fast-pathed for <500ms response |
| Wake word | "ok nabu" via micro_wake_word on ESP32 |
| LLM model | Mistral 7B via Ollama |
| Multiple users | Supported, no enrollment needed |
| Commands | Single intent per utterance |

---

## 13. Remaining Open Questions

1. **ESP32 audio return format** - Confirm ESPHome voice assistant speaker component sample rate/format for TTS playback. Likely 16kHz 16-bit mono but needs verification.
2. **aioesphomeapi voice pipeline** - Verify that `aioesphomeapi` exposes the voice assistant pipeline events (start, audio data, stop) for external handling. The library is primarily used by HA but the voice pipeline API should be accessible.
3. **Piper male voice model** - Choose specific voice (e.g. `en_US-lessac-medium`, `en_GB-alan-medium`). Will select during implementation.

---

## 14. Next Steps

1. **`/sc:design`** - Architecture: message formats, ROS 2 topic names, API interfaces
2. **`/sc:workflow`** - Implementation plan with ordered tasks
3. **`/sc:implement`** - Build, starting with ESPHome YAML config and Python server skeleton
