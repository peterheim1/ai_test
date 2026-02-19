# Robbie Voice Control - Implementation Workflow

**Source**: `voice_control_requirements.md`, `voice_control_design.md`
**Strategy**: Systematic, bottom-up (hardware → transport → pipeline → integration)
**Principle**: Each phase produces a testable artifact before moving to the next.

---

## Phase 0: Environment Setup
> Get all dependencies installed and verify hardware connectivity.

### Task 0.1: GPU PC Python Environment
- Create `voice_control/` directory structure per design §4.1
- Create `requirements.txt` per design §13
- Create Python venv: `python3 -m venv venv`
- Install: `pip install aioesphomeapi faster-whisper piper-tts webrtcvad ollama pyyaml`
- Verify CUDA available: `python3 -c "from faster_whisper import WhisperModel; m = WhisperModel('tiny', device='cuda'); print('OK')"`
- **Checkpoint**: Faster-Whisper tiny model loads on CUDA without error

### Task 0.2: Ollama + Mistral
- Install Ollama if not present
- Pull model: `ollama pull mistral`
- Verify: `ollama run mistral "say hello in one word"`
- **Checkpoint**: Mistral responds locally

### Task 0.3: Piper TTS
- Install piper-tts, download male voice model (e.g. `en_US-lessac-medium`)
- Test: generate a WAV from "hello world" and play it
- **Checkpoint**: Piper produces audible speech audio

### Task 0.4: ESPHome Toolchain
- Install ESPHome: `pip install esphome`
- Verify: `esphome version`
- **Checkpoint**: ESPHome CLI available

**Phase 0 gate**: All tools installed, CUDA works, Ollama responds, Piper speaks.

---

## Phase 1: Configuration Files
> Create all YAML config files. No code yet — just data.

### Task 1.1: voice_config.yaml
- Create `voice_control/config/voice_config.yaml` per design §10.1
- Set ESP32 IP, port, encryption key placeholder
- Set STT model sizes, TTS voice, LLM settings, VAD thresholds
- Set ROS 2 topic names

### Task 1.2: locations.yaml
- Create `voice_control/config/locations.yaml` per design §6.3
- All 5 locations with coordinates and aliases from requirements §FR-7

### Task 1.3: intents.yaml
- Create `voice_control/config/intents.yaml` per design §6.2
- All 12 intents with regex patterns, param extraction rules, response templates

### Task 1.4: head_positions.yaml
- Create `voice_control/config/head_positions.yaml` per design §10.2
- 6 direction entries with pan/tilt values from requirements §FR-8

**Phase 1 gate**: All 4 YAML files created and parseable with `python3 -c "import yaml; yaml.safe_load(open('...'))"`.

---

## Phase 2: ESPHome Firmware
> Flash the ReSpeaker Lite and verify it boots with wake word detection.

### Task 2.1: ESPHome YAML Config
- Create `robbie_voice_satellite.yaml` per design §3.1
- Configure: WiFi static IP, API encryption, I2S mic + speaker, micro_wake_word, LED
- Create `secrets.yaml` with WiFi credentials and API key

### Task 2.2: Flash & Boot Test
- Connect ReSpeaker Lite via USB
- Run: `esphome run robbie_voice_satellite.yaml`
- Verify ESPHome logs show: WiFi connected, API listening on port 6053
- **Checkpoint**: ESP32 boots and connects to WiFi

### Task 2.3: Wake Word Test
- Say "ok nabu" near the ReSpeaker
- Verify in ESPHome logs: wake word detected event
- Verify LED pulses on detection
- **Checkpoint**: Wake word triggers reliably

### Task 2.4: API Connection Test
- Write a minimal Python script using `aioesphomeapi` to connect to ESP32
- Verify connection established, device info retrieved
- **Checkpoint**: `aioesphomeapi` can connect to ESP32

**Phase 2 gate**: ESP32 running, wake word works, Python can connect via API.

---

## Phase 3: Audio Pipeline (ESP32 → PC)
> Get audio streaming from ESP32 to PC and verify it's usable audio.

### Task 3.1: esphome_client.py
- Implement `ESPHomeClient` class per design §4.2
- Connect to ESP32 via `aioesphomeapi`
- Register voice assistant pipeline handler
- Receive `on_voice_start` and `on_audio_data` callbacks
- Buffer incoming PCM audio chunks into a `bytearray`
- Implement VAD-based end-of-speech detection (silence timeout)
- Implement hard timeout (5s max)
- **Depends on**: Task 2.4

### Task 3.2: Audio Capture Test
- Say "ok nabu" then speak a test phrase
- Save buffered audio to a WAV file
- Play WAV file on PC and verify it's clear, audible speech
- **Checkpoint**: Audio captured from ESP32 is intelligible when played back

**Phase 3 gate**: Clear audio arrives on PC after wake word, with proper start/end detection.

---

## Phase 4: Speech-to-Text
> Transcribe captured audio and verify accuracy.

### Task 4.1: stt_engine.py
- Implement `STTEngine` class per design §4.2
- Load Faster-Whisper `small` model with CUDA
- `transcribe(pcm_audio: bytes) -> str` method
- Keep model loaded in VRAM (initialize once)
- **Depends on**: Task 0.1 (CUDA verified)

### Task 4.2: STT Integration Test
- Wire `esphome_client.py` → `stt_engine.py`
- Say "ok nabu, go to the kitchen"
- Print transcription to console
- Verify accuracy on 5+ test phrases:
  - "go to the kitchen"
  - "look left"
  - "stop"
  - "what's the battery level"
  - "what is the capital of France"
- Measure transcription latency (target: <500ms)
- **Checkpoint**: 5/5 test phrases transcribed correctly, latency under 500ms

**Phase 4 gate**: STT accurately transcribes short commands within latency target.

---

## Phase 5: Fast-Path Stop Detector
> Implement and test the emergency stop fast-path.

### Task 5.1: stop_detector.py
- Implement `StopDetector` class per design §7.1
- Load Faster-Whisper `tiny` model (separate from main STT)
- `check(audio_chunk: bytes) -> bool` method
- Check first ~0.5s of audio for stop keywords
- **Depends on**: Task 0.1

### Task 5.2: Stop Detection Test
- Feed audio of "stop", "halt", "freeze" to detector
- Verify all three detected correctly
- Feed audio of "go to the kitchen" — verify NOT detected as stop
- Measure detection latency (target: <100ms for 0.5s chunk)
- **Checkpoint**: Stop words detected in <100ms, no false positives on non-stop phrases

**Phase 5 gate**: Stop detector is fast and accurate.

---

## Phase 6: Intent Classifier
> Map transcribed text to intents with correct parameters.

### Task 6.1: intent_classifier.py
- Implement `IntentClassifier` class per design §6.1
- Load `intents.yaml` and `locations.yaml` on init
- `classify(text: str) -> Intent` method
- Implement pattern matching pipeline: exact → regex → fuzzy → question → unknown
- Implement location alias resolution with fuzzy matching (threshold 80%)
- Implement direction synonym resolution
- Return `Intent` dataclass with name, params, confidence, response_text
- **Depends on**: Task 1.2, Task 1.3

### Task 6.2: Classifier Unit Tests
- Test each intent with its example phrases from requirements §FR-6:
  - "look left" → `look(direction=left)`
  - "go to the living room" → `goto(location=tv)`
  - "stop" → `stop_all()`
  - "what's the battery level" → `query_status(subject=battery)`
  - "dock" → `dock()`
  - "undock" → `undock()`
  - "calibrate" → `cal()`
  - "show topics" → `list_topics()`
  - "show params for move_group" → `list_node_params(node_name=move_group)`
  - "what is the capital of France" → `general_question(original=...)`
  - "blinkify the wotsit" → `unknown(original=...)`
- Test location aliases: "living room" → tv, "charger" → charging_station, "front door" → foyer
- Test edge cases: empty string, very long string, partial matches
- **Checkpoint**: All example phrases correctly classified

**Phase 6 gate**: All intents classified correctly with proper parameter extraction.

---

## Phase 7: TTS Engine
> Generate speech audio and stream it back to ESP32 speaker.

### Task 7.1: tts_engine.py
- Implement `TTSEngine` class per design §4.2
- Load Piper male voice model on init
- `synthesize(text: str) -> bytes` method returning PCM audio
- Handle sample rate conversion if needed for ESP32 playback
- **Depends on**: Task 0.3

### Task 7.2: TTS Playback Test
- Generate TTS for "navigating to the kitchen"
- Stream audio back to ESP32 via `aioesphomeapi` voice pipeline response
- Verify audio plays clearly from ReSpeaker Lite speaker
- **Checkpoint**: TTS audio plays from robot speaker, clearly audible
- **Depends on**: Task 3.1 (ESPHome client)

**Phase 7 gate**: TTS audio plays from ESP32 speaker via ESPHome API.

---

## Phase 8: LLM Client
> Route general questions to Mistral and get text responses.

### Task 8.1: llm_client.py
- Implement `LLMClient` class per design §4.2
- Connect to Ollama at `http://localhost:11434`
- `ask(question: str) -> str` method
- System prompt per design §8.2 (brief, conversational robot persona)
- Max tokens: 100 (keep responses short for TTS)
- Health check method for startup verification
- **Depends on**: Task 0.2

### Task 8.2: LLM Integration Test
- Ask: "what is the capital of France" → verify sensible answer
- Ask: "tell me a joke" → verify response
- Measure response latency (expect 1-3s for Mistral 7B)
- **Checkpoint**: LLM responds correctly, latency acceptable

**Phase 8 gate**: LLM answers general questions with short, spoken-friendly responses.

---

## Phase 9: ROS 2 Dispatcher
> Publish intents and commands to ROS 2 topics.

### Task 9.1: ros2_dispatcher.py
- Implement `ROS2Dispatcher` class per design §4.2
- Create ROS 2 node `robbie_voice` in background thread
- Create publishers for all topics per design §2.1:
  - `/voice/intent` (String) — JSON-encoded intent
  - `/voice/stop` (Empty) — fast-path stop
  - `/voice/tts_text` (String) — spoken response text
  - `/cmd_vel` (Twist) — navigation commands / stop zeros
  - `/drive` (Float64MultiArray) — direct motor stop
  - `/head/position` (Float64MultiArray) — head pan/tilt
- Create subscribers:
  - `/battery_state` (BatteryState) — cache latest for status queries
  - `/diagnostics` (DiagnosticArray) — cache latest for error queries
- Implement `publish_intent(intent)` — publishes JSON + action-specific topics
- Implement `publish_stop()` — publishes to /voice/stop, /cmd_vel zeros, /drive zeros simultaneously
- Implement `publish_head(pan, tilt)` — publishes Float64MultiArray
- Thread-safe: publishers called from asyncio thread, rclpy spins in daemon thread
- **Depends on**: Task 1.1 (topic names from config)

### Task 9.2: ROS 2 Publish Test
- Start dispatcher
- Call `publish_stop()` → verify with `ros2 topic echo /voice/stop`
- Call `publish_head(1.2, 0.0)` → verify with `ros2 topic echo /head/position`
- Call `publish_intent(goto_intent)` → verify JSON on `/voice/intent`
- **Checkpoint**: All topics publishing correctly, visible via ros2 CLI

**Phase 9 gate**: All ROS 2 topics publishing correctly.

---

## Phase 10: Console Logger
> Simple stdout logging for all voice interactions.

### Task 10.1: console_logger.py
- Implement `ConsoleLogger` class per design §9
- Methods: `log_wake()`, `log_utterance(text)`, `log_intent(intent)`, `log_response(text)`, `log_error(msg)`, `log_fast_stop()`
- Format: `[HH:MM:SS] TYPE  message` with separator lines
- Print to stdout, no file logging needed
- **No dependencies** — pure formatting

**Phase 10 gate**: Logger produces clean, readable output matching design §9 format.

---

## Phase 11: Main Server Integration
> Wire all components together into the main server.

### Task 11.1: robbie_voice_server.py
- Implement `RobbieVoiceServer` class per design §4.2
- Startup sequence per design §12:
  1. Load all config files
  2. Initialize ROS 2 dispatcher (background thread)
  3. Load Faster-Whisper models (tiny + small) on CUDA
  4. Load Piper TTS
  5. Verify Ollama health
  6. Connect to ESPHome
  7. Register voice pipeline handler
  8. Log "ready" to console
- Main pipeline per design §5.1:
  1. Receive pipeline start event → log wake
  2. Buffer audio chunks
  3. [Parallel] Run stop detector on first chunk
  4. On stop detected → fast-path publish + TTS "stopping"
  5. On silence/timeout → run full STT
  6. Classify intent
  7. Dispatch to ROS 2
  8. Generate response text
  9. If general_question → query LLM first
  10. Synthesize TTS → stream back to ESP32
  11. Log everything to console
- Graceful shutdown on Ctrl+C: stop ROS 2 thread, disconnect ESP32
- **Depends on**: ALL previous tasks (3.1, 4.1, 5.1, 6.1, 7.1, 8.1, 9.1, 10.1)

### Task 11.2: Smoke Test
- Start server: `python3 robbie_voice_server.py`
- Verify startup log shows all components initialized
- Say "ok nabu, look left" → verify:
  - Console shows: WAKE → HEAR → INTENT → PUB → TTS
  - `/head/position` publishes [1.2, 0.0]
  - Speaker says "looking left"
- **Checkpoint**: Single command works end-to-end

**Phase 11 gate**: Server starts, processes one command end-to-end.

---

## Phase 12: End-to-End Testing
> Test every intent and edge case.

### Task 12.1: Test All Intents
Run each intent through the full pipeline (wake word → speaker response):

| # | Say | Expected Intent | Expected Response |
|---|---|---|---|
| 1 | "ok nabu, look left" | look(left) | "looking left" + /head/position |
| 2 | "ok nabu, look up" | look(up) | "looking up" + /head/position |
| 3 | "ok nabu, go to the kitchen" | goto(kitchen) | "navigating to the kitchen" |
| 4 | "ok nabu, go to the living room" | goto(tv) | "navigating to the living room" |
| 5 | "ok nabu, navigate to the charger" | goto(charging_station) | "navigating to the charger" |
| 6 | "ok nabu, stop" | stop_all (fast) | "stopping" + zeros on /cmd_vel, /drive |
| 7 | "ok nabu, halt" | stop_all (fast) | "stopping" |
| 8 | "ok nabu, what's the battery level" | query_status(battery) | dynamic battery response |
| 9 | "ok nabu, dock" | dock | "docking" |
| 10 | "ok nabu, undock" | undock | "undocking" |
| 11 | "ok nabu, calibrate" | cal | "calibrating" |
| 12 | "ok nabu, show topics" | list_topics | dynamic topic list |
| 13 | "ok nabu, point at the door" | point_to(the door) | "pointing at the door" |
| 14 | "ok nabu, what is the capital of France" | general_question | LLM answer via TTS |
| 15 | "ok nabu, blinkify wotsit" | unknown | "I heard ... but I don't know that command" |

### Task 12.2: Fast-Path Stop Timing
- Measure time from end of "stop" utterance to `/voice/stop` publish
- Run 5 trials, record latency
- **Checkpoint**: All 5 trials under 500ms

### Task 12.3: Multi-User Test
- Have 2+ family members each say "ok nabu, look left"
- Verify wake word and STT work for different voices
- **Checkpoint**: Commands recognized from multiple speakers

### Task 12.4: Edge Cases
- Say "ok nabu" then stay silent → verify timeout, no crash
- Say "ok nabu" then speak for >5s → verify hard cutoff
- Disconnect ESP32 WiFi → verify server logs error and retries
- Kill Ollama → ask a general question → verify graceful error message
- **Checkpoint**: No crashes on any edge case

**Phase 12 gate**: All 15 intents work, fast stop under 500ms, multi-user works, edge cases handled.

---

## Dependency Graph

```
Phase 0 (Environment Setup)
    │
    ├──► Phase 1 (Config Files) ──► Phase 6 (Intent Classifier)
    │                                        │
    ├──► Phase 2 (ESPHome FW) ──► Phase 3 (Audio Pipeline) ──► Phase 4 (STT)
    │                                   │                          │
    │                                   └──► Phase 7 (TTS) ◄──────┤
    │                                                              │
    ├──► Phase 5 (Stop Detector)                                   │
    │                                                              │
    ├──► Phase 8 (LLM Client)                                     │
    │                                                              │
    ├──► Phase 9 (ROS 2 Dispatcher)                                │
    │                                                              │
    └──► Phase 10 (Console Logger)                                 │
                                                                   │
         All phases ──────────────────────────────────────────────►│
                                                                   │
                                                              Phase 11 (Integration)
                                                                   │
                                                              Phase 12 (E2E Testing)
```

### Parallelizable Work

These phases can be developed **in parallel** after Phase 0:

| Track A (Hardware) | Track B (Processing) | Track C (Output) |
|---|---|---|
| Phase 1: Config files | Phase 5: Stop detector | Phase 8: LLM client |
| Phase 2: ESPHome FW | Phase 6: Intent classifier | Phase 10: Console logger |
| Phase 3: Audio pipeline | | Phase 9: ROS 2 dispatcher |
| Phase 4: STT | | Phase 7: TTS |

Phase 11 (integration) requires all tracks complete.

---

## Estimated Task Count

| Phase | Tasks | Description |
|---|---|---|
| 0 | 4 | Environment setup |
| 1 | 4 | Config files |
| 2 | 4 | ESPHome firmware |
| 3 | 2 | Audio pipeline |
| 4 | 2 | STT engine |
| 5 | 2 | Stop detector |
| 6 | 2 | Intent classifier |
| 7 | 2 | TTS engine |
| 8 | 2 | LLM client |
| 9 | 2 | ROS 2 dispatcher |
| 10 | 1 | Console logger |
| 11 | 2 | Main server |
| 12 | 4 | E2E testing |
| **Total** | **33** | |

---

## Critical Path

The longest chain that determines minimum total time:

```
Phase 0 → Phase 2 → Phase 3 → Phase 4 → Phase 11 → Phase 12
(setup)   (flash)   (audio)   (STT)     (integrate) (test)
```

Everything else can be built in parallel alongside this chain.

---

## Next Step

Use `/sc:implement` to begin execution, starting with Phase 0.
