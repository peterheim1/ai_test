# Voice Control - Project Status

**Date**: 2026-02-19
**Status**: Full pipeline working on desktop. Ready to deploy to robot.

---

## Quick Start

```bash
~/ai_projects/voice_control/start.sh
```

Say **"hey Jarvis"** then a command (e.g. "look right", "go to kitchen", "run task nav3", "stop").

---

## What Works (Desktop - AM4137 USB mic, GPU PC)

- **Wake word**: openwakeword `hey_jarvis` triggers reliably (scores 0.85–0.95)
- **STT**: Faster-Whisper (small, CUDA) transcribes speech correctly
- **Intent classifier**: "look right" → direction=right, pan=-1.2, tilt=0.0
- **ROS 2 dispatcher**: publishes to /voice/intent, /head/position, /cmd_vel etc
- **TTS**: Piper `en_GB-alan-medium` (British male voice, Jarvis-like)
- **Audio**: Captures at 16kHz directly via sounddevice (PipeWire resamples internally)
- **LLM memory**: Ollama (Mistral) retains today's conversation history as context
- **Task runner**: loads and executes .txt sequence files (NAV/MakeSound/Wait/Dock)
- **Web UI**: control panel at http://localhost:8090 with tasks side panel
- **`/voice/speak`**: any ROS2 node can trigger TTS by publishing a String to this topic
- **Config**: `audio_source: "local_mic"` in `config/voice_config.yaml`

---

## Audio Pipeline

```
AM4137 USB mic
  ↓ sounddevice (16kHz, 1024 block, int16)
  ↓ openwakeword hey_jarvis (threshold 0.85, int16 input)
  ↓ webrtcvad (VAD end-of-speech, ~1.5s silence)
  ↓ fast-path stop detector (tiny model, first 0.5s)
  ↓ Faster-Whisper STT (CUDA on desktop / CPU tiny on robot)
  ↓ Intent classifier (regex + fuzzy location matching)
  ↓ ROS 2 dispatcher (/voice/intent, /head/position, /cmd_vel, /drive …)
  ↓ Task runner (if run_task intent)
  ↓ Piper TTS (en_GB-alan-medium)
  ↓ sounddevice playback (mic paused during playback)
```

Key lessons learned:
- Capture at **16kHz natively** — let sounddevice/PipeWire resample internally
- Pass **raw int16** to openwakeword `predict()` — not float32
- Block size **1024** samples works well
- Threshold **0.85** gives good balance of sensitivity vs false positives

---

## Robot Deploy

### Robot hardware
- i7 CPU, 8GB RAM, Ubuntu 24.04, ROS 2 Jazzy
- USB mic: AM4137
- **No GPU** — STT runs on CPU with tiny model

### Step 1 — Get the code

```bash
cd ~/ai_projects && git pull
```

### Step 2 — Python environment

```bash
cd ~/ai_projects/voice_control
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### Step 3 — Download models (first time only)

```bash
# openwakeword — downloads hey_jarvis automatically on first run
python -c "from openwakeword.model import Model; Model()"

# Piper TTS voice
mkdir -p models/piper && cd models/piper
wget "https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_GB/alan/medium/en_GB-alan-medium.onnx"
wget "https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_GB/alan/medium/en_GB-alan-medium.onnx.json"
cd ~/ai_projects

# Ollama + Mistral for LLM Q&A (install Ollama if not present)
curl -fsSL https://ollama.com/install.sh | sh
ollama pull mistral
```

### Step 4 — Config change (only 2 lines)

Edit `voice_control/config/voice_config.yaml`:

```yaml
stt:
  model: "tiny"    # was "small" — tiny is ~3x faster on CPU
  device: "cpu"    # was "cuda" — robot has no GPU
```

Everything else (tasks dir, locations, ROS2 topics, web port 8090) is identical to desktop.

### Step 5 — Run

```bash
~/ai_projects/voice_control/start.sh
```

Web UI accessible from desktop browser at `http://<robot-ip>:8090`

### Transferring from desktop (alternative to git pull)

```bash
# From the desktop machine
rsync -av --exclude='.venv' --exclude='__pycache__' --exclude='*.bin' \
  ~/ai_projects/voice_control/ robot@<robot-ip>:~/ai_projects/voice_control/
```

---

## Voice Commands

| Say | What happens |
|---|---|
| `"hey Jarvis"` | Wake word — activates listening |
| `"look right / left / up / down"` | Moves head via /head/position |
| `"go to kitchen"` | Nav2 navigation goal |
| `"run task nav3"` | Executes nav3.txt sequence |
| `"list tasks"` | Speaks available task file names |
| `"stop"` | Emergency stop + cancels running task |
| `"dock"` | Calls start_docking service |
| `"what time is it"` | Speaks current time |
| `"what's the battery"` | Reads cached battery state |
| `"what is ..."` | General Q&A via Ollama/Mistral |

Any ROS2 node can also make Robbie speak:
```bash
ros2 topic pub /voice/speak std_msgs/msg/String "data: 'hello'" --once
```

---

## Task Sequence Files

Stored in `/home/peter/ros2_ws/src/robbie/scripts/*.txt`

Format (one step per line):
```
NAV:Kitchen
TASK:MakeSound:I'm in the kitchen
TASK:Wait
TASK:Dock
```

Step types: `NAV:<location>`, `TASK:MakeSound:<text>`, `TASK:Wait`, `TASK:Dock`, `TASK:LookAt`

---

## ReSpeaker Lite Status (deferred)

The ReSpeaker Lite (Seeed, USB VID:PID 2886:0019) is stuck in DFU boot loop.
- Hardware bcdDevice=1.10, tried firmware v2.0.7 (USB) and v1.1.0 (I2S) — both boot back to DFU
- Likely firmware version mismatch with this hardware revision
- **Not blocking** — AM4137 USB mic works well as replacement
- When resolved: change `audio_source: "tcp"` in config and flash ESP32 firmware

---

## Key Files

| File | Purpose |
|---|---|
| `start.sh` | Start the voice server |
| `robbie_voice_server.py` | Main orchestrator |
| `local_mic_client.py` | USB mic capture + wake word + VAD + TTS playback |
| `stt_engine.py` | Faster-Whisper STT wrapper |
| `stop_detector.py` | Fast-path stop keyword detector |
| `intent_classifier.py` | Text → intent (regex + fuzzy location match) |
| `tts_engine.py` | Piper TTS subprocess wrapper |
| `ros2_dispatcher.py` | ROS2 publisher + /voice/speak subscriber |
| `task_runner.py` | Loads and executes .txt task sequence files |
| `llm_client.py` | Ollama client with daily conversation memory |
| `tcp_audio_client.py` | TCP server for ReSpeaker Lite (when ready) |
| `web_server.py` | FastAPI web UI with tasks side panel |
| `config/voice_config.yaml` | Main config (audio source, models, thresholds) |
| `config/intents.yaml` | Intent patterns and response templates |
| `config/locations.yaml` | Named navigation locations (x, y, yaw_deg) |
| `models/piper/en_GB-alan-medium.onnx` | British TTS voice (60MB) |

---

## Next Steps

1. **Deploy to robot** — follow steps above, only 2 config lines change
2. **Test core intents** — stop, look, goto, dock, query_status
3. **Test task runner** — "run task nav3", verify Nav2 navigation works
4. **Tune STT on CPU** — tiny model may be less accurate; adjust threshold if needed
5. **Fix ReSpeaker Lite DFU** — switch to TCP audio when resolved
6. **Arm poses** — extend `TASK:LookAt` in task_runner.py for arm control
