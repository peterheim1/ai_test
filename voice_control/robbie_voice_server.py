#!/usr/bin/env python3
"""
Robbie Voice Server - Main entry point.

Receives audio from a ReSpeaker Lite (XIAO ESP32-S3) via TCP,
detects wake words with openwakeword, transcribes with Faster-Whisper,
classifies intent, dispatches to ROS 2, and responds via Piper TTS.

Usage:
    python3 -m voice_control.robbie_voice_server
    python3 voice_control/robbie_voice_server.py
    python3 voice_control/robbie_voice_server.py --config voice_control/config/voice_config.yaml
"""

import argparse
import asyncio
import logging
import signal
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import yaml

from voice_control.console_logger import ConsoleLogger
from voice_control.tcp_audio_client import TCPAudioClient, LED_IDLE, LED_WAKE, LED_THINKING
from voice_control.intent_classifier import IntentClassifier
from voice_control.llm_client import LLMClient
from voice_control.ros2_dispatcher import ROS2Dispatcher
from voice_control.stop_detector import StopDetector
from voice_control.stt_engine import STTEngine
from voice_control.tts_engine import TTSEngine

logging.basicConfig(level=logging.DEBUG, format="%(name)s: %(message)s")
logger = logging.getLogger(__name__)

# Bytes of audio for ~0.5s at 16kHz 16-bit mono
STOP_CHECK_BYTES = 16000


class RobbieVoiceServer:
    """Main voice control server orchestrating the full pipeline."""

    def __init__(self, config_path: str):
        self.config = self._load_config(config_path)
        config_dir = Path(config_path).parent

        self.console = ConsoleLogger()
        self._executor = ThreadPoolExecutor(max_workers=3)
        self._running = False

        # Components (initialized in startup)
        self._esphome: TCPAudioClient | None = None
        self._stt: STTEngine | None = None
        self._stop_detector: StopDetector | None = None
        self._tts: TTSEngine | None = None
        self._llm: LLMClient | None = None
        self._classifier: IntentClassifier | None = None
        self._dispatcher: ROS2Dispatcher | None = None

        # Config file paths
        self._intents_path = str(config_dir / "intents.yaml")
        self._locations_path = str(config_dir / "locations.yaml")
        self._head_path = str(config_dir / "head_positions.yaml")

    def _load_config(self, path: str) -> dict:
        with open(path) as f:
            return yaml.safe_load(f)

    async def startup(self):
        """Initialize all components in order."""
        self.console.log_info("Starting Robbie Voice Server...")

        # 1. ROS 2 dispatcher (background thread)
        self.console.log_info("Initializing ROS 2...")
        ros2_cfg = self.config.get("ros2", {})
        self._dispatcher = ROS2Dispatcher(
            node_name=ros2_cfg.get("node_name", "robbie_voice"),
            topic_config=ros2_cfg.get("topics", {}),
        )

        # 2. STT engines (CUDA)
        stt_cfg = self.config.get("stt", {})
        device = stt_cfg.get("device", "cuda")

        self.console.log_info(f"Loading Faster-Whisper ({stt_cfg.get('model', 'small')}) on {device}...")
        self._stt = await asyncio.get_event_loop().run_in_executor(
            self._executor,
            lambda: STTEngine(
                model_size=stt_cfg.get("model", "small"),
                device=device,
                language=stt_cfg.get("language", "en"),
            ),
        )

        # 3. Stop detector (tiny model)
        stop_cfg = self.config.get("stop_detector", {})
        self.console.log_info("Loading stop detector (tiny model)...")
        self._stop_detector = await asyncio.get_event_loop().run_in_executor(
            self._executor,
            lambda: StopDetector(
                keywords=stop_cfg.get("keywords"),
                device=device,
            ),
        )

        # 4. TTS
        tts_cfg = self.config.get("tts", {})
        self.console.log_info(f"Initializing Piper TTS ({tts_cfg.get('model', 'en_US-lessac-medium')})...")
        self._tts = TTSEngine(
            model=tts_cfg.get("model", "en_US-lessac-medium"),
            output_sample_rate=tts_cfg.get("output_sample_rate", 22050),
        )

        # 5. LLM
        llm_cfg = self.config.get("llm", {})
        self._llm = LLMClient(
            model=llm_cfg.get("model", "mistral"),
            host=llm_cfg.get("host", "http://localhost:11434"),
            system_prompt=llm_cfg.get("system_prompt", ""),
            max_tokens=llm_cfg.get("max_tokens", 100),
        )
        if self._llm.health_check():
            self.console.log_info(f"Ollama connected ({llm_cfg.get('model', 'mistral')})")
        else:
            self.console.log_error(
                f"Ollama not available - general questions will fail"
            )

        # 6. Intent classifier
        self._classifier = IntentClassifier(
            intents_path=self._intents_path,
            locations_path=self._locations_path,
            head_positions_path=self._head_path,
        )
        self.console.log_info("Intent classifier loaded")

        # 7. TCP audio connection (replaces ESPHome)
        tcp_cfg = self.config.get("tcp_audio", {})
        self._esphome = TCPAudioClient(
            listen_host=tcp_cfg.get("listen_host", "0.0.0.0"),
            listen_port=tcp_cfg.get("listen_port", 8765),
            wake_word_model=tcp_cfg.get("wake_word_model", "hey_jarvis"),
            wake_word_threshold=tcp_cfg.get("wake_word_threshold", 0.5),
        )
        self._esphome.on_wake_word = self._on_wake_word

        self.console.log_info(f"Starting TCP audio server on {tcp_cfg.get('listen_host', '0.0.0.0')}:{tcp_cfg.get('listen_port', 8765)}...")
        await self._esphome.connect()

        self.console.log_info("Robbie Voice Server ready")
        self.console.log_end()

    def _on_wake_word(self):
        """Called when ESP32 detects the wake word."""
        self.console.log_wake()
        loop = asyncio.get_event_loop()
        # Send LED wake (magenta) + chime trigger
        loop.create_task(self._esphome.send_led_state(LED_WAKE))
        # Schedule the pipeline handling
        loop.create_task(self._handle_pipeline())

    async def _handle_pipeline(self):
        """Handle a complete voice pipeline: audio → STT → intent → dispatch → TTS."""
        loop = asyncio.get_event_loop()
        vad_cfg = self.config.get("vad", {})
        max_duration = vad_cfg.get("max_duration_ms", 5000) / 1000.0
        stop_check_secs = self.config.get("stop_detector", {}).get("max_audio_seconds", 0.5)

        # Wait briefly for initial audio to arrive for stop check
        await asyncio.sleep(stop_check_secs + 0.1)

        # Fast-path stop check on partial audio
        partial_audio = self._esphome.get_partial_audio(STOP_CHECK_BYTES)
        if partial_audio and len(partial_audio) >= STOP_CHECK_BYTES // 2:
            is_stop = await loop.run_in_executor(
                self._executor,
                self._stop_detector.check,
                partial_audio,
            )
            if is_stop:
                self.console.log_fast_stop()
                self._dispatcher.publish_stop()
                self.console.log_publish(["/voice/stop", "/cmd_vel", "/drive"])
                # Send short TTS response
                await self._speak("stopping")
                await self._esphome.send_led_state(LED_IDLE)
                self.console.log_end()
                return

        # Wait for full audio
        audio = await self._esphome.wait_for_audio(timeout=max_duration)

        if not audio or len(audio) < 1600:  # less than 0.05s of audio
            self.console.log_error("No audio received")
            await self._esphome.send_led_state(LED_IDLE)
            self.console.log_end()
            return

        # LED → thinking (yellow) before STT
        await self._esphome.send_led_state(LED_THINKING)

        # Full STT transcription
        text = await loop.run_in_executor(
            self._executor, self._stt.transcribe, audio
        )
        self.console.log_utterance(text)

        if not text:
            await self._speak("I didn't catch that")
            await self._esphome.send_led_state(LED_IDLE)
            self.console.log_end()
            return

        # Classify intent
        intent = self._classifier.classify(text)
        self.console.log_intent(intent.name, intent.params)

        # Dispatch to ROS 2
        self._dispatcher.publish_intent(intent)
        published_topics = [self._dispatcher._topic_names["intent"]]
        if intent.name == "look":
            published_topics.append(self._dispatcher._topic_names["head_position"])
        elif intent.name == "stop_all":
            published_topics.extend([
                self._dispatcher._topic_names["stop"],
                self._dispatcher._topic_names["cmd_vel"],
                self._dispatcher._topic_names["drive"],
            ])
        self.console.log_publish(published_topics)

        # Generate response
        response_text = await self._generate_response(intent)

        # Speak response
        if response_text:
            await self._speak(response_text)

        await self._esphome.send_led_state(LED_IDLE)
        self.console.log_end()

    async def _generate_response(self, intent) -> str:
        """Generate the text response for an intent."""
        loop = asyncio.get_event_loop()

        if intent.name == "general_question":
            question = intent.params.get("original", intent.utterance)
            self.console.log_info("Querying LLM...")
            answer = await loop.run_in_executor(
                self._executor, self._llm.ask, question
            )
            self.console.log_llm(answer)
            return answer

        if intent.name == "query_status":
            return self._build_status_response(intent.params.get("subject", "general"))

        if intent.name == "list_topics":
            return "Here are the active topics: " + ", ".join(
                self._dispatcher.get_published_topics()
            )

        return intent.response_text

    def _build_status_response(self, subject: str) -> str:
        """Build a status response based on cached ROS 2 data."""
        if subject == "battery":
            pct = self._dispatcher.get_battery_percentage()
            if pct is not None:
                return f"battery is at {pct:.0f} percent"
            return "I don't have battery information right now"
        elif subject == "dock":
            state = self._dispatcher.get_dock_state()
            if state is not None:
                return "I am docked" if state else "I am not docked"
            return "I don't have docking information right now"
        elif subject == "errors":
            return "no errors reported"
        elif subject == "general":
            return "I'm doing well, all systems are running"
        return f"I don't have {subject} information right now"

    async def _speak(self, text: str):
        """Synthesize and send TTS audio to ESP32."""
        loop = asyncio.get_event_loop()
        self.console.log_response(text)
        try:
            audio = await loop.run_in_executor(
                self._executor, self._tts.synthesize, text
            )
            if audio and self._esphome:
                await self._esphome.send_tts_audio(audio)
        except Exception as e:
            self.console.log_error(f"TTS failed: {e}")

    async def run(self):
        """Main run loop."""
        self._running = True
        await self.startup()

        # Keep running until interrupted
        stop_event = asyncio.Event()

        def _signal_handler():
            stop_event.set()

        loop = asyncio.get_event_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, _signal_handler)

        await stop_event.wait()
        await self.shutdown()

    async def shutdown(self):
        """Gracefully shut down all components."""
        self._running = False
        self.console.log_info("Shutting down...")
        if self._esphome:
            await self._esphome.disconnect()
        if self._dispatcher:
            self._dispatcher.shutdown()
        self._executor.shutdown(wait=False)
        self.console.log_info("Robbie Voice Server stopped")


def main():
    parser = argparse.ArgumentParser(description="Robbie Voice Control Server")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).parent / "config" / "voice_config.yaml"),
        help="Path to voice_config.yaml",
    )
    args = parser.parse_args()

    server = RobbieVoiceServer(args.config)
    asyncio.run(server.run())


if __name__ == "__main__":
    main()
