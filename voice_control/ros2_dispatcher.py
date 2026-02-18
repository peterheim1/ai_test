"""ROS 2 topic publisher for voice command dispatch."""

import json
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty, Float64MultiArray
from geometry_msgs.msg import Twist

from voice_control.intent_classifier import Intent


class ROS2Dispatcher:
    """Publishes voice intents and commands to ROS 2 topics.

    Runs rclpy in a background daemon thread. Publisher methods are
    thread-safe and can be called from asyncio.
    """

    def __init__(self, node_name: str = "robbie_voice", topic_config: dict | None = None):
        topics = topic_config or {}
        self._topic_names = {
            "intent": topics.get("intent", "/voice/intent"),
            "stop": topics.get("stop", "/voice/stop"),
            "tts_text": topics.get("tts_text", "/voice/tts_text"),
            "cmd_vel": topics.get("cmd_vel", "/cmd_vel"),
            "drive": topics.get("drive", "/drive"),
            "head_position": topics.get("head_position", "/head/position"),
        }

        rclpy.init()
        self._node = Node(node_name)

        # Publishers
        self._pub_intent = self._node.create_publisher(
            String, self._topic_names["intent"], 10)
        self._pub_stop = self._node.create_publisher(
            Empty, self._topic_names["stop"], 10)
        self._pub_tts_text = self._node.create_publisher(
            String, self._topic_names["tts_text"], 10)
        self._pub_cmd_vel = self._node.create_publisher(
            Twist, self._topic_names["cmd_vel"], 10)
        self._pub_drive = self._node.create_publisher(
            Float64MultiArray, self._topic_names["drive"], 10)
        self._pub_head = self._node.create_publisher(
            Float64MultiArray, self._topic_names["head_position"], 10)

        # Cached state from subscribers
        self._battery_percentage: float | None = None
        self._is_docked: bool | None = None

        # Spin in background thread
        self._spin_thread = threading.Thread(
            target=self._spin, daemon=True, name="ros2_spin")
        self._spin_thread.start()

    def _spin(self):
        try:
            rclpy.spin(self._node)
        except Exception:
            pass

    def publish_intent(self, intent: Intent):
        """Publish a JSON-encoded intent to /voice/intent and action-specific topics."""
        # JSON intent
        msg = String()
        msg.data = json.dumps({
            "timestamp": datetime.now().isoformat(),
            "utterance": intent.utterance,
            "intent": intent.name,
            "confidence": intent.confidence,
            "params": intent.params,
        })
        self._pub_intent.publish(msg)

        # Action-specific publishing
        if intent.name == "look":
            self.publish_head(
                intent.params.get("pan", 0.0),
                intent.params.get("tilt", 0.0),
            )
        elif intent.name == "stop_all":
            self.publish_stop()

        # Publish TTS text if available
        if intent.response_text:
            tts_msg = String()
            tts_msg.data = intent.response_text
            self._pub_tts_text.publish(tts_msg)

    def publish_stop(self):
        """Fast-path emergency stop: publish to multiple topics simultaneously."""
        # /voice/stop
        self._pub_stop.publish(Empty())

        # /cmd_vel - all zeros
        twist = Twist()
        self._pub_cmd_vel.publish(twist)

        # /drive - all zeros [FL, RL, RR, FR]
        drive_msg = Float64MultiArray()
        drive_msg.data = [0.0, 0.0, 0.0, 0.0]
        self._pub_drive.publish(drive_msg)

    def publish_head(self, pan: float, tilt: float):
        """Publish head position [pan, tilt] to /head/position."""
        msg = Float64MultiArray()
        msg.data = [pan, tilt]
        self._pub_head.publish(msg)

    def get_battery_percentage(self) -> float | None:
        """Return cached battery percentage, or None if unknown."""
        return self._battery_percentage

    def get_dock_state(self) -> bool | None:
        """Return cached dock state, or None if unknown."""
        return self._is_docked

    def get_published_topics(self) -> list[str]:
        """Return list of topic names this dispatcher publishes to."""
        return list(self._topic_names.values())

    def shutdown(self):
        """Shutdown rclpy and stop the spin thread."""
        try:
            self._node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
