#!/usr/bin/env python3
"""Test script for /voice/speak topic.

Publishes a String message to /voice/speak to trigger Robbie's TTS.

Usage:
    python3 test_speak.py
    python3 test_speak.py "Hello, I am Robbie"
    ros2 run <pkg> test_speak.py
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    text = " ".join(sys.argv[1:]) if len(sys.argv) > 1 else "Hello, I am Robbie, your robot assistant."

    rclpy.init()
    node = Node("test_speak_pub")
    pub = node.create_publisher(String, "/voice/speak", 10)

    # Wait briefly for the subscriber (robbie_voice) to connect
    import time
    time.sleep(0.5)

    msg = String()
    msg.data = text
    pub.publish(msg)
    node.get_logger().info(f"Published to /voice/speak: '{text}'")

    # Spin briefly to let the message flush
    time.sleep(0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
