#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import speech_recognition as sr
from std_msgs.msg import String

"""
This implementation was inspired by:
https://learnbydoing.dev/how-to-build-voice-interaction-model-in-ros-2/

Original tutorial © 2025 All rights reserved.
This file has been modified and adapted for this project.
"""


class VoiceInteractionNode(Node):
    def __init__(self):
        super().__init__('voice_interaction_node')
        self.publisher_ = self.create_publisher(String, 'voice_commands', 10)
        self.timer = self.create_timer(1.0, self.listen_command)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.get_logger().info("Voice Interaction Node started and listening...")

    def listen_command(self):
        with self.microphone as source:
            self.get_logger().info("Listening for a command...")
            audio = self.recognizer.listen(source)
        try:
            command = self.recognizer.recognize_google(
                audio,  language="pl-PL")
            self.get_logger().info(f"Recognized command: {command}")
            msg = String()
            msg.data = command
            self.publisher_.publish(msg)
        except sr.UnknownValueError:
            self.get_logger().warn("Could not understand the audio.")
        except sr.RequestError as e:
            self.get_logger().error(f"Speech Recognition service error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = VoiceInteractionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
