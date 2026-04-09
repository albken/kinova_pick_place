#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CommandInterpreter(Node):
    def __init__(self):
        super().__init__('command_interpreter')
        self.subscription = self.create_subscription(
            String,
            'voice_commands',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'cube_color', 10)
        self.get_logger().info("Command Interpreter Node started.")

    def listener_callback(self, msg):
        command = msg.data.lower()
        cube_color = String()

        if ("niebieski" in command) or ("niebieskie" in command) or ("niebieska" in command):
            cube_color.data = "blue"
        elif ("czerwony" in command) or ("czerwone" in command) or ("czerwona" in command):
            cube_color.data = "red"
        else:
            self.get_logger().info(f"Unknow: {command}")
            return

        self.publisher_.publish(cube_color)
        self.get_logger().info(f"Executing command: {command}")


def main(args=None):
    rclpy.init(args=args)
    node = CommandInterpreter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
