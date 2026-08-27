#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess


class BashExecutor(Node):

    def __init__(self):
        super().__init__('bash_executor')

        self.subscription = self.create_subscription(
            String,
            '/baton_cmd',
            self.callback,
            10
        )

        self.get_logger().info('bash_executor started')

    def callback(self, msg):
        command = msg.data

        self.get_logger().info(f'Received: {command}')

        try:
            if command == "finish":
                subprocess.Popen(
                    ["bash", "/home/colcon_ws/src/g1_navigation/bash/baton.sh"]
                )

            else:
                self.get_logger().warn('Unknown command')

        except Exception as e:
            self.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=args)

    node = BashExecutor()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()