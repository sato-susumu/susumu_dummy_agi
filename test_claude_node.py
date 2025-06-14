#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class TestNode(Node):
    def __init__(self):
        super().__init__('test_claude_node')
        
        # パブリッシャーとサブスクライバーを作成
        self.publisher = self.create_publisher(String, '/from_human', 10)
        self.subscription = self.create_subscription(
            String,
            '/to_human',
            self.response_callback,
            10
        )
        
        # 未使用変数警告を回避
        _ = self.subscription
        
        self.get_logger().info('Test node started. Waiting for Claude processor...')
        
        # 少し待ってからテストメッセージを送信
        self.timer = self.create_timer(3.0, self.send_test_message)
        self.message_sent = False

    def send_test_message(self):
        if not self.message_sent:
            msg = String()
            msg.data = "Hello Claude! Can you translate 'こんにちは' to English?"
            self.publisher.publish(msg)
            self.get_logger().info(f'Sent test message: {msg.data}')
            self.message_sent = True

    def response_callback(self, msg):
        self.get_logger().info(f'Received response: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    
    test_node = TestNode()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()