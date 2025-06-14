#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RelayNode(Node):
    def __init__(self):
        super().__init__('relay_node')
        
        self.subscription = self.create_subscription(
            String,
            '/from_human',
            self.from_human_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            String,
            '/to_human',
            10
        )
        
        self.get_logger().info('Relay node started: /from_human -> /to_human')

    def from_human_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        
        relay_msg = String()
        relay_msg.data = msg.data
        
        self.publisher.publish(relay_msg)
        self.get_logger().info(f'Relayed: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    
    relay_node = RelayNode()
    
    try:
        rclpy.spin(relay_node)
    except KeyboardInterrupt:
        pass
    
    relay_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()