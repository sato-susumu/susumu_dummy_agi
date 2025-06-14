#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String


class RelayNode(Node):
    def __init__(self) -> None:
        super().__init__('relay_node')
        
        # QoS設定を明示的に定義
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.subscription = self.create_subscription(
            String,
            '/from_human',
            self.from_human_callback,
            qos_profile
        )
        
        self.publisher = self.create_publisher(
            String,
            '/to_human',
            qos_profile
        )
        
        # subscriptionを参照して未使用変数警告を回避  
        _ = self.subscription
        
        self.get_logger().info('Relay node started: /from_human -> /to_human')

    def from_human_callback(self, msg: String) -> None:
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