#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

def publisher_thread():
    """メッセージを送信するスレッド"""
    time.sleep(2)  # 少し待ってから送信
    
    rclpy.init(args=None)
    node = Node('test_publisher')
    
    publisher = node.create_publisher(String, '/from_human', 10)
    
    # メッセージ作成
    msg = String()
    msg.data = "Hello Claude! What is 2+2?"
    
    print(f"📤 送信メッセージ: {msg.data}")
    
    # 少し待ってから送信
    time.sleep(1)
    publisher.publish(msg)
    print("✅ メッセージ送信完了")
    
    # 少し待ってから終了
    time.sleep(5)
    node.destroy_node()
    rclpy.shutdown()

def subscriber_thread():
    """レスポンスを受信するスレッド"""
    rclpy.init(args=None)
    node = Node('test_subscriber')
    
    def response_callback(msg):
        print(f"📥 受信レスポンス: {msg.data}")
    
    subscription = node.create_subscription(
        String,
        '/to_human',
        response_callback,
        10
    )
    
    # 未使用変数警告を回避
    _ = subscription
    
    print("🔄 レスポンス待機中...")
    
    # 10秒間待機
    try:
        rclpy.spin_until_future_complete(node, timeout_sec=10.0)
    except:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("🚀 ClaudeProcessorNode テスト開始")
    
    # パブリッシャーとサブスクライバーを別スレッドで実行
    pub_thread = threading.Thread(target=publisher_thread)
    sub_thread = threading.Thread(target=subscriber_thread)
    
    pub_thread.start()
    sub_thread.start()
    
    pub_thread.join()
    sub_thread.join()
    
    print("✅ テスト完了")