#!/usr/bin/env python3

# ClaudeProcessorNodeの動作確認用シンプルテスト

import sys
import os
sys.path.append(os.path.dirname(__file__))

from susumu_dummy_agi.claude_processor_node import ClaudeProcessorNode
import rclpy
from std_msgs.msg import String
import time

def test_claude_processor():
    rclpy.init()
    
    # ノードを作成
    node = ClaudeProcessorNode()
    
    print("✅ ClaudeProcessorNode作成成功")
    
    # テストメッセージを作成
    test_msg = String()
    test_msg.data = "Hello, what is the capital of Japan?"
    
    print(f"📤 テストメッセージ: {test_msg.data}")
    
    # コールバックを直接呼び出してテスト
    try:
        node.from_human_callback(test_msg)
        print("✅ コールバック実行成功")
        
        # 少し待ってログを確認
        time.sleep(2)
        
    except Exception as e:
        print(f"❌ エラー: {e}")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    test_claude_processor()