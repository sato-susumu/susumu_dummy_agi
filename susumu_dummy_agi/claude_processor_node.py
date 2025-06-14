#!/usr/bin/env python3

import asyncio
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from concurrent.futures import ThreadPoolExecutor
import threading
from typing import Optional


class ClaudeProcessorNode(Node):
    def __init__(self) -> None:
        super().__init__('claude_processor_node')
        
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
        
        # 未使用変数警告を回避
        _ = self.subscription
        
        # 非同期処理用のイベントループとスレッドプール
        self.loop = None
        self.thread_pool = ThreadPoolExecutor(max_workers=3)
        self.processing_count = 0
        self.max_concurrent_tasks = 2
        
        # 別スレッドでasyncioイベントループを開始
        self.loop_thread = threading.Thread(target=self._start_event_loop, daemon=True)
        self.loop_thread.start()
        
        self.get_logger().info('Claude Processor node started: /from_human -> claude -> /to_human')

    def _start_event_loop(self) -> None:
        """別スレッドでasyncioイベントループを実行"""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def from_human_callback(self, msg: String) -> None:
        """メッセージ受信時のコールバック - 即座にレスポンスし非同期処理開始"""
        self.get_logger().info(f'Received: {msg.data[:50]}...')
        
        # 同時実行数チェック
        if self.processing_count >= self.max_concurrent_tasks:
            error_msg = "処理中のタスクが多すぎます。しばらくお待ちください。"
            self._publish_message(error_msg)
            return
        
        # 即座に処理開始メッセージを送信
        start_msg = f"処理中です"
        self._publish_message(start_msg)
        
        # 非同期でClaude処理を実行
        if self.loop and not self.loop.is_closed():
            asyncio.run_coroutine_threadsafe(
                self._process_with_claude_async(msg.data), 
                self.loop
            )

    async def _process_with_claude_async(self, input_text: str) -> None:
        """Claude処理を非同期で実行"""
        self.processing_count += 1
        
        try:
            # Claude処理実行
            result = await self._execute_claude_command(input_text)
            
            # 結果をパブリッシュ
            result_msg = f"{result}"
            self._publish_message(result_msg)
            
        except Exception as e:
            error_msg = f"❌ エラー: {str(e)}"
            self._publish_message(error_msg)
            self.get_logger().error(f'Claude processing error: {e}')
            
        finally:
            self.processing_count -= 1

    async def _execute_claude_command(self, input_text: str) -> str:
        """Claudeコマンドを非同期で実行"""
        self.get_logger().info(f'Claude実行開始: "{input_text[:100]}{"..." if len(input_text) > 100 else ""}"')
        
        try:
            # subprocessを非同期で実行
            process = await asyncio.create_subprocess_exec(
                'claude', input_text,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            
            # 60秒でタイムアウト
            stdout, stderr = await asyncio.wait_for(
                process.communicate(), 
                timeout=60.0
            )
            
            if process.returncode == 0:
                response = stdout.decode('utf-8').strip()
                self.get_logger().info(f'Claude実行成功: レスポンス長={len(response)}文字')
                self.get_logger().info(f'Claudeレスポンス: "{response[:200]}{"..." if len(response) > 200 else ""}"')
                return response
            else:
                error_output = stderr.decode('utf-8').strip()
                self.get_logger().error(f'Claude実行失敗: return_code={process.returncode}, stderr="{error_output}"')
                raise Exception(f'Claude command failed: {error_output}')
                
        except asyncio.TimeoutError:
            self.get_logger().error('Claude処理がタイムアウトしました（60秒）')
            raise Exception('Claude処理がタイムアウトしました（60秒）')
        except FileNotFoundError:
            self.get_logger().error('claudeコマンドが見つかりません')
            raise Exception('claudeコマンドが見つかりません')
        except Exception as e:
            self.get_logger().error(f'Claude実行中に予期しないエラー: {str(e)}')
            raise Exception(f'Claude実行エラー: {str(e)}')

    def _publish_message(self, message: str) -> None:
        """メッセージをパブリッシュ"""
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {message[:50]}...')

    def destroy_node(self) -> None:
        """ノード終了時のクリーンアップ"""
        if self.loop and not self.loop.is_closed():
            self.loop.call_soon_threadsafe(self.loop.stop)
        
        self.thread_pool.shutdown(wait=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    claude_processor_node = ClaudeProcessorNode()
    
    try:
        rclpy.spin(claude_processor_node)
    except KeyboardInterrupt:
        pass
    finally:
        claude_processor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()