#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
import asyncio
from datetime import datetime
from collections import deque
from typing import Dict, List, Optional
import threading
import time
from rich.console import Console
from rich.table import Table
from rich.live import Live
from rich.panel import Panel
from rich.columns import Columns
from rich.text import Text


class TopicMonitor(Node):
    def __init__(self):
        super().__init__('topic_monitor')
        
        # QoS設定
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # メッセージ履歴とメトリクス
        self.from_human_history = deque(maxlen=50)
        self.to_human_history = deque(maxlen=50)
        self.from_human_count = 0
        self.to_human_count = 0
        self.from_human_last_time = None
        self.to_human_last_time = None
        
        # サブスクライバー作成
        self.from_human_sub = self.create_subscription(
            String,
            '/from_human',
            self.from_human_callback,
            qos_profile
        )
        
        self.to_human_sub = self.create_subscription(
            String,
            '/to_human',
            self.to_human_callback,
            qos_profile
        )
        
        self.get_logger().info('Topic monitor started')
    
    def from_human_callback(self, msg: String):
        timestamp = datetime.now()
        self.from_human_history.append({
            'data': msg.data,
            'timestamp': timestamp
        })
        self.from_human_count += 1
        self.from_human_last_time = timestamp
    
    def to_human_callback(self, msg: String):
        timestamp = datetime.now()
        self.to_human_history.append({
            'data': msg.data,
            'timestamp': timestamp
        })
        self.to_human_count += 1
        self.to_human_last_time = timestamp


class MonitorCLI:
    def __init__(self, monitor_node: TopicMonitor):
        self.monitor = monitor_node
        self.console = Console()
        self.running = True
    
    def create_statistics_table(self) -> Table:
        table = Table(title="Topic Statistics")
        table.add_column("Topic", style="cyan")
        table.add_column("Count", style="green")
        table.add_column("Last Message", style="yellow")
        
        from_human_last = self.monitor.from_human_last_time.strftime("%H:%M:%S") if self.monitor.from_human_last_time else "Never"
        to_human_last = self.monitor.to_human_last_time.strftime("%H:%M:%S") if self.monitor.to_human_last_time else "Never"
        
        table.add_row("/from_human", str(self.monitor.from_human_count), from_human_last)
        table.add_row("/to_human", str(self.monitor.to_human_count), to_human_last)
        
        return table
    
    def create_history_panel(self, title: str, history: deque, color: str) -> Panel:
        if not history:
            content = Text("No messages yet", style="dim")
        else:
            lines = []
            for item in list(history)[-10:]:  # 最新10件表示
                timestamp = item['timestamp'].strftime("%H:%M:%S")
                lines.append(f"[{timestamp}] {item['data']}")
            content = "\n".join(lines)
        
        return Panel(content, title=title, border_style=color)
    
    def generate_display(self) -> Table:
        main_table = Table.grid()
        main_table.add_column()
        
        # 統計情報
        stats_table = self.create_statistics_table()
        main_table.add_row(stats_table)
        main_table.add_row("")
        
        # メッセージ履歴
        from_human_panel = self.create_history_panel(
            "/from_human Messages", 
            self.monitor.from_human_history, 
            "blue"
        )
        
        to_human_panel = self.create_history_panel(
            "/to_human Messages", 
            self.monitor.to_human_history, 
            "green"
        )
        
        columns = Columns([from_human_panel, to_human_panel])
        main_table.add_row(columns)
        
        return main_table
    
    def run(self):
        try:
            with Live(self.generate_display(), refresh_per_second=2) as live:
                while self.running:
                    live.update(self.generate_display())
                    time.sleep(0.5)
        except KeyboardInterrupt:
            self.running = False
            self.console.print("\n[yellow]Monitor stopped[/yellow]")


def ros_spin_thread(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor_node = TopicMonitor()
        
        # ROS2ノードを別スレッドで実行
        ros_thread = threading.Thread(target=ros_spin_thread, args=(monitor_node,))
        ros_thread.daemon = True
        ros_thread.start()
        
        # CLIを実行
        cli = MonitorCLI(monitor_node)
        cli.run()
        
    except KeyboardInterrupt:
        pass
    finally:
        monitor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()