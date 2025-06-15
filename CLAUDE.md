# susumu_dummy_agi

ROS2パッケージ用のメモリファイル

## プロジェクト概要

このパッケージは、ROS2トピックのリレー機能を提供するダミーAGIパッケージです。

## ビルド・実行方法

### ビルド
```bash
# ROS2ワークスペースのルートディレクトリで実行
cd /home/taro/ros2_ws
colcon build
```

### 実行
```bash
# 環境をソース
source install/setup.bash

# リレーノードを実行
ros2 run susumu_dummy_agi relay_node

# Claude処理ノードを実行
ros2 run susumu_dummy_agi claude_processor_node

# トピック監視ツールを実行
ros2 run susumu_dummy_agi topic_monitor
```

## パッケージ構成

- `susumu_dummy_agi/relay_node.py`: メインのリレーノード実装
- `susumu_dummy_agi/claude_processor_node.py`: Claude処理ノード（非同期処理）
- `susumu_dummy_agi/topic_monitor.py`: リアルタイムトピック監視ツール
- `test/`: テストファイル群
- `package.xml`: ROS2パッケージ定義
- `setup.py`: Python セットアップファイル

## 開発メモ

- colcon buildは必ず `/home/taro/ros2_ws` で実行すること
- ビルドアーティファクト（build/, install/, log/）はgitignoreに含まれています

## Claude Code プロジェクト情報

詳細なプロジェクト情報は以下のファイルを参照:

- **`.claude/context.md`**: プロジェクト背景、技術環境、制約事項
- **`.claude/project-knowledge.md`**: 技術的知見、アーキテクチャパターン、実装詳細
- **`.claude/project-improvements.md`**: 改善履歴、学習内容、将来の改善案
- **`.claude/common-patterns.md`**: よく使うコマンド、コードテンプレート、テスト方法
- **`.claude/debug-log.md`**: 既知の問題と解決策、デバッグ情報

### 主要な技術情報

- **非同期処理**: ClaudeProcessorNodeでasyncio + ThreadPoolExecutorを使用
- **QoS設定**: 明示的なQoSプロファイル設定（RELIABLE, VOLATILE）
- **エラーハンドリング**: 包括的な例外処理とタイムアウト制御
- **ログ戦略**: 詳細なログ出力による問題追跡