# susumu_dummy_agi

ROS2 ダミーAGIパッケージ - トピックリレー機能

## 概要

このパッケージは、ROS2トピックを使用したメッセージリレーとClaude AIによる処理機能を提供します。人間からのメッセージを受け取り、Claude AIで処理して結果を返すAGIシステムのプロトタイプです。

## 機能

### RelayNode（基本リレー）
- シンプルなトピックリレー機能
- `/from_human` → `/to_human` への直接転送

### ClaudeProcessorNode（AI処理）
- Claude AIを使用したメッセージ処理
- 非同期処理による高速レスポンス
- バックグラウンド処理とリアルタイム進捗通知
- エラーハンドリングとタイムアウト制御

## 依存関係

- ROS2 (Humbleでテスト済み)
- Python 3.8+
- rclpy

## インストール

1. ROS2ワークスペースにこのリポジトリをクローン:
```bash
cd ~/ros2_ws/src
git clone <repository-url> susumu_dummy_agi
```

2. パッケージをビルド:
```bash
cd ~/ros2_ws
colcon build --packages-select susumu_dummy_agi
```

3. ワークスペースをソース:
```bash
source install/setup.bash
```

## 使用方法

### 基本リレーノードの実行

```bash
ros2 run susumu_dummy_agi relay_node
```

### Claude処理ノードの実行 (動作未確認)

```bash
ros2 run susumu_dummy_agi claude_processor_node
```

### トピック

両ノード共通:
- **購読トピック**: `/from_human` (std_msgs/String)
- **発行トピック**: `/to_human` (std_msgs/String)

### テスト方法

別ターミナルでメッセージを送信:
```bash
# メッセージ送信
ros2 topic pub /from_human std_msgs/String "data: 'こんにちは、Claudeで翻訳してください'"

# メッセージ受信確認
ros2 topic echo /to_human
```

### パラメータ

ノードは以下のパラメータを受け付けます:
- `input_topic` (文字列, デフォルト: "/input_topic"): 入力トピック名
- `output_topic` (文字列, デフォルト: "/output_topic"): 出力トピック名

カスタムパラメータの例:
```bash
ros2 run susumu_dummy_agi relay_node --ros-args -p input_topic:=/custom_input -p output_topic:=/custom_output
```

## テスト

テストの実行:
```bash
cd ~/ros2_ws
colcon test --packages-select susumu_dummy_agi
```

## ライセンス

このプロジェクトはMITライセンスの下でライセンスされています。詳細は[LICENSE](LICENSE)ファイルを参照してください。

## 貢献

1. リポジトリをフォーク
2. 機能ブランチを作成
3. 変更を加える
4. テストを実行
5. プルリクエストを提出