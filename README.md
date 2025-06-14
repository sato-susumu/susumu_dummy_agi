# susumu_dummy_agi

ROS2 ダミーAGIパッケージ - トピックリレー機能

## 概要

このパッケージは、トピックリレー機能を実演するシンプルなROS2ノードを提供します。より複雑なAGI関連ROS2アプリケーションのテンプレートとして使用できます。

## 機能

- トピックの購読と発行
- 設定可能なトピックでのメッセージリレー
- ROS2ノードライフサイクル管理

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

### リレーノードの実行

```bash
ros2 run susumu_dummy_agi relay_node
```

### トピック

- **購読トピック**: `/input_topic` (std_msgs/String)
- **発行トピック**: `/output_topic` (std_msgs/String)

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