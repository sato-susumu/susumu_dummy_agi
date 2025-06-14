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
```

## パッケージ構成

- `susumu_dummy_agi/relay_node.py`: メインのリレーノード実装
- `test/`: テストファイル群
- `package.xml`: ROS2パッケージ定義
- `setup.py`: Python セットアップファイル

## 開発メモ

- colcon buildは必ず `/home/taro/ros2_ws` で実行すること
- ビルドアーティファクト（build/, install/, log/）はgitignoreに含まれています