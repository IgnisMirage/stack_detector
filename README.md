# Stack Detector

2つの`cmd_vel`トピックを監視し、ロボットのスタック状態を検出するROS 2パッケージです。

## 概要

このパッケージは以下の機能を提供します:

- **デュアルcmd_vel監視**: 元の速度司令と安全機能適用後の速度司令を監視
- **スタック検出**: 速度司令が0より大きいが安全機能により停止している状態を検出
- **リカバリーモード**: スタック検出時に自動的にリカバリーモードに入る
- **自動復帰**: 一定時間スタックが検出されなければ平常状態に戻る

## 動作原理

### スタック条件

以下の両方の条件を満たす場合にスタックと判定されます:

1. **速度司令 (`cmd_vel_command`) が 0 より大きい**
   - ロボットが動こうとしている状態
2. **安全機能後の速度 (`cmd_vel_safe`) が 0**
   - 安全機能により実際には停止している状態

### 状態遷移

```
NORMAL (平常状態)
    ↓ スタック条件が一定時間継続
STUCK (スタック検出)
    ↓ 即座に遷移
RECOVERY (リカバリーモード)
    ↓ スタックが解除され、一定時間経過
NORMAL (平常状態)
```

## インストール

### 前提条件

- ROS 2 (Humble, Iron, Jazzy などをサポート)
- Python 3

### ビルド

```bash
cd ~/ros2_ws/src
git clone <this-repository-url> stack_detector
cd ~/ros2_ws
colcon build --packages-select stack_detector
source install/setup.bash
```

## 使用方法

### 基本的な起動

```bash
ros2 launch stack_detector stack_detector.launch.py
```

### カスタムトピック名で起動

```bash
ros2 launch stack_detector stack_detector.launch.py \
  cmd_vel_command_topic:=/robot/cmd_vel_original \
  cmd_vel_safe_topic:=/robot/cmd_vel_filtered
```

### パラメータのカスタマイズ

```bash
ros2 launch stack_detector stack_detector.launch.py \
  stuck_timeout:=3.0 \
  recovery_timeout:=15.0 \
  normal_timeout:=5.0
```

### ノードを直接実行

```bash
ros2 run stack_detector stack_detector_node
```

## パラメータ

| パラメータ名 | デフォルト値 | 説明 |
|-------------|------------|------|
| `cmd_vel_command_topic` | `/cmd_vel_command` | 元の速度司令トピック |
| `cmd_vel_safe_topic` | `/cmd_vel_safe` | 安全機能適用後の速度司令トピック |
| `stuck_state_topic` | `/stuck_state` | スタック状態を出力するトピック (Bool) |
| `recovery_status_topic` | `/recovery_status` | リカバリー状態を出力するトピック (String) |
| `velocity_threshold` | `0.01` | 速度が0と見なす閾値 (m/s または rad/s) |
| `stuck_timeout` | `5.0` | スタックと判定するまでの時間（秒） |
| `recovery_timeout` | `10.0` | リカバリーモードの継続時間（秒） |
| `normal_timeout` | `3.0` | 平常状態に戻るまでの時間（秒） |
| `check_rate` | `10.0` | チェック頻度（Hz） |

## トピック

### サブスクライブ

- `cmd_vel_command` (`geometry_msgs/Twist`): 元の速度司令
- `cmd_vel_safe` (`geometry_msgs/Twist`): 安全機能適用後の速度司令

### パブリッシュ

- `stuck_state` (`std_msgs/Bool`): スタック状態 (True: スタック検出時)
- `recovery_status` (`std_msgs/String`): 現在の状態 ("NORMAL", "STUCK", "RECOVERY")

## 使用例

### スタック状態の監視

```bash
# 別のターミナルでスタック状態を監視
ros2 topic echo /stuck_state

# リカバリー状態を監視
ros2 topic echo /recovery_status
```

### テスト用のcmd_velパブリッシュ

```bash
# 速度司令をパブリッシュ（動こうとしている）
ros2 topic pub /cmd_vel_command geometry_msgs/Twist "{linear: {x: 0.5}}" -r 10

# 安全機能後の速度（停止）
ros2 topic pub /cmd_vel_safe geometry_msgs/Twist "{linear: {x: 0.0}}" -r 10
```

上記のコマンドを実行すると、数秒後にスタックが検出され、リカバリーモードに入ります。

## アーキテクチャ

```
┌─────────────────────┐
│  Velocity Command   │
│   (cmd_vel_command) │
└──────────┬──────────┘
           │
           ├──────────┐
           │          │
           ▼          ▼
    ┌──────────────────────┐
    │  Stack Detector Node │
    └──────────┬───────────┘
               │
               ▼
        ┌─────────────┐
        │   Safety    │
        │   Filter    │
        └──────┬──────┘
               │
               ▼
     ┌──────────────────┐
     │  cmd_vel_safe    │
     └──────────────────┘

Output Topics:
  - /stuck_state (Bool)
  - /recovery_status (String)
```

## リカバリーロジックの統合

このノードはスタック状態を検出してパブリッシュします。実際のリカバリー動作（後退、回転など）を実装する場合は、`/stuck_state`または`/recovery_status`トピックをサブスクライブする別のノードを作成してください。

### リカバリーノードの例

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class RecoveryNode(Node):
    def __init__(self):
        super().__init__('recovery_node')
        self.status_sub = self.create_subscription(
            String, '/recovery_status', self.status_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def status_callback(self, msg):
        if msg.data == 'RECOVERY':
            # リカバリー動作を実行（例: 後退）
            twist = Twist()
            twist.linear.x = -0.2  # 後退
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('Executing recovery maneuver')
```

## ライセンス

Apache License 2.0

## 貢献

プルリクエストを歓迎します。大きな変更の場合は、まずissueを開いて変更内容を議論してください。

## トラブルシューティング

### ノードが起動しない

- ROS 2が正しくインストールされているか確認
- `source install/setup.bash`を実行したか確認

### スタックが検出されない

- トピック名が正しいか確認: `ros2 topic list`
- cmd_velメッセージがパブリッシュされているか確認: `ros2 topic echo /cmd_vel_command`
- `velocity_threshold`パラメータを調整

### 誤検出が多い

- `stuck_timeout`を増やして、一時的な停止を無視
- `velocity_threshold`を調整

## 開発者向け情報

### テストの実行

```bash
cd ~/ros2_ws
colcon test --packages-select stack_detector
```

### コードフォーマット

```bash
black stack_detector/
flake8 stack_detector/
```

## 今後の改善案

- [ ] スタック状態の履歴記録
- [ ] 自動リカバリー動作の実装
- [ ] 診断トピック (`diagnostic_msgs`) のサポート
- [ ] パラメータの動的再設定
- [ ] ビジュアライゼーションツール (RViz2 plugin)
