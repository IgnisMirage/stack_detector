#!/usr/bin/env python3
"""
Stack Detector Node

2つのcmd_velトピックを監視し、スタック状態を検出します:
- cmd_vel_command: 元の速度司令
- cmd_vel_safe: 安全機能が適用された後の速度司令

スタック条件:
- 速度司令が0より大きい（ロボットが動こうとしている）
- かつ、安全機能後のcmd_velが0（安全機能により停止）

スタック検出時:
1. スタック状態をパブリッシュ
2. リカバリーモードに入る
3. 一定時間スタックが検出されなければ、平常状態に戻る
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from enum import Enum
import math


class RobotState(Enum):
    """ロボットの状態"""
    NORMAL = 0      # 平常状態
    STUCK = 1       # スタック検出状態
    RECOVERY = 2    # リカバリーモード


class StackDetectorNode(Node):
    def __init__(self):
        super().__init__('stack_detector_node')

        # パラメータの宣言
        self.declare_parameter('cmd_vel_command_topic', '/cmd_vel_command')
        self.declare_parameter('cmd_vel_safe_topic', '/cmd_vel_safe')
        self.declare_parameter('stuck_state_topic', '/stuck_state')
        self.declare_parameter('recovery_status_topic', '/recovery_status')
        self.declare_parameter('velocity_threshold', 0.01)  # m/s または rad/s
        self.declare_parameter('stuck_timeout', 5.0)  # スタックと判定するまでの時間（秒）
        self.declare_parameter('recovery_timeout', 10.0)  # リカバリーモードの継続時間（秒）
        self.declare_parameter('normal_timeout', 3.0)  # 平常状態に戻るまでの時間（秒）
        self.declare_parameter('check_rate', 10.0)  # チェック頻度（Hz）

        # パラメータの取得
        cmd_vel_command_topic = self.get_parameter('cmd_vel_command_topic').value
        cmd_vel_safe_topic = self.get_parameter('cmd_vel_safe_topic').value
        stuck_state_topic = self.get_parameter('stuck_state_topic').value
        recovery_status_topic = self.get_parameter('recovery_status_topic').value
        self.velocity_threshold = self.get_parameter('velocity_threshold').value
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.recovery_timeout = self.get_parameter('recovery_timeout').value
        self.normal_timeout = self.get_parameter('normal_timeout').value
        check_rate = self.get_parameter('check_rate').value

        # サブスクライバー
        self.cmd_vel_command_sub = self.create_subscription(
            Twist,
            cmd_vel_command_topic,
            self.cmd_vel_command_callback,
            10
        )

        self.cmd_vel_safe_sub = self.create_subscription(
            Twist,
            cmd_vel_safe_topic,
            self.cmd_vel_safe_callback,
            10
        )

        # パブリッシャー
        self.stuck_state_pub = self.create_publisher(Bool, stuck_state_topic, 10)
        self.recovery_status_pub = self.create_publisher(String, recovery_status_topic, 10)

        # 内部状態
        self.cmd_vel_command = Twist()
        self.cmd_vel_safe = Twist()
        self.current_state = RobotState.NORMAL
        self.stuck_start_time = None
        self.recovery_start_time = None
        self.normal_start_time = None

        # タイマー
        self.timer = self.create_timer(1.0 / check_rate, self.check_stuck_condition)

        self.get_logger().info(f'Stack Detector Node started')
        self.get_logger().info(f'Monitoring: {cmd_vel_command_topic} and {cmd_vel_safe_topic}')
        self.get_logger().info(f'Publishing stuck state to: {stuck_state_topic}')
        self.get_logger().info(f'Publishing recovery status to: {recovery_status_topic}')

    def cmd_vel_command_callback(self, msg):
        """速度司令のコールバック"""
        self.cmd_vel_command = msg

    def cmd_vel_safe_callback(self, msg):
        """安全機能後の速度司令のコールバック"""
        self.cmd_vel_safe = msg

    def is_velocity_zero(self, twist_msg):
        """速度が0かどうかを判定"""
        linear_speed = math.sqrt(
            twist_msg.linear.x**2 +
            twist_msg.linear.y**2 +
            twist_msg.linear.z**2
        )
        angular_speed = math.sqrt(
            twist_msg.angular.x**2 +
            twist_msg.angular.y**2 +
            twist_msg.angular.z**2
        )
        return linear_speed < self.velocity_threshold and angular_speed < self.velocity_threshold

    def is_velocity_nonzero(self, twist_msg):
        """速度が0より大きいかを判定"""
        return not self.is_velocity_zero(twist_msg)

    def is_stuck_condition(self):
        """スタック条件を満たしているかチェック"""
        # 速度司令が0より大きく、かつ安全機能後の速度が0
        command_moving = self.is_velocity_nonzero(self.cmd_vel_command)
        safe_stopped = self.is_velocity_zero(self.cmd_vel_safe)

        return command_moving and safe_stopped

    def check_stuck_condition(self):
        """定期的にスタック状態をチェック"""
        current_time = self.get_clock().now()
        is_stuck = self.is_stuck_condition()

        # 状態マシン
        if self.current_state == RobotState.NORMAL:
            if is_stuck:
                if self.stuck_start_time is None:
                    self.stuck_start_time = current_time
                    self.get_logger().info('Potential stuck condition detected, monitoring...')
                else:
                    # スタック条件が継続している時間をチェック
                    elapsed = (current_time - self.stuck_start_time).nanoseconds / 1e9
                    if elapsed >= self.stuck_timeout:
                        self.transition_to_stuck()
            else:
                # スタック条件が解除されたらタイマーをリセット
                self.stuck_start_time = None

        elif self.current_state == RobotState.STUCK:
            # スタック状態からリカバリーモードへ自動遷移
            self.transition_to_recovery()

        elif self.current_state == RobotState.RECOVERY:
            if is_stuck:
                # リカバリー中に再度スタックが検出された場合
                self.get_logger().warn('Stuck condition detected again during recovery!')
                self.normal_start_time = None
                # リカバリータイマーをリセット
                self.recovery_start_time = current_time
            else:
                # スタック条件が解除されている
                if self.normal_start_time is None:
                    self.normal_start_time = current_time
                    self.get_logger().info('No stuck condition, starting normal state timer...')
                else:
                    # スタックしていない時間をチェック
                    elapsed_normal = (current_time - self.normal_start_time).nanoseconds / 1e9
                    if elapsed_normal >= self.normal_timeout:
                        self.transition_to_normal()

            # リカバリーモードのタイムアウトチェック
            if self.recovery_start_time is not None:
                elapsed_recovery = (current_time - self.recovery_start_time).nanoseconds / 1e9
                if elapsed_recovery >= self.recovery_timeout:
                    self.get_logger().info(f'Recovery timeout ({self.recovery_timeout}s) reached')
                    # タイムアウト後も継続的にリカバリーを試みる場合はここに処理を追加

    def transition_to_stuck(self):
        """スタック状態へ遷移"""
        self.current_state = RobotState.STUCK
        self.get_logger().warn('=== STUCK DETECTED ===')

        # スタック状態をパブリッシュ
        stuck_msg = Bool()
        stuck_msg.data = True
        self.stuck_state_pub.publish(stuck_msg)

        status_msg = String()
        status_msg.data = 'STUCK'
        self.recovery_status_pub.publish(status_msg)

        self.stuck_start_time = None

    def transition_to_recovery(self):
        """リカバリーモードへ遷移"""
        self.current_state = RobotState.RECOVERY
        self.recovery_start_time = self.get_clock().now()
        self.normal_start_time = None

        self.get_logger().info('=== ENTERING RECOVERY MODE ===')

        # リカバリー状態をパブリッシュ
        status_msg = String()
        status_msg.data = 'RECOVERY'
        self.recovery_status_pub.publish(status_msg)

        # スタック状態をfalseに設定
        stuck_msg = Bool()
        stuck_msg.data = False
        self.stuck_state_pub.publish(stuck_msg)

    def transition_to_normal(self):
        """平常状態へ遷移"""
        self.current_state = RobotState.NORMAL
        self.recovery_start_time = None
        self.normal_start_time = None

        self.get_logger().info('=== RETURNING TO NORMAL STATE ===')

        # 平常状態をパブリッシュ
        status_msg = String()
        status_msg.data = 'NORMAL'
        self.recovery_status_pub.publish(status_msg)

        # スタック状態をfalseに設定
        stuck_msg = Bool()
        stuck_msg.data = False
        self.stuck_state_pub.publish(stuck_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StackDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
