#!/usr/bin/env python3
"""
Stack Detector Launch File

2つのcmd_velトピックを監視するstack_detectorノードを起動します。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch引数の宣言
    cmd_vel_command_topic_arg = DeclareLaunchArgument(
        'cmd_vel_command_topic',
        default_value='/cmd_vel_command',
        description='元の速度司令トピック'
    )

    cmd_vel_safe_topic_arg = DeclareLaunchArgument(
        'cmd_vel_safe_topic',
        default_value='/cmd_vel_safe',
        description='安全機能が適用された後の速度司令トピック'
    )

    stuck_state_topic_arg = DeclareLaunchArgument(
        'stuck_state_topic',
        default_value='/stuck_state',
        description='スタック状態をパブリッシュするトピック'
    )

    recovery_status_topic_arg = DeclareLaunchArgument(
        'recovery_status_topic',
        default_value='/recovery_status',
        description='リカバリー状態をパブリッシュするトピック'
    )

    velocity_threshold_arg = DeclareLaunchArgument(
        'velocity_threshold',
        default_value='0.01',
        description='速度が0と見なす閾値 (m/s または rad/s)'
    )

    stuck_timeout_arg = DeclareLaunchArgument(
        'stuck_timeout',
        default_value='5.0',
        description='スタックと判定するまでの時間（秒）'
    )

    recovery_timeout_arg = DeclareLaunchArgument(
        'recovery_timeout',
        default_value='10.0',
        description='リカバリーモードの継続時間（秒）'
    )

    normal_timeout_arg = DeclareLaunchArgument(
        'normal_timeout',
        default_value='3.0',
        description='平常状態に戻るまでの時間（秒）'
    )

    check_rate_arg = DeclareLaunchArgument(
        'check_rate',
        default_value='10.0',
        description='チェック頻度（Hz）'
    )

    # Stack Detector Node
    stack_detector_node = Node(
        package='stack_detector',
        executable='stack_detector_exe',
        name='stack_detector_node',
        output='screen',
        parameters=[{
            'cmd_vel_command_topic': LaunchConfiguration('cmd_vel_command_topic'),
            'cmd_vel_safe_topic': LaunchConfiguration('cmd_vel_safe_topic'),
            'stuck_state_topic': LaunchConfiguration('stuck_state_topic'),
            'recovery_status_topic': LaunchConfiguration('recovery_status_topic'),
            'velocity_threshold': LaunchConfiguration('velocity_threshold'),
            'stuck_timeout': LaunchConfiguration('stuck_timeout'),
            'recovery_timeout': LaunchConfiguration('recovery_timeout'),
            'normal_timeout': LaunchConfiguration('normal_timeout'),
            'check_rate': LaunchConfiguration('check_rate'),
        }]
    )

    return LaunchDescription([
        cmd_vel_command_topic_arg,
        cmd_vel_safe_topic_arg,
        stuck_state_topic_arg,
        recovery_status_topic_arg,
        velocity_threshold_arg,
        stuck_timeout_arg,
        recovery_timeout_arg,
        normal_timeout_arg,
        check_rate_arg,
        stack_detector_node,
    ])
