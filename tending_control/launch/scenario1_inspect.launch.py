# 시나리오 1 검사 — 제어방식 (1) 룰베이스 / (3) 하이브리드 (tending_control::scenario1_inspect)
#
# 로봇 브링업은 tmr_ros2 공식 런치로 별도 수행한다(본 런치는 제어 노드만 띄운다).
#   실물 예:  ros2 launch tm5-700_moveit_config tm5-700_run_move_group.launch.py robot_ip:=<IP>
#            (또는 드라이버만: ros2 launch tm_driver tm_bringup.launch.py <IP>)
#
# 예:
#   # 오프라인(포즈만 출력, 로봇 불필요)
#   ros2 launch tending_control scenario1_inspect.launch.py control_mode:=rule dry_run:=true
#   # 실물 (tmr_ros2 로 로봇 bringup 후)
#   ros2 launch tending_control scenario1_inspect.launch.py control_mode:=hybrid dry_run:=false
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('tending_control'), 'config', 'scenario1.yaml')

    control_mode = LaunchConfiguration('control_mode')
    dry_run = LaunchConfiguration('dry_run')

    return LaunchDescription([
        DeclareLaunchArgument('control_mode', default_value='rule',
                              description='rule | hybrid'),
        DeclareLaunchArgument('dry_run', default_value='true',
                              description='true: 포즈만 출력(오프라인) / false: 실제 이동(로봇 필요)'),
        Node(
            package='tending_control', executable='scenario1_inspect', name='scenario1_inspect',
            output='screen',
            parameters=[params, {'control_mode': control_mode, 'dry_run': dry_run}],
        ),
    ])
