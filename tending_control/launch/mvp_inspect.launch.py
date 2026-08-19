# MVP 검사 파이프라인 (검사 오케스트레이션 노드만) — camera(stub) + data + inspection_manager
#
# 로봇 브링업은 tmr_ros2 공식 런치로 별도 수행한다.
#   실물 드라이버:  ros2 launch tm_driver tm_bringup.launch.py <ROBOT_IP>
#   MoveIt 포함:    ros2 launch tm5-700_moveit_config tm5-700_run_move_group.launch.py robot_ip:=<IP>
#
# 실행 후 검사 시작(액션):
#   ros2 action send_goal /run_inspection tending_interfaces/action/RunInspection \
#     "{machine_id: 'mc_a', scenario: 1, angle_start: 0.0, angle_end: 6.283, num_views: 8, inspect_distance: 0.1}" --feedback
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    poses_yaml = os.path.join(
        get_package_share_directory('tending_control'), 'config', 'poses.yaml')

    output_dir = LaunchConfiguration('output_dir')
    use_stub = LaunchConfiguration('use_stub')

    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='/tmp/tending_dataset',
                              description='이미지/메타 저장 루트'),
        DeclareLaunchArgument('use_stub', default_value='true',
                              description='카메라 stub 모드(카메라 부착 전 true)'),
        Node(
            package='tending_camera', executable='camera_node', name='camera_node',
            output='screen',
            parameters=[{'use_stub': use_stub, 'output_dir': output_dir}],
        ),
        Node(
            package='tending_data', executable='dataset_recorder', name='dataset_recorder',
            output='screen',
            parameters=[{'output_dir': output_dir}],
        ),
        Node(
            package='tending_control', executable='inspection_manager', name='inspection_manager',
            output='screen',
            parameters=[poses_yaml],
        ),
    ])
