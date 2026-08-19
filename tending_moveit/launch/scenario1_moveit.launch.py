# 시나리오 1 MoveIt 검사 노드 실행 (제어방식 2, 충돌회피)
#
# MoveGroupInterface 클라이언트는 robot_description/SRDF/kinematics 파라미터가 필요하다.
# ★ tmr_ros2 의 run_move_group.launch.py 와 "동일한 방식"으로 description 을 구성해야
#    move_group 서버와 모델이 일치한다(MoveItConfigsBuilder 는 URDF/SRDF 링크명 불일치로
#    tmr_arm 그룹이 비므로 사용하지 않음).
#
# 전제: move_group 이 이미 떠 있어야 한다.
#   실물:   ros2 launch tm5-700_moveit_config tm5-700_run_move_group.launch.py robot_ip:=<IP>
#   가상:   ros2 launch tm5-700_moveit_config tm5-700_run_move_group.launch.py   (robot_ip 생략 = fake 로봇)
# 그런 다음:
#   ros2 launch tending_moveit scenario1_moveit.launch.py
import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _load_file(pkg, rel):
    path = os.path.join(get_package_share_directory(pkg), rel)
    with open(path, 'r') as f:
        return f.read()


def _load_yaml(pkg, rel):
    path = os.path.join(get_package_share_directory(pkg), rel)
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    moveit_pkg = 'tm5-700_moveit_config'

    # run_move_group 과 동일: robot_description 은 tm_description 의 xacro 로.
    urdf_xacro = os.path.join(
        get_package_share_directory('tm_description'), 'xacro', 'tm5-700.urdf.xacro')
    robot_description = {
        'robot_description': xacro.process_file(urdf_xacro).toxml()}
    robot_description_semantic = {
        'robot_description_semantic': _load_file(moveit_pkg, 'config/tm5-700.srdf')}
    robot_description_kinematics = {
        'robot_description_kinematics': _load_yaml(moveit_pkg, 'config/kinematics.yaml')}

    scenario1_yaml = os.path.join(
        get_package_share_directory('tending_control'), 'config', 'scenario1.yaml')

    return LaunchDescription([
        Node(
            package='tending_moveit',
            executable='scenario1_inspect_moveit',
            name='scenario1_inspect_moveit',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                scenario1_yaml,
            ],
        ),
    ])
