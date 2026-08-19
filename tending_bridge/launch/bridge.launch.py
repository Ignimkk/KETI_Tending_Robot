# tending_bridge — Windows GUI(KETI_CNCTendingRobot) 연동 TCP/JSON 브리지
#
# 선행 조건: inspection_manager 와 tm_driver 가 이미 떠 있어야 한다.
#   1) 로봇 브링업 (tmr_ros2 공식 런치)
#        실물:   ros2 launch tm_driver tm_bringup.launch.py <ROBOT_IP>
#        가상:   ros2 launch tm5-900_moveit_config tm5-900_run_move_group.launch.py
#   2) 검사 오케스트레이션
#        ros2 launch tending_control mvp_inspect.launch.py
#   3) 본 런치
#        ros2 launch tending_bridge bridge.launch.py
#
# 포트만 바꾸려면:  ros2 launch tending_bridge bridge.launch.py port:=5901
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bridge_yaml = os.path.join(
        get_package_share_directory('tending_bridge'), 'config', 'bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('bind_address', default_value='0.0.0.0',
                              description='TCP 리스닝 주소'),
        DeclareLaunchArgument('port', default_value='5901',
                              description='TCP 리스닝 포트 (TM 5890/5891 과 충돌 회피)'),
        DeclareLaunchArgument('allowed_client_ip', default_value='172.21.60.68',
                              description='접속을 허용할 Windows GUI IPv4 (빈 값이면 전체 허용)'),
        DeclareLaunchArgument('max_clients', default_value='1',
                              description='동시 TCP 클라이언트 상한'),
        Node(
            package='tending_bridge', executable='tending_bridge', name='tending_bridge',
            output='screen',
            parameters=[
                bridge_yaml,
                # 런치 인자는 YAML 보다 뒤에 두어 덮어쓰게 한다.
                {'bind_address': ParameterValue(
                    LaunchConfiguration('bind_address'), value_type=str)},
                # 런치 인자는 문자열이므로 int 로 명시 변환해야 파라미터 타입이 맞는다.
                {'port': ParameterValue(LaunchConfiguration('port'), value_type=int)},
                {'allowed_client_ip': ParameterValue(
                    LaunchConfiguration('allowed_client_ip'), value_type=str)},
                {'max_clients': ParameterValue(
                    LaunchConfiguration('max_clients'), value_type=int)},
            ],
        ),
    ])
