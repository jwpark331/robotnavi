"""Gazebo(시청 2층 월드) + 로봇 스폰 + robot_state_publisher.

    ros2 launch cliff_demo sim.launch.py gui:=true
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from cliff_demo import geometry as G


def generate_launch_description():
    pkg = get_package_share_directory("cliff_demo")
    gz = get_package_share_directory("gazebo_ros")
    robot = xacro.process_file(os.path.join(pkg, "urdf", "servi.urdf.xacro")).toxml()
    gui = LaunchConfiguration("gui")
    ir = LaunchConfiguration("ir")

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("world", default_value=os.path.join(pkg, "worlds", "city_hall.world"),
                              description="월드 파일 (젖은 바닥 실험은 회차 폴더에 만든 월드)"),
        DeclareLaunchArgument("ir", default_value="false", description="하향 IR 급정지 켜기 (P1i)"),
        DeclareLaunchArgument("mat_dropout", default_value=str(G.MAT_DROPOUT)),
        DeclareLaunchArgument("seed", default_value="1"),
        DeclareLaunchArgument("ir_log", default_value=""),
        DeclareLaunchArgument("ir_plus", default_value="false", description="P3+: 속도 상한 적용 · IR 지점 기억 · 후진"),
        DeclareLaunchArgument("ir_rear", default_value="false", description="P1irb: 뒤 IR로 후진도 막기"),
        DeclareLaunchArgument("spawn_x", default_value=str(G.START[0])),
        DeclareLaunchArgument("spawn_y", default_value=str(G.START[1])),
        DeclareLaunchArgument("spawn_yaw", default_value=str(G.START[2])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gz, "launch", "gzserver.launch.py")),
                                 launch_arguments={"world": LaunchConfiguration("world"), "verbose": "false"}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gz, "launch", "gzclient.launch.py")),
                                 condition=IfCondition(gui)),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="log",
             parameters=[{"robot_description": robot, "use_sim_time": True}]),
        # 모든 조건에서 Nav2 → /cmd_vel → ir_guard → /cmd_vel_hw → 바퀴. ir=false면 그대로 넘긴다.
        Node(package="cliff_demo", executable="ir_guard", output="log",
             parameters=[{"use_sim_time": True, "enabled": ParameterValue(ir, value_type=bool),
                          "mat_dropout": ParameterValue(LaunchConfiguration("mat_dropout"), value_type=float),
                          "seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
                          "log_path": LaunchConfiguration("ir_log"),
                          "plus": ParameterValue(LaunchConfiguration("ir_plus"), value_type=bool),
                          "rear": ParameterValue(LaunchConfiguration("ir_rear"), value_type=bool)}]),
        Node(package="gazebo_ros", executable="spawn_entity.py", output="log",
             arguments=["-entity", "servi", "-topic", "robot_description",
                        "-x", LaunchConfiguration("spawn_x"), "-y", LaunchConfiguration("spawn_y"), "-z", "0.01",
                        "-Y", LaunchConfiguration("spawn_yaw")]),
    ])
