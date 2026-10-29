"""매핑: Gazebo + slam_toolbox(online async) + 정해진 경로 주행 → 지도 저장 후 종료.

    ros2 launch cliff_demo mapping.launch.py map_path:=<저장 경로(확장자 없이)>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("cliff_demo")
    slam_cfg = os.path.join(get_package_share_directory("slam_toolbox"), "config", "mapper_params_online_async.yaml")
    builder = Node(package="cliff_demo", executable="map_builder", output="screen",
                   parameters=[{"use_sim_time": True, "map_path": LaunchConfiguration("map_path")}])
    return LaunchDescription([
        DeclareLaunchArgument("map_path", default_value=os.path.join(pkg, "maps", "city_hall")),
        DeclareLaunchArgument("gui", default_value="false"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "sim.launch.py")),
                                 launch_arguments={"gui": LaunchConfiguration("gui")}.items()),
        Node(package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox", output="log",
             parameters=[slam_cfg, {"use_sim_time": True, "max_laser_range": 8.0, "map_update_interval": 1.0,
                                    "minimum_travel_distance": 0.2, "minimum_travel_heading": 0.2}]),
        builder,
        RegisterEventHandler(OnProcessExit(target_action=builder, on_exit=[EmitEvent(event=Shutdown())])),
    ])
