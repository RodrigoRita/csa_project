import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("csa_project")

    # Launch args
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([pkg_share, "world", "factory_world.yaml"]),
        description="Path to Flatland world file",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([pkg_share, "rviz", "nav_sim.rviz"]),
        description="RViz configuration file",
    )

    map_arg = DeclareLaunchArgument(
        "map_file",
        default_value="/home/vscode/trsa_ws/src/csa_project/world/factory_map_nav.yaml",
        description="Path to the map YAML file",
    )

    # Flatland simulation
    flatland_server = Node(
        package="flatland_server",
        executable="flatland_server",
        name="flatland_server",
        output="screen",
        parameters=[
            {
                "world_path": LaunchConfiguration("world"),
                "update_rate": 5.0,
                "step_size": 0.01,
                "show_viz": True,
                "viz_pub_rate": 30.0,
                "use_sim_time": True,
            }
        ],
    )
    # RViz visualization
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
    )
    return LaunchDescription(
        [
            world_arg,
            rviz_arg,
            map_arg,
            flatland_server,
            rviz2,
        ]
    )
