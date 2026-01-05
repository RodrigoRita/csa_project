import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_robot_group(robot_name: str, delay: float = 2.0):
    """Launch AMCL + lifecycle manager for a robot with optional delay"""
    pkg_share = get_package_share_directory("csa_project")
    params_file = os.path.join(pkg_share, "params", f"nav2_params_{robot_name}.yaml")

    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "amcl.launch.py")
        ),
        launch_arguments={
            "use_namespace": "True",
            "use_sim_time": "True",
            "params_file": params_file,
        }.items(),
    )

    # Delay the robot group to avoid lifecycle manager waiting too early
    return TimerAction(
        period=delay, actions=[GroupAction([PushRosNamespace(robot_name), amcl_launch])]
    )


def generate_launch_description():
    map_file_default = "/home/vscode/trsa_ws/src/csa_project/world/factory_map_nav.yaml"

    map_arg = DeclareLaunchArgument(
        "map_file",
        default_value=map_file_default,
        description="Map YAML file",
    )

    # Map server node
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": LaunchConfiguration("map_file")},
            {"use_sim_time": True},
        ],
    )

    # Map lifecycle manager with a small delay to ensure map_server is ready
    map_lifecycle_manager = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"autostart": True},
                    {"node_names": ["map_server"]},
                ],
            )
        ],
    )

    robot_names = ["AGV1", "AGV2", "AGV3", "AGV4", "AGV5", "AGV6", "AGV7", "AGV8"]

    ld = LaunchDescription()
    ld.add_action(map_arg)
    ld.add_action(map_server)
    ld.add_action(map_lifecycle_manager)

    # Add robot groups with incremental delays to avoid simultaneous lifecycle manager start
    delay_increment = 3.0
    current_delay = 4.0  # start after map lifecycle
    for name in robot_names:
        ld.add_action(generate_robot_group(name, delay=current_delay))
        current_delay += delay_increment

    return ld
