from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        namespace=namespace,
        parameters=[
            {"use_sim_time": use_sim_time},
            params_file,
            {"use_namespace": True},
            {"global_frame_id": "map", "tf_broadcast": True},
        ],
        remappings=[
            # ("/tf", [namespace, TextSubstitution(text="/tf")]),
            # ("/tf_static", [namespace, TextSubstitution(text="/tf_static")]),
            ("/tf", "/tf"),
            ("/tf_static", "/tf_static"),
            ("map", "/map"),
            ("map_metadata", "/map_metadata"),
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        namespace=namespace,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["amcl"]},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("params_file"),
            GroupAction(
                [
                    amcl_node,
                    lifecycle_manager,
                ]
            ),
        ]
    )
