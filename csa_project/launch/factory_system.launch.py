from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    gui_node = Node(
        package="csa_project",
        executable="factory_gui",
        name="factory_gui",
        output="screen",
    )

    resource_controller_node = Node(
        package="csa_project",
        executable="ResourceController",
        name="ResourceController",
        output="screen",
    )

    execute_skill_node = Node(
        package="csa_project",
        executable="execute_skill_action",
        name="execute_skill_action",
        output="screen",
    )

    rl_action_node = Node(
        package="csa_project",
        executable="rl_action",
        name="rl_action",
        output="screen",
    )

    change_color_node = Node(
        package="csa_project",
        executable="set_product_color",
        name="change_product_color",
        output="screen",
    )

    return LaunchDescription(
        [
            resource_controller_node,
            gui_node,
            execute_skill_node,
            rl_action_node,
            change_color_node,
        ]
    )
