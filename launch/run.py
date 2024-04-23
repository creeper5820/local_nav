from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = [
        PathJoinSubstitution([FindPackageShare("local_nav"), "config", "config.yaml"])
    ]

    node = Node(
        package="local_nav",
        executable="run",
        parameters=params,
    )

    # Assemble the launch description
    ld = LaunchDescription([node])

    return ld
