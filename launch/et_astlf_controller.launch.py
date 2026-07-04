from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("et_astlf_path_tracking"), "config", "et_astlf_params.yaml"]
                ),
                description="Path to the ASTLF controller parameter file.",
            ),
            Node(
                package="et_astlf_path_tracking",
                executable="et_astlf_controller_node",
                name="et_astlf_controller_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
