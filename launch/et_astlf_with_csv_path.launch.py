from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    csv_file = LaunchConfiguration("csv_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("et_astlf_control"), "config", "et_astlf_params.yaml"]
                ),
                description="Path to ET-ASTLF controller parameter file.",
            ),
            DeclareLaunchArgument(
                "csv_file",
                description="CSV file containing x,y path points.",
            ),
            Node(
                package="et_astlf_control",
                executable="csv_path_publisher",
                name="csv_path_publisher",
                output="screen",
                parameters=[params_file, {"csv_file": csv_file}],
            ),
            Node(
                package="et_astlf_control",
                executable="et_astlf_node",
                name="et_astlf_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
