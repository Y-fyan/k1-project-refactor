from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=PathJoinSubstitution([
            FindPackageShare("game_controller"), "config", "config.yaml"])),
        Node(package="game_controller", executable="game_controller_node",
             name="game_controller", output="screen",
             parameters=[LaunchConfiguration("config")]),
    ])
