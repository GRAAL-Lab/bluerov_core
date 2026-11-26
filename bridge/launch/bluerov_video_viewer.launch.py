from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("bridge"))
        / "param"
        / "bluerov_video.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            Node(
                package="bridge",
                executable="bluerov_video_viewer_node",
                name="bluerov_video_viewer",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
