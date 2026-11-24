from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    params = Path(get_package_share_directory("bridge")) / "param" / "ardusub_sitl.yaml"
    return LaunchDescription(
        [
            Node(
                package="bridge",
                executable="bridge_node",
                name="mavlink_bridge",
                output="screen",
                parameters=[str(params)],
            )
        ]
    )
