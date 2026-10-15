from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    dvl_ip = LaunchConfiguration('dvl_ip')
    sonar_ip = LaunchConfiguration('sonar_ip')
    host_ip = LaunchConfiguration('host_ip')

    return LaunchDescription([
        DeclareLaunchArgument(
            'dvl_ip',
            default_value='192.168.2.125',
            description='Water Linked A50 DVL IP address',
        ),
        DeclareLaunchArgument(
            'sonar_ip',
            default_value='192.168.2.196',
            description='Water Linked Sonar 3D-15 IP address',
        ),
        DeclareLaunchArgument(
            'host_ip',
            default_value='192.168.2.3',
            description="This computer's sensor-network IPv4 address",
        ),
        Node(
            package='bridge',
            executable='bridge_node',
            name='mavlink_bridge',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('bridge'),
                    'param',
                    'bridge.yaml',
                ])
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('sonar3d'),
                    'launch',
                    'sonar3d.launch.py',
                ])
            ),
            launch_arguments={
                'sonar_ip': sonar_ip,
                'local_interface_ip': host_ip,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('dvl_a50'),
                    'launch',
                    'dvl_a50.launch.py',
                ])
            ),
            launch_arguments={'ip_address': dvl_ip}.items(),
        ),
    ])
