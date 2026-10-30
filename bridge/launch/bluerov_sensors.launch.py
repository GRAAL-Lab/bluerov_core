from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    dvl_ip = LaunchConfiguration('dvl_ip')
    sonar_ip = LaunchConfiguration('sonar_ip')
    host_ip = LaunchConfiguration('host_ip')
    camera_port = LaunchConfiguration('camera_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'dvl_ip',
            default_value='192.168.2.125',
            description='Water Linked A50 DVL IP address',
        ),
        DeclareLaunchArgument(
            'sonar_ip',
            default_value='192.168.2.197',
            description='Water Linked Sonar 3D-15 IP address',
        ),
        DeclareLaunchArgument(
            'host_ip',
            default_value='192.168.2.1',
            description="This computer's sensor-network IPv4 address",
        ),
        DeclareLaunchArgument(
            'camera_port',
            default_value='5601',
            description='UDP port for the ROS H.264 RTP camera stream',
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
        Node(
            package='bluerov_video_viewer',
            executable='bluerov_video_viewer_node',
            name='bluerov_video_viewer',
            output='screen',
            parameters=[{
                'port': ParameterValue(camera_port, value_type=int),
                'show_window': False,
                'publish_ros_topic': True,
            }],
            remappings=[('image_raw', 'camera/image_raw')],
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
