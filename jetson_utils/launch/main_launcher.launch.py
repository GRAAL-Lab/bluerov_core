from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    vision_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('image_pipeline_launcher'),
                'launch',
                'vision_pipeline_launch.py'
            ])
        )
    )
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('image_pipeline_obstacle_tracking'),
                'launch/launch/',
                'perception.py'
            ])
        )
    )
    mission_control_monitor = Node(
        package='mission_ctrl',
        executable='controller_node',
        name='controller_node'
    )
    mission_control_controller = Node(
        package='mission_ctrl',
        executable='monitor_node',
        name='monitor_node'
    )
    kcl = Node(
        package='kcl',
        executable='kinematic_control_layer_node',
        name='kinematic_control_layer_node'
    )
    logger = Node(
        package='logger',
        executable='logger_node',
        name='logger_node'
    )
    bridge = Node(
        package='bridge',
        executable='bridge_node',
        name='bridge_node'
    )

    return LaunchDescription([
        perception,
        vision_pipeline,
        mission_control_monitor,
        mission_control_controller,
        kcl,
        logger,
        bridge
    ])
