
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        ExecuteProcess(
            cmd=['rm', '-rf', '/home/lucas/.ros/bags/rami/rami_results'],
            output='screen'
        ),
        
        # Vision node
        Node(
            package='image_pipeline_obstacle_tracking',
            executable='python_yolo_publisher.py',
            name='marine_detector',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
                   
        # Marine detector node
        Node(
            package='image_pipeline_obstacle_tracking',
            executable='marine_detector_node',
            name='marine_detector',
            output='screen',
            arguments=['--settings', '/home/lucas/ros2_ws_rami/src/obstacle_tracking_rami/data/rami.cfg'],
            parameters=[{'use_sim_time': True}]
        ),
        
        # Record bag process
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record', 
                '/dtc/stats', '/dtc/obstacles', '/dtc/detection_settings',
                '/trk/tracks'
                '-o', '/home/lucas/.ros/bags/rami/rami_results'
            ],
            output='screen'
        ),

        # Play bag process
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/home/lucas/rami/rami/rami.db3', '--clock', '--start-offset', '0'],
            output='screen'
        )
    ])
