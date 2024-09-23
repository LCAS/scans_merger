from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scans_merger',
            executable='scans_merger',
            name='cloud_merger_node',
            output='screen',
            parameters=[{
                'destination_frame': 'base_link',
                'input_cloud_1': '/front_lidar/points',
                'input_cloud_2': '/back_lidar/points',
                'merged_cloud': '/merged_cloud'
            }]
        )
    ])
