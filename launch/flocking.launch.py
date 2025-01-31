from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    num_robots = 10

    config_file_path = os.path.join(
        os.getcwd(), 'config', 'config.yaml'
    )

    nodes = [
        Node(
            package='flocking',
            executable='flocking',
            namespace=f'bot{i}',
            name=f'flocking_bot_{i}',
            output='screen',
            parameters=[config_file_path]
        )
        for i in range(num_robots)
    ]

    return LaunchDescription(nodes)
