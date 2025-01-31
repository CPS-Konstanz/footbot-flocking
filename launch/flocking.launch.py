from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    num_robots = 10

    config_file_path = os.path.join(
        get_package_share_directory('flocking'),
        'config',
        'config.yaml'
    )
    print(f"Config file being used: {config_file_path}")
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
