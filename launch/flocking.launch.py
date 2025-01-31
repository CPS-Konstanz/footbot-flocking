from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    num_robots = 10

    nodes = [
        Node(
            package='flocking',
            executable='flocking',
            namespace=f'bot{i}',
            name=f'flocking_bot_{i}',
            output='screen',
            parameters=[]
        )
        for i in range(num_robots)
    ]

    return LaunchDescription(nodes)
