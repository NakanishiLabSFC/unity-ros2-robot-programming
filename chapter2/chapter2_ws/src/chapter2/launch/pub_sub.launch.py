from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chapter2',
            executable='simple_publisher',
        ),
        Node(
            package='chapter2',
            executable='simple_subscriber',
        ),
    ])
