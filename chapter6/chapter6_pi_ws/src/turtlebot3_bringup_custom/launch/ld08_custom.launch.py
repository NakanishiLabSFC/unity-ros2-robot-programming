from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    port = LaunchConfiguration('port', default='/dev/ttyUSB0')
    frame_id = LaunchConfiguration('frame_id', default='base_scan')
    ROS_NAMESPACE = os.environ['ROS_NAMESPACE'] #<<編集箇所-1

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value=port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar. Default frame_id is \'laser\''),

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Specifying namespace of robot'), 

        Node(
            package='ld08_driver',
            executable='ld08_driver',
            name='ld08_driver',
            namespace=ROS_NAMESPACE,
            parameters=[{'frame_id': frame_id}],
            output='screen'),
    ])