import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
            package='hls_lfcd_lds_driver',
            executable='hlds_laser_publisher',
            name='hlds_laser_publisher',
            namespace=ROS_NAMESPACE, #<<編集箇所-2
            remappings=[('/scan', 'scan')],
            parameters=[{'port': port, 'frame_id': frame_id}],
            output='screen'),
    ])