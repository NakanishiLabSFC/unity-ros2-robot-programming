import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    LDS_LAUNCH_FILE = '/hlds_laser_custom.launch.py'
    ROS_NAMESPACE = os.environ['ROS_NAMESPACE'] #<<編集箇所-1

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')
    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_bringup_custom'), #<<編集箇所-2
            'params',
            TURTLEBOT3_MODEL + '_' +  ROS_NAMESPACE + '.yaml')) #<<編集箇所-3
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    scan_frame_id = 'base_scan' #<<編集箇所-4
    if(len(ROS_NAMESPACE) != 0):
        scan_frame_id = ROS_NAMESPACE + '/' + scan_frame_id

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot3_state_publisher_custom.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'namespace': ROS_NAMESPACE}.items(), #<<編集箇所-5
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), LDS_LAUNCH_FILE]),
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': scan_frame_id, 'namespace': ROS_NAMESPACE}.items(), #<<編集箇所-6
        ),

        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            namespace=ROS_NAMESPACE, #<<編集箇所-7
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen'),
    ])
