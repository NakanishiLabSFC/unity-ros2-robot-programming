import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    ROS_NAMESPACE = os.environ['ROS_NAMESPACE'] #<<編集箇所-1

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='') #<<編集箇所-2

    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '_' +  ROS_NAMESPACE +'.urdf' #<<編集箇所-3
    print("urdf_file_name : {}".format(urdf_file_name))
    urdf = os.path.join(
        get_package_share_directory('turtlebot3_bringup_custom'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        #<<編集箇所-4
        DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Specifying namespace of robot'),  
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace, #<<編集箇所-5
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time}])
    ])
