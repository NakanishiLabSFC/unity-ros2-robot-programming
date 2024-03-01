import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ROS_NAMESPACE = os.environ['ROS_NAMESPACE']

    bringup_dir = get_package_share_directory('nav2_bringup_custom')
    default_params= os.path.join(bringup_dir, 'params', 'nav2_params_' + ROS_NAMESPACE + '.yaml')
    default_map = os.path.join(bringup_dir, 'maps', 'map.yaml')

    map_file = LaunchConfiguration('map', default=default_map)
    params_file = LaunchConfiguration('params_file', default=default_params)    
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    autostart = LaunchConfiguration('autostart')

    lifecycle_nodes = ['map_server', 'amcl']
        
    return LaunchDescription([
        Node(
            namespace=ROS_NAMESPACE,
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'yaml_filename': map_file}]
        ),

        Node(
            namespace=ROS_NAMESPACE,
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            namespace=ROS_NAMESPACE,
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]
        ),        
    ])
