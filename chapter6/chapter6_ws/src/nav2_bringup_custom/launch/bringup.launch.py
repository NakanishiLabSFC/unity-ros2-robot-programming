import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load')
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_dir + "/launch/localization.launch.py"]),
        launch_arguments={
            'namespace': ROS_NAMESPACE,
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time, 
            'autostart': autostart
        }.items()
    )
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_dir + "/launch/navigation.launch.py"]),
        launch_arguments={
            'namespace': ROS_NAMESPACE,
            'params_file': params_file,
            'use_sim_time': use_sim_time, 
            'autostart': autostart
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(localization_launch)
    ld.add_action(navigation_launch)
    
    return ld
