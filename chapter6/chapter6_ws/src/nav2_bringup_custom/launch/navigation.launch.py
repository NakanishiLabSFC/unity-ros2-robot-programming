import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ROS_NAMESPACE = os.environ['ROS_NAMESPACE']
    
    bringup_dir = get_package_share_directory('nav2_bringup_custom')
    default_params= os.path.join(bringup_dir, 'params', 'nav2_params_' + ROS_NAMESPACE + '.yaml')

    params_file = LaunchConfiguration('params_file', default=default_params)    
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    autostart = LaunchConfiguration('autostart')

    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']
    
    return LaunchDescription([
        Node(
            namespace=ROS_NAMESPACE,
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            respawn=False,
            respawn_delay=2.0,
            parameters=[params_file,
                        {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=[('/' + ROS_NAMESPACE + '/cmd_vel', '/' + ROS_NAMESPACE + '/cmd_vel_nav')
        ]),
        Node(
            namespace=ROS_NAMESPACE,
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            respawn=False,
            respawn_delay=2.0,
            parameters=[params_file,
                        {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(
            namespace=ROS_NAMESPACE,
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=False,
            respawn_delay=2.0,
            parameters=[params_file,
                        {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(
            namespace=ROS_NAMESPACE,
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=False,
            respawn_delay=2.0,
            parameters=[params_file,
                        {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(
            namespace=ROS_NAMESPACE,
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=False,
            respawn_delay=2.0,
            parameters=[params_file,
                        {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(
            namespace=ROS_NAMESPACE,
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            respawn=False,
            respawn_delay=2.0,
            parameters=[params_file,
                        {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(
            namespace=ROS_NAMESPACE,
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            respawn=False,
            respawn_delay=2.0,
            parameters=[params_file,
                        {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=[('/' + ROS_NAMESPACE + '/cmd_vel', '/' + ROS_NAMESPACE + '/cmd_vel_nav'), 
                        ('/' +  ROS_NAMESPACE +'/cmd_vel_smoothed', '/' + ROS_NAMESPACE + '/cmd_vel')]
            ),
        Node(
            namespace=ROS_NAMESPACE,
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),
    ])
