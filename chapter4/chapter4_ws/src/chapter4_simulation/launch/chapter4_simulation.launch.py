import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    package_dir = get_package_share_directory("chapter4_description")
    urdf = os.path.join(package_dir, "urdf", "turtlebot3_burger_chapter4.urdf")
    
    rviz_dir = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'rviz',
        'model.rviz')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_world.world'
    )
    

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=[urdf]),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            arguments=[urdf]),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_dir],
            output='screen'),
        
        # ExecuteProcess(
        #     cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"],
        #     ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items()
        ), 

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        ), 
        

        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="urdf_spawner",
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=["-topic", "/robot_description", "-entity", "turtlebot3_burger_chapter4"]),
    ])
