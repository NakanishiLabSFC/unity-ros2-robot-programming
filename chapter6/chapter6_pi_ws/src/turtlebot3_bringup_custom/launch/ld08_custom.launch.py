#!/usr/bin/env python3
#
# Copyright 2021 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Will Son


from launch import LaunchDescription
from launch_ros.actions import Node

import os

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    ROS_NAMESPACE = os.environ['ROS_NAMESPACE'] #<<編集箇所-1
    port = LaunchConfiguration('port', default='/dev/ttyUSB0')
    frame_id = LaunchConfiguration('frame_id', default='base_scan')

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
            # remappings=[('/scan', 'scan')],
            # parameters=[{'port': port, 'frame_id': frame_id}],
            parameters=[{'frame_id': frame_id}],
            output='screen'),
    ])