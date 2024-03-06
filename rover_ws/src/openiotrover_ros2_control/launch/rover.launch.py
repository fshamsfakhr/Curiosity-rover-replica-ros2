# Developer Farhad Shamsfakhr
# Copyright 2023 Open IoT Robotics Foundation, Inc.
# Licensed under the Open IoT License, Version 1.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     TBD
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers


def generate_launch_description():

    pkg_box_car_gazebo = get_package_share_directory('openiotrover_ros2_control')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    gazebo_ros2_control_demos_path = os.path.join(
        get_package_share_directory('openiotrover_ros2_control'))

    xacro_file = os.path.join(gazebo_ros2_control_demos_path,
                              'urdf',
                              'rover.xacro.urdf')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_velocity_publisher = Node(
        package='openiotrover_ros2_control',
        executable='joint_velocity_publisher_node.py',
        name='joint_velocity_publisher',
        output='screen',
    )

    state_publisher = Node(
        package='openiotrover_ros2_control',
        executable='state_pub.py',
        name='entity_state_publisher',
        output='screen',
    )
 
    scan_control = Node(
        package='openiotrover_ros2_control',
        executable='scan_ctrl.py',
        name='scan_control',
        output='screen',
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
        '-topic', 'robot_description',
        '-entity', 'cartpole',
        '-x', '1.0',  # Set your desired X position
        '-y', '2.0',  # Set your desired Y position
        '-z', '3.0',  # Set your desired Z position
        '-R', '0.0',  # Set your desired roll orientation
        '-P', '0.0',  # Set your desired pitch orientation
        '-Y', '0.0',  # Set your desired yaw orientation
        ],
    )
    
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_velocity_publisher,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_box_car_gazebo, 'worlds', 'rover_empty.world'), ''],
          description='SDF world file'),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        joint_velocity_publisher,
        # state_publisher,
        scan_control,
        launch.actions.TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='openiotrover_ros2_control',
                    executable='state_pub.py',
                    name='entity_state_publisher',
                    output='screen',
                ),
            ],
        ),
    ])
