# -*- coding: utf-8 -*-

"""
Description: TODO
Author: Muhammed Abdullah Shaikh
Date Created: Mar 14, 2024
Last Modified: Mar 14, 2024
Python Version: 3.8.11
License: BSD-3-Clause License
"""

import launch
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    pkg_jde_ros2_asgn = FindPackageShare('jde_ros2_asgn')
    pkg_gazebo_ros  = FindPackageShare('gazebo_ros')
    pkg_turtlebot3_descrition  = FindPackageShare('turtlebot3_description')

    urdf_path = PathJoinSubstitution([pkg_turtlebot3_descrition, 'urdf', 'turtlebot3_burger.urdf'])
    world_path = PathJoinSubstitution([pkg_jde_ros2_asgn, 'worlds', 'dynamic_world.world']) 
    rviz_config_path =  PathJoinSubstitution([pkg_jde_ros2_asgn, 'rviz', 'laser_scan.rviz'])  

    return launch.LaunchDescription([
        
        DeclareLaunchArgument(
            name='world', 
            default_value=world_path,
            description='Absolute path to world file'
        ),

        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=rviz_config_path,
            description='Absolute path to rviz config file'
        ),

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzserver.launch.py'])
            ),
            launch_arguments={'world': LaunchConfiguration('world')}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                 PathJoinSubstitution([pkg_gazebo_ros, 'launch','gzclient.launch.py'])
            ),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time', default='false')}],
            arguments=[urdf_path]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        ),

    ])