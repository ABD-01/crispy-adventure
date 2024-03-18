# Visualizing Laser Scan Data

```{image} ../_static/gifs/Jack_Jack_Laser_Vision.webp
---
alt: talker-listener
align: center
width: 70%
---
```

This part involves using laser sensors to scan the environment and visualize the data in rviz2.

## Pre-requisites

I have used `turtlebot3_burger` as the robot model, and [`dynamic_world`](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps/tree/master/worlds/dynamic_world) as the world.

### Environment Setup

My `~/.bashrc` includes these lines:
```bash
source /opt/ros/foxy/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/foxy/
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/opt/ros/foxy/share/turtlebot3_gazebo/models/:$GAZEBO_MODEL_PATH
```

Also, the package models if any need to be added to the path as well.

```bash
export GAZEBO_MODEL_PATH=/home/ros2_ws/src/jde_ros2_asgn/models:$GAZEBO_MODEL_PATH
```

### Nodes

To view the Laser Scans we need atleast 4 nodes:

**Gazebo** - The simulation environment.
```
gazebo $(ros2 pkg prefix --share jde_ros2_asgn)/worlds/dynamic_world.world
```

**Robot State Publisher** - To broadcast the state of the robot to the [tf2](https://wiki.ros.org/tf2) transform library.
```
ros2 run robot_state_publisher robot_state_publisher $(ros2 pkg prefix --share turtlebot3_description)/urdf/turtlebot3_burger.urdf
```

**Teleop Twist Keyboard** - The title is self-explanatory.
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
**RViz2** - ROS vizualtion tool
```
rviz2
```

## Launching the nodes

I have created a launch file named `laser_scan.launch` in `launch` directory to launch all the nodes from one command.
(Except for the keyboard node, you know why.)

```bash
ros2 launch jde_ros2_asgn laser_scan.launch.py
```
and 
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Source Code

````{dropdown} laser_scan.launch.py
```python
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
```
````

## Use of AI

Why add this section? Because I read a discussion where [@jmplaza](https://github.com/jmplaza) mentioned how selection process is more rigorous in these ChatGPT times.
<br>
I have not used any assitance of AI tools like ChatGPT, co-pilot, etc for the programming part of this challenge.
<br>
Also, AI is still not very good at debugging ROS errors.
<br>
I have included the reveleant resources I used during this challenge.


## References

* Gazebo not showing my models: [include a model which is not in default path](https://answers.gazebosim.org/question/24935/how-to-include-a-model-which-is-not-in-default-path/#:~:text=You%20can%20check%20the%20current%20value%20of%20GAZEBO_MODEL_PATH,them%20using%20%3A%20export%20GAZEBO_MODEL_PATH%3D%24GAZEBO_MODEL_PATH%3A%3Cpath%20to%20your%20model%3E)
* Dynamic World Model: [Dataset-of-Gazebo-Worlds-Models-and-Maps](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps/tree/master/worlds/dynamic_world)
* View URDF in Gazebo: [URDF](https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/URDF-Main.html) & [Tutorial: Using a URDF in Gazebo](https://classic.gazebosim.org/tutorials?tut=ros_urdf)
* Racecar URDF: [mit-racecar/racecar_gazebo](https://github.com/mit-racecar/racecar_gazebo/blob/master/racecar_description/urdf/racecar.xacro)  