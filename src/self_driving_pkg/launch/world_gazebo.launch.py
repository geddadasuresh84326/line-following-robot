#!/usr/bin/env python3

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command,PathJoinSubstitution

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_share_dir = get_package_share_directory("self_driving_pkg")
    model_share_dir = os.path.join(pkg_share_dir,"models")
    robot_description_pkg = get_package_share_directory("robot_description")

    # GAZEBO_MODEL_PATH 
    os.environ['GAZEBO_MODEL_PATH'] = (
        os.environ.get("GAZEBO_MODEL_PATH",'') + os.pathsep + model_share_dir
    )
    
    # world path 
    world = os.path.join(pkg_share_dir,"worlds","b.world")

    # launch gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros,"launch","gzserver.launch.py")
        ),
        launch_arguments={'world':world}.items()
    )

    # launch gazebo client (GUI)
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros,"launch","gzclient.launch.py")
        )
    )

    # urdf robot path
    urdf_xml = Command(['xacro ', os.path.join(robot_description_pkg, 'urdf', 'bumperbot.urdf.xacro')])
    
    # Publish robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_xml}]
    )

    # spawning robot into gazebo 
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=['-entity',"lane_following_robot1","-topic",'robot_description'],
        output = "screen"
    )

    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_entity)
    ld.add_action(robot_state_publisher_node)

    return ld
