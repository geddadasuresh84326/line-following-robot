from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution,Command
import os 
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    pkg_share = FindPackageShare(package='robot_description').find('robot_description')

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'urdf',
                'bumperbot.urdf.xacro'  
    ])
        ]
    )
    robot_description = {"robot_description":robot_description_content}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters = [robot_description]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
        package="rviz2",
        executable='rviz2',
        name = "rviz2",
        output = "screen",
        arguments=["-d",os.path.join(get_package_share_directory("robot_description"),"rviz","display.rviz")]
    )
    ])
