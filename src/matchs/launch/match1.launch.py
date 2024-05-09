import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import launch_ros.actions

def generate_launch_description():

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ezbot_robot'),'launch', 'real_robot.launch.py'
        )]),
    )
    tirette_node = Node(
        package='ezbot_command_module',
        executable='tirette_node',
        name='tirette_node',
        output='screen'
    )

    lidar_colision_avoidance = Node(
        package='listen_package',
        executable='ecoute',
        name='ecoute',
        output='screen'
    )

    match = Node(
        package='matches',
        executable='match1',
        name='match1',
        output='screen'
    )




    return LaunchDescription([
        robot,
        tirette_node,
        lidar_colision_avoidance,
        match
    ])

if __name__ == '__main__':
    generate_launch_description()