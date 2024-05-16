import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():

    ld = LaunchDescription()

    # map file
    map_file_path = os.path.join(
        get_package_share_directory('ezbot_robot'),
        'maps',
        'testMap',
        'testMap.yaml'
    )

    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        namespace='map',
        parameters=[{'yaml_filename': map_file_path},
                    {'frame_id': 'map'}],
        remappings=[('/map/map', '/map')]
    )


    lifecycle_nodes = ['map_server']
    autostart = True

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            namespace='map',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'autostart': autostart},
                        {'node_names': lifecycle_nodes}])


    ld.add_action(map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld
