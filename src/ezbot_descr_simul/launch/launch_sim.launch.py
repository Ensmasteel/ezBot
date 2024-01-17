# copyright option3 ensma
import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='ezbot_descr_simul' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    rsp2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ezbot_static_camera'),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'verbose': 'true'}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'EzBot',
                                   '-x','1.0',
                                   '-y','-1.0',
                                   '-z', '1.0' ],
                        output='screen')

    spawn_entity2 = Node(package='gazebo_ros', executable='spawn_entity.py',
                        namespace='camera_namespace',
                        name='Pascameracontroller',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'camera',
                                   '-x','1.0',
                                   '-y','-1.0',
                                   '-z', '1.0' ],
                        output='screen')
    
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                       get_package_share_directory("teleop_twist_joy"),'launch','teleop-launch.py'
                  )]), launch_arguments={'config_filepath': '/home/vincent/joystick.yaml'}.items())


    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            #default_value=[os.path.join('ezbot_gazebo','worlds','table2024.world'), ''],
            default_value=[os.path.join(get_package_share_directory("ezbot_gazebo"), 'worlds', 'table2024Poteaux.world'), ''],
            description='SDF world file'),
        rsp,
        spawn_entity,
        rsp2,
        spawn_entity2,
        gazebo,
        joystick
    ])