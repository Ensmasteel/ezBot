# copyright option3 ensma
import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import Command
import xacro



def generate_launch_description():

    # Declare the launch arguments
    # check if log_level is set

    logger = LaunchConfiguration('log_level')  
    # if not set, set to debug
    if logger is None:
        logger = 'debug'



    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='ezbot_robot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()

    )

    #robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    #controller_params_file = os.path.join(get_package_share_directory('ezbot_robot'), 'config', 'controllers.yaml')
    controller_params_file = os.path.join(get_package_share_directory('ezbot_robot'), 'config', 'omnidirectional_controller.yaml')

 
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file],
        remappings = [('/controller_manager/robot_description', '/robot_description')],
    )





    delayed_controller_manager = TimerAction(
        period=2.0,
        actions=[controller_manager],
    )


    omnidrive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omnidirectional_controller'],
    )
    delayed_omnidrive_spawner = TimerAction(
        period=1.0,
        actions=[omnidrive_spawner],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    delayed_joint_broad_spawner = TimerAction(
        period=1.0,
        actions=[joint_broad_spawner],
    )   



    #joystick = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([os.path.join(
    #        get_package_share_directory("teleop_twist_joy"),'launch','teleop-launch.py'
    #    )]), launch_arguments={'config_filepath': '/home/vincent/joystick.yaml','joy_vel':'/omnidirectional_controller/cmd_vel_unstamped'}.items(),
    #    
    #)

    #homemade_controller = Node(
    #    package='ezbot_robot',
    #    executable='homemade_controller',
    #    name='homemade_controller',
    #    output='screen',
    #    parameters=[{'log_level': logger}]
    #)

    return LaunchDescription([
        rsp,
        delayed_controller_manager,
        delayed_omnidrive_spawner,
        delayed_joint_broad_spawner
        #joystick,
        #homemade_controller
        
    ])