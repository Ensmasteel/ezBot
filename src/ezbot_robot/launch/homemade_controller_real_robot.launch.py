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

   
    pkg_path = os.path.join(get_package_share_directory('ezbot_robot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description = {"robot_description": xacro.process_file(xacro_file).toxml()}



    homemade_controller = Node(
        package='homemade_controller',
        executable='homemade_controller',
        name='homemade_controller',
        output='screen',
        parameters=[{'log_level': logger}]
    )

    return LaunchDescription([
        rsp,
        homemade_controller
        
    ])