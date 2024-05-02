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

    actuators_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['actuator_controller'],
    )

    delayed_actuators_spawner = TimerAction(
        period=1.0,
        actions=[actuators_spawner],
    )


    imu_node =   Node(
            package='ros_qwiic_icm_20948',
            executable='ros_qwiic_icm_20948',
            name='ros_qwiic_icm_20948',
            output='screen'
    )



    lidar_with_mgr = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ldlidar_node'),'launch', 'ldlidar_with_mgr.py'
        )]),
    )




    # don't know what the parameters do
    imu_complementary_filter = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
        ],
    )

    delayed_imu_filter = TimerAction(
        period=5.0,
        actions = [imu_complementary_filter],
    )


    #joystick = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([os.path.join(
    #        get_package_share_directory("teleop_twist_joy"),'launch','teleop-launch.py'
    #    )]), launch_arguments={'config_filepath': '/home/vincent/joystick.yaml','joy_vel':'/omnidirectional_controller/cmd_vel_unstamped'}.items(),
    #    
    #)

    return LaunchDescription([
        rsp,
        delayed_controller_manager,
        delayed_omnidrive_spawner,
        delayed_joint_broad_spawner,
        imu_node,
        delayed_imu_filter,
        delayed_actuators_spawner,
        lidar_with_mgr,
        #joystick,
        
    ])