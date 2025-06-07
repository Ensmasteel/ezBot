
exit
cd ezBot
source install/setup.bash
ros2 launch ezbot_robot real_robot.launch.py &
ros2 run joy joy_node &
ros2 run teleop_twist_joy teleop_node cmd_vel:=/omnidirectional_controller/cmd_vel_unstamped --ros-args -p require_enable_button:=false -p axis_linear.x:=1 -p axis_linear.y:=0 -p scale_linear.y:=0.5 -p enable_turbo_button:=5 -p scale_angular.yaw:=1.0 -p scale_angular_turbo.yaw:=1.5 -p scale_linear_turbo.y:=1.0
