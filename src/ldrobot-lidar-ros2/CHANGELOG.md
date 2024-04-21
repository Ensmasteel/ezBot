CHANGELOG
=========

2022-11-27
----------
* Add direct serial port connection feature

v0.2.0 - Humble
---------------
* Change license from MIT to Apache-2.0
* Code refactoring to meet [ROS2 rules](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
* Add CONTRIBUTING.md
* Add udev rules and install scripts
* Fix issue with Rviz2 config files
* Fix `nav2_lifecycle_manager` interaction in ROS2 Humble
  * Derive `ldlidar::LdLidarComponent` from `nav2_util::LifecycleNode` instead of `rclcpp::LifecycleNode` 
* Improve parameter handling
* Set default QoS to `rclcpp::SensorDataQoS`

v0.1.0
------
* ROS2 Foxy and Ubuntu 20.04
* First working version of the node based on ROS2 lifecycle architecture
* Python launch scripts
* Parameters customization
* Examples
