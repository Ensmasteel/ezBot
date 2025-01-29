cmake_minimum_required(VERSION 3.16)
project(omnidrive_rppico LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  omnidrive_rppico
  SHARED
  hardware/omnibot_pico_system.cpp
)
target_compile_features(omnidrive_rppico PUBLIC cxx_std_17)
target_include_directories(omnidrive_rppico PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/omnidrive_rppico>
)
ament_target_dependencies(
  omnidrive_rppico PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(omnidrive_rppico PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DIFFDRIVE_ARDUINO_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface omnidrive_rppico.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/omnidrive_rppico
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/rviz
  DESTINATION share/omnidrive_rppico
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/omnidrive_rppico
)
install(TARGETS omnidrive_rppico
  EXPORT export_omnidrive_rppico
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_omnidrive_rppico HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
