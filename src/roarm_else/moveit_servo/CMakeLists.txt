cmake_minimum_required(VERSION 3.22)
project(moveit_servo LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    control_msgs
    geometry_msgs
    moveit_core
    moveit_msgs
    roarm_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    rclcpp
    rclcpp_components
    realtime_tools
    sensor_msgs
    std_msgs
    std_srvs
    tf2_eigen
    trajectory_msgs)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

# ##############################################################################
# C++ Libraries ##
# ##############################################################################

# This library provides a way of loading parameters for servo
generate_parameter_library(moveit_servo_lib_parameters
                           config/servo_parameters.yaml)

# This library provides a C++ interface for sending realtime twist or joint
# commands to a robot
add_library(
  moveit_servo_lib_cpp SHARED src/collision_monitor.cpp src/servo.cpp
                              src/utils/common.cpp src/utils/command.cpp src/utils/logger.cpp)
set_target_properties(moveit_servo_lib_cpp PROPERTIES VERSION
                                                      "${moveit_servo_VERSION}")
target_link_libraries(moveit_servo_lib_cpp moveit_servo_lib_parameters)
ament_target_dependencies(moveit_servo_lib_cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(moveit_servo_lib_ros SHARED src/servo_node.cpp)
set_target_properties(moveit_servo_lib_ros PROPERTIES VERSION
                                                      "${moveit_servo_VERSION}")
target_link_libraries(moveit_servo_lib_ros moveit_servo_lib_cpp)
ament_target_dependencies(moveit_servo_lib_ros ${THIS_PACKAGE_INCLUDE_DEPENDS})

# ##############################################################################
# Components ##
# ##############################################################################

rclcpp_components_register_node(moveit_servo_lib_ros PLUGIN
                                "moveit_servo::ServoNode" EXECUTABLE servo_node)

# ##############################################################################
# Install ##
# ##############################################################################

# Install Libraries
install(
  TARGETS moveit_servo_lib_cpp moveit_servo_lib_ros moveit_servo_lib_parameters
  EXPORT moveit_servoTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_servo)

# Install Binaries
install(
  TARGETS servo_node 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/moveit_servo)

# Install include, launch, config directories
install(DIRECTORY include/ DESTINATION include/moveit_servo)
install(DIRECTORY launch DESTINATION share/moveit_servo)
install(DIRECTORY config DESTINATION share/moveit_servo)

ament_export_targets(moveit_servoTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})



ament_package()
