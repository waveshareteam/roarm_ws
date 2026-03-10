cmake_minimum_required(VERSION 3.22)
project(roarm_moveit_cmd)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(roarm_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  sensor_msgs
  rclcpp
  rclcpp_action
  tf2_geometry_msgs
  tf2_ros
  moveit_ros_planning_interface
  Eigen3
  Boost
  geometry_msgs
  control_msgs
  roarm_msgs
)

# Include directories
include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${THIS_PACKAGE_INCLUDE_DIRS}
)

# Declare dependencies
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)

# Define executables
add_executable(getposecmd_tf 
    src/getposecmd_tf.cpp)
target_include_directories(getposecmd_tf 
   PUBLIC include)
ament_target_dependencies(getposecmd_tf 
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

add_executable(getposecmd_moveit
    src/getposecmd_moveit.cpp)
target_include_directories(getposecmd_moveit
   PUBLIC include)
ament_target_dependencies(getposecmd_moveit
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

add_executable(setgrippercmd 
    src/setgrippercmd.cpp)
target_include_directories(setgrippercmd 
    PUBLIC include)
ament_target_dependencies(setgrippercmd 
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
    
add_executable(roarmserver 
    src/roarmserver.cpp)
target_include_directories(roarmserver 
    PUBLIC include)
ament_target_dependencies(roarmserver 
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

# Install targets
install(TARGETS getposecmd_moveit getposecmd_tf setgrippercmd roarmserver DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

  
ament_package()


