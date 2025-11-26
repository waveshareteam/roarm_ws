cmake_minimum_required(VERSION 3.22)
project(roarm_moveit_ikfast_plugins)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(LAPACK REQUIRED)
find_package(geometric_shapes REQUIRED)

include_directories(include ${geometric_shapes_INCLUDE_DIRS})

add_library(roarm_m2_hand_moveit_ikfast_plugin SHARED src/roarm_m2_hand_moveit_ikfast_plugin.cpp)
add_library(roarm_m3_hand_moveit_ikfast_plugin SHARED src/roarm_m3_hand_moveit_ikfast_plugin.cpp)

ament_target_dependencies(roarm_m2_hand_moveit_ikfast_plugin
  rclcpp
  moveit_core
  pluginlib
  tf2_kdl
  orocos_kdl
  tf2_eigen
  LAPACK
)

ament_target_dependencies(roarm_m3_hand_moveit_ikfast_plugin
  rclcpp
  moveit_core
  pluginlib
  tf2_kdl
  orocos_kdl
  tf2_eigen
  LAPACK
)

# suppress warnings about unused variables in OpenRave's solver code
target_compile_options(roarm_m2_hand_moveit_ikfast_plugin PRIVATE -Wno-unused-variable -Wno-unused-parameter)
target_compile_options(roarm_m3_hand_moveit_ikfast_plugin PRIVATE -Wno-unused-variable -Wno-unused-parameter)

install(TARGETS roarm_m2_hand_moveit_ikfast_plugin
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS roarm_m3_hand_moveit_ikfast_plugin
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(moveit_core roarm_m2_hand_moveit_ikfast_plugin.xml)
pluginlib_export_plugin_description_file(moveit_core roarm_m3_hand_moveit_ikfast_plugin.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core)
ament_export_dependencies(pluginlib)
ament_export_dependencies(rclcpp)
ament_export_dependencies(tf2_kdl)
ament_export_dependencies(tf2_eigen)
ament_package()

