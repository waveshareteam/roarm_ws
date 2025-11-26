cmake_minimum_required(VERSION 3.5)
project(roarm_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(MSG_FILES
  msg/ServoStatus.msg
)

set(SRV_FILES
  srv/GetPoseCmd.srv
  srv/MoveJointCmd.srv
  srv/MoveCircleCmd.srv
  srv/MoveLineCmd.srv
  srv/ServoCommandType.srv
  srv/PickPlaceCmd.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
