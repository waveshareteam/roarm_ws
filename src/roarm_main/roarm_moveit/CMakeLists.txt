cmake_minimum_required(VERSION 3.22)
project(roarm_moveit)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES  DESTINATION share/${PROJECT_NAME})
