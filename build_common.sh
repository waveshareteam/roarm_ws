#!/bin/bash
set -e

WS=/home/ws/roarm_ws
cd $WS || exit 1

PACKAGES=(
  moveit_servo
  rviz_marker_tools
  moveit_task_constructor_msgs
  moveit_task_constructor_core
  moveit_task_constructor_capabilities
  moveit_task_constructor_visualization
  gazebo_ros2_control
  roarm_msgs
  roarm_moveit_cmd
  roarm_moveit_ikfast_plugins
  roarm_moveit_mtc_demo
  roarm_moveit_servo
  roarm_description
  roarm_driver
  roarm_moveit
  roarm_vision
)

echo "=============================="
echo "  Select packages to build"
echo "=============================="

for i in "${!PACKAGES[@]}"; do
  printf "[%2d] %s\n" $((i+1)) "${PACKAGES[$i]}"
done

echo
read -p "Please enter the package number to be compiled (space-separated): " SELECTION

SELECTED_PKGS=""

for index in $SELECTION; do
  pkg="${PACKAGES[$((index-1))]}"
  if [ -n "$pkg" ]; then
    SELECTED_PKGS="$SELECTED_PKGS $pkg"
  else
    echo "❌ Invalid number: $index"
    exit 1
  fi
done

echo
echo "✔ The following packages will be compiled.:"
echo "$SELECTED_PKGS"
echo

colcon build \
  --packages-select $SELECTED_PKGS \
  --symlink-install \
  --executor sequential

echo
echo "===== Build finished ====="
source install/setup.bash
echo "✔ Workspace sourced."
