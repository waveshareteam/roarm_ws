#!/bin/bash
set -e

WS=/home/ws/roarm_ws
BASHRC=~/.bashrc

add_if_not_exist () {
    grep -qxF "$1" "$BASHRC" || echo "$1" >> "$BASHRC"
}

echo "=============================="
echo "   ROARM Build & Config Script"
echo "=============================="
echo

# ---------- Basic system deps ----------
echo "[1/7] Installing basic dependencies..."
apt-get update
apt-get install -y \
  net-tools \
  python3-pip \
  python3-colcon-argcomplete \
  screen \
  gstreamer1.0-rtsp \

# ---------- Python deps (WARNING) ----------
echo
echo "⚠️  Python dependencies will be installed via pip"
echo "⚠️  It is STRONGLY recommended to use a virtualenv for AI/Vision"
read -p "Continue pip install requirements.txt? [y/N]: " PIP_CONFIRM
if [[ "$PIP_CONFIRM" =~ ^[Yy]$ ]]; then
    python3 -m pip install -r requirements.txt
else
    echo "⏭ Skipped pip install"
fi

# ---------- ROS 2 packages ----------
echo
echo "[2/7] Installing ROS 2 packages..."
apt-get install -y \
    ros-dev-tools \
    ros-humble-desktop-* \
    ros-humble-joint-state-publisher-* \
    ros-humble-generate-parameter-library \
    ros-humble-moveit-* \
    ros-humble-rqt-* \
    ros-humble-py-binding-tools \
    ros-humble-v4l2-camera \
    ros-humble-depthai-bridge-dbgsym \
    ros-humble-depthai-ros-driver \
    ros-humble-depthai-ros-msgs \
    ros-humble-depthai-ros-msgs-dbgsym \
    ros-humble-depthai-bridge \
    ros-humble-depthai-descriptions \
    ros-humble-depthai-examples \
    ros-humble-depthai-ros-driver-dbgsym \
    ros-humble-depthai \
    ros-humble-depthai-dbgsym \
    ros-humble-depthai-examples-dbgsym \
    ros-humble-depthai-filters \
    ros-humble-depthai-filters-dbgsym \
    ros-humble-depthai-ros


# ---------- Gazebo (OPTIONAL) ----------
echo
echo "=============================="
echo "     Gazebo Version Select"
echo "=============================="
echo "⚠️  Gazebo is resource-intensive"
echo "⚠️  Recommended ONLY for desktop / VM"
echo
echo "Select Gazebo version to install:"
echo "  [1] Gazebo Classic (gazebo11)"
echo "  [2] Gazebo Harmonic (gz-sim)"
echo "  [0] Skip Gazebo installation"
echo

read -p "Your choice [0-2]: " GAZEBO_CHOICE

GAZEBO_INSTALLED=false
GZ_VERSION=""

case "$GAZEBO_CHOICE" in

  # ---------- Gazebo Classic ----------
  1)
    echo "✔ Installing Gazebo Classic (gazebo11)..."

    apt-get install -y \
      gazebo \
      gazebo-common \
      gazebo-plugin-base \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-gazebo-ros2-control

    GZ_VERSION="classic"
    GAZEBO_INSTALLED=true

    # Gazebo Classic environment
    add_if_not_exist "source /usr/share/gazebo/setup.bash"
    ;;

  # ---------- Gazebo Harmonic ----------
  2)
    echo "⚠️ Installing Gazebo Harmonic (gz-sim)..."

    # OSRF repository (only needed for gz)
    apt-get install -y \
      curl \
      lsb-release \
      gnupg

    curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
https://packages.osrfoundation.org/gazebo/ubuntu-stable \
$(lsb_release -cs) main" \
      | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

    apt-get update

    apt-get install -y \
      gz-harmonic \
      ros-humble-ros-gzharmonic

    GZ_VERSION="harmonic"
    GAZEBO_INSTALLED=true
    ;;

  # ---------- Skip ----------
  0)
    echo "⏭ Skipped Gazebo installation"
    ;;

  *)
    echo "❌ Invalid choice, skipping Gazebo installation"
    ;;
esac

# ---------- Export Gazebo version ----------
if [ "$GAZEBO_INSTALLED" = true ]; then
  add_if_not_exist "export GZ_VERSION=${GZ_VERSION}"
fi

# ---------- ROS env ----------
echo
echo "[3/7] Configuring ROS environment..."
add_if_not_exist "source /opt/ros/humble/setup.bash"
source ~/.bashrc

# ---------- Model selection ----------
echo
echo "[4/7] Select ROARM model:"
select ROARM_MODEL in roarm_m2 roarm_m3; do
    [ -n "$ROARM_MODEL" ] && break
    echo "Invalid selection."
done

echo

echo
echo "[5/7] Select GRIPPER type:"
select GRIPPER_TYPE in angular_direct angular_gear; do
    [ -n "$GRIPPER_TYPE" ] && break
    echo "Invalid selection."
done

echo

echo
echo "Selected configuration:"
echo "  ROARM_MODEL     = $ROARM_MODEL"
echo "  GRIPPER_TYPE    = $GRIPPER_TYPE"
echo "  Gazebo        = $GAZEBO_INSTALLED"

read -p "Save model selection to ~/.bashrc? [y/N]: " SAVE_ENV
if [[ "$SAVE_ENV" =~ ^[Yy]$ ]]; then
    add_if_not_exist "export ROARM_MODEL=$ROARM_MODEL"
    add_if_not_exist "export GRIPPER_TYPE=$GRIPPER_TYPE"
    echo "✔ Model selection saved to ~/.bashrc"
else
    export ROARM_MODEL
    export GRIPPER_TYPE
    echo "✔ Model selection exported for current shell only"
fi

# ---------- Build ----------
echo
echo "[6/7] Building workspace: $WS"
cd "$WS" || exit 1

colcon build --packages-select roarm_msgs --symlink-install
source install/setup.bash

colcon build \
  --packages-select \
    moveit_servo \
    rviz_marker_tools \
    moveit_task_constructor_msgs moveit_task_constructor_core \
    moveit_task_constructor_capabilities moveit_task_constructor_visualization gazebo_ros2_control \
    roarm_moveit_cmd \
    roarm_moveit_ikfast_plugins \
    roarm_moveit_mtc_demo \
    roarm_moveit_servo \
  --symlink-install \
  --executor sequential

colcon build \
  --packages-select \
    roarm_description roarm_driver roarm_moveit roarm_vision roarm_gazebo \
  --symlink-install \
  --executor sequential

# ---------- Final env ----------
echo
echo "[7/7] Finalizing environment..."
add_if_not_exist "source $WS/install/setup.bash"
add_if_not_exist "export PULSE_SERVER=unix:/run/user/1000/pulse/native"
add_if_not_exist "export XDG_RUNTIME_DIR=/run/user/1000"
add_if_not_exist "# ---- ROS 2 & colcon argcomplete ----"

# ROS 2 CLI completion
add_if_not_exist 'if [ -f /usr/share/ros2cli/ros2cli-completion.bash ]; then source /usr/share/ros2cli/ros2cli-completion.bash; fi'
add_if_not_exist 'if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi'

source ~/.bashrc

echo
echo "=============================="
echo "✔ Environment ready."
echo "✔ ROARM_model=$ROARM_model"
echo "✔ GRIPPER_TYPE=$GRIPPER_TYPE"
echo "✔ Gazebo installed: $GAZEBO_INSTALLED"
echo "✔ GZ_VERSION=$GZ_VERSION"
echo "=============================="