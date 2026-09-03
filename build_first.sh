#!/bin/bash
set -e

WS=/home/ws/roarm_ws
BASHRC=~/.bashrc
APT_PREFS_BLOCK_CLASSIC=/etc/apt/preferences.d/block-gazebo-classic

add_if_not_exist () {
    grep -qxF "$1" "$BASHRC" || echo "$1" >> "$BASHRC"
}

set_gz_version_env () {
    local ver="$1"
    sed -i '/^export GZ_VERSION=/d' "$BASHRC"
    echo "export GZ_VERSION=${ver}" >> "$BASHRC"
    export GZ_VERSION="${ver}"
}

remove_installed_pkgs () {
    local pattern="$1"
    local pkgs
    pkgs=$(dpkg -l 2>/dev/null | awk -v p="$pattern" '$1 ~ /^ii/ && $2 ~ p {print $2}' || true)
    if [ -n "$pkgs" ]; then
        echo "🧹 Removing packages matching /$pattern/:"
        echo "$pkgs"
        # shellcheck disable=SC2086
        sudo apt-get remove -y --purge $pkgs || true
    else
        echo "✔ No packages matching /$pattern/"
    fi
}

purge_gazebo_harmonic_stack () {
    echo "🧹 Purging Gazebo Harmonic / ros-gzharmonic..."
    remove_installed_pkgs '^ros-humble-ros-gzharmonic'
    remove_installed_pkgs '^ros-humble-gz-ros2-control'
    remove_installed_pkgs '^gz-harmonic'
    remove_installed_pkgs '^gz-sim'
    remove_installed_pkgs '^gz-tools'
    remove_installed_pkgs '^gz-plugin'
    remove_installed_pkgs '^libgz-'
}

purge_gazebo_fortress_ros_gz () {
    echo "🧹 Purging Fortress ros-gz / ignition gazebo6..."
    remove_installed_pkgs '^ros-humble-ros-gz-'
    remove_installed_pkgs '^ros-humble-ros-gz$'
    remove_installed_pkgs '^ros-humble-ign-ros2-control'
    remove_installed_pkgs '^libignition-gazebo6'
    remove_installed_pkgs '^libignition-gui6'
    remove_installed_pkgs '^libignition-rendering6'
    remove_installed_pkgs '^libignition-sensors6'
    sudo apt-get remove -y --purge ignition-tools ignition-transport11-cli || true
}

purge_gazebo_classic_stack () {
    echo "🧹 Purging Gazebo Classic..."
    remove_installed_pkgs '^gazebo'
    remove_installed_pkgs '^libgazebo'
    remove_installed_pkgs '^ros-humble-gazebo'
}

ensure_universe () {
    sudo apt-get install -y software-properties-common
    sudo add-apt-repository -y universe || true
    sudo apt-get update
}

reload_bashrc () {
    # shellcheck disable=SC1090
    source "$BASHRC" 2>/dev/null || true

    # 非交互脚本里 bashrc 可能直接 return，再强制加载关键变量
    while IFS= read -r line; do
        eval "$line"
    done < <(grep -E '^export (GZ_VERSION|UGV_MODEL|LDLIDAR_MODEL|ROARM_MODEL|GRIPPER_TYPE)=' "$BASHRC" 2>/dev/null || true)

    echo "✔ Reloaded env from ~/.bashrc (GZ_VERSION=${GZ_VERSION:-none})"
}

echo "=============================="
echo "   ROARM Build & Config Script"
echo "=============================="
echo

# ---------- Basic system deps ----------
echo "[1/7] Installing basic dependencies..."
sudo apt-get update
sudo apt-get install -y \
  net-tools \
  python3-pip \
  python3-colcon-argcomplete \
  screen \
  gstreamer1.0-rtsp \
  software-properties-common \
  curl \
  lsb-release \
  gnupg

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
# 用 desktop，避免 desktop-full 乱拉 Gazebo 依赖
sudo apt-get install -y \
    ros-dev-tools \
    ros-humble-desktop \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-generate-parameter-library \
    ros-humble-moveit \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-py-binding-tools \
    ros-humble-v4l2-camera \
    ros-humble-depthai-bridge \
    ros-humble-depthai-ros-driver \
    ros-humble-depthai-ros-msgs \
    ros-humble-depthai-descriptions \
    ros-humble-depthai-examples \
    ros-humble-depthai \
    ros-humble-depthai-filters \
    ros-humble-depthai-ros || true

sudo -E apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gripper-controllers \
  ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-control-toolbox

# depthai dbgsym 可选，失败不阻断
sudo apt-get install -y \
    ros-humble-depthai-bridge-dbgsym \
    ros-humble-depthai-ros-msgs-dbgsym \
    ros-humble-depthai-ros-driver-dbgsym \
    ros-humble-depthai-dbgsym \
    ros-humble-depthai-examples-dbgsym \
    ros-humble-depthai-filters-dbgsym 2>/dev/null || true

# ---------- Gazebo (OPTIONAL) ----------
echo
echo "=============================="
echo "     Gazebo Version Select"
echo "=============================="
echo "⚠️  Gazebo is resource-intensive"
echo "⚠️  Classic / Harmonic / Fortress ros-gz 不能混装"
echo
echo "Select Gazebo version to install:"
echo "  [1] Gazebo Classic (gazebo11)"
echo "  [2] Gazebo Harmonic (gz-sim 8)  ← ros-humble-ros-gzharmonic"
echo "  [0] Skip Gazebo installation"
echo
echo "Note: Humble official ros-humble-ros-gz* = Fortress (6)."
echo "      Harmonic must use ros-humble-ros-gzharmonic (conflicts with ros-gz*)."
echo

read -p "Your choice [0-2]: " GAZEBO_CHOICE

GAZEBO_INSTALLED=false
GZ_VERSION=""

case "$GAZEBO_CHOICE" in

  1)
    echo "✔ Installing Gazebo Classic (gazebo11)..."

    # 取消可能阻止 Classic 的 apt pin（例如 ugv 脚本留下的）
    sudo rm -f "$APT_PREFS_BLOCK_CLASSIC"

    # Classic 在 universe
    ensure_universe

    # 卸干净 Harmonic + Fortress
    purge_gazebo_harmonic_stack
    purge_gazebo_fortress_ros_gz
    sudo apt-get autoremove -y || true

    sudo apt-get install -y \
      gazebo \
      gazebo-common \
      gazebo-plugin-base \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-gazebo-ros2-control

    GZ_VERSION="classic"
    GAZEBO_INSTALLED=true

    set_gz_version_env "classic"
    add_if_not_exist "source /usr/share/gazebo/setup.bash"
    ;;

  2)
    echo "⚠️ Installing Gazebo Harmonic (gz-sim 8) for ROS 2 Humble..."
    echo "   Per https://gazebosim.org/docs/harmonic/ros_installation/"

    # Harmonic 时阻止 Classic 被其它包拉回来（可选）
    #cat > "$APT_PREFS_BLOCK_CLASSIC" << 'EOF'
    sudo tee "$APT_PREFS_BLOCK_CLASSIC" > /dev/null << 'EOF'
Package: gazebo
Pin: release *
Pin-Priority: -1

Package: gazebo-*
Pin: release *
Pin-Priority: -1

Package: libgazebo*
Pin: release *
Pin-Priority: -1

Package: ros-humble-gazebo*
Pin: release *
Pin-Priority: -1
EOF

    sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
https://packages.osrfoundation.org/gazebo/ubuntu-stable \
$(lsb_release -cs) main" \
      | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

    sudo apt-get update

    # 官方：与 ros-humble-ros-gz*（Fortress）冲突；同时卸 Classic
    purge_gazebo_classic_stack
    purge_gazebo_fortress_ros_gz
    sudo apt-get autoremove -y || true

    # 正确栈：不要装 ros-humble-ros-gz
    sudo apt-get install -y \
      gz-harmonic \
      ros-humble-ros-gzharmonic \
      ros-humble-gz-ros2-control

    GZ_VERSION="harmonic"
    GAZEBO_INSTALLED=true

    set_gz_version_env "harmonic"
    ;;

  0)
    echo "⏭ Skipped Gazebo installation"
    ;;

  *)
    echo "❌ Invalid choice, skipping Gazebo installation"
    ;;
esac

# ---------- ROS env ----------
echo
echo "[3/7] Configuring ROS environment..."
add_if_not_exist "source /opt/ros/humble/setup.bash"
# shellcheck disable=SC1090
source "$BASHRC" || true

# ---------- Model selection ----------
echo
echo "[4/7] Select ROARM model:"
select ROARM_MODEL in roarm_m2 roarm_m3; do
    [ -n "$ROARM_MODEL" ] && break
    echo "Invalid selection."
done

echo
echo "[5/7] Select GRIPPER type:"
select GRIPPER_TYPE in angular_direct angular_gear; do
    [ -n "$GRIPPER_TYPE" ] && break
    echo "Invalid selection."
done

echo
echo "Selected configuration:"
echo "  ROARM_MODEL     = $ROARM_MODEL"
echo "  GRIPPER_TYPE    = $GRIPPER_TYPE"
echo "  Gazebo          = $GAZEBO_INSTALLED"
echo "  GZ_VERSION      = ${GZ_VERSION:-none}"

read -p "Save model selection to ~/.bashrc? [y/N]: " SAVE_ENV
if [[ "$SAVE_ENV" =~ ^[Yy]$ ]]; then
    sed -i '/^export ROARM_MODEL=/d' "$BASHRC"
    sed -i '/^export GRIPPER_TYPE=/d' "$BASHRC"
    echo "export ROARM_MODEL=${ROARM_MODEL}" >> "$BASHRC"
    echo "export GRIPPER_TYPE=${GRIPPER_TYPE}" >> "$BASHRC"
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
# shellcheck disable=SC1091
source install/setup.bash

BUILD_PKGS=(
  moveit_servo
  rviz_marker_tools
  moveit_task_constructor_msgs
  moveit_task_constructor_core
  moveit_task_constructor_capabilities
  moveit_task_constructor_visualization
  roarm_moveit_cmd
  roarm_moveit_ikfast_plugins
  roarm_moveit_mtc_demo
  roarm_moveit_servo
)

if [ "$GZ_VERSION" = "classic" ]; then
  echo "✔ GZ_VERSION=classic → build gazebo_ros2_control from source (if present)"
  # 工作区有源码才编；没有则依赖 apt 的 ros-humble-gazebo-ros2-control
  if [ -d "$WS/src" ] && find "$WS/src" -maxdepth 4 -type d -name 'gazebo_ros2_control' | grep -q .; then
    BUILD_PKGS+=(gazebo_ros2_control)
  else
    echo "⏭ No source gazebo_ros2_control; using apt package"
  fi
elif [ "$GZ_VERSION" = "harmonic" ]; then
  echo "✔ GZ_VERSION=harmonic → use apt ros-humble-gz-ros2-control"
else
  echo "⏭ Skip gazebo control packages (GZ_VERSION='${GZ_VERSION:-none}')"
fi

colcon build \
  --packages-select "${BUILD_PKGS[@]}" \
  --symlink-install \
  --executor sequential

ROARM_PKGS=(
  roarm_description
  roarm_driver
  roarm_moveit
  roarm_vision
)

if [ "$GAZEBO_INSTALLED" = true ]; then
  ROARM_PKGS+=(roarm_gazebo)
else
  echo "⏭ Skip roarm_gazebo (Gazebo not installed)"
fi

colcon build \
  --packages-select "${ROARM_PKGS[@]}" \
  --symlink-install \
  --executor sequential

# ---------- Final env ----------
echo
echo "[7/7] Finalizing environment..."
add_if_not_exist "source $WS/install/setup.bash"
add_if_not_exist "export PULSE_SERVER=unix:/run/user/1000/pulse/native"
add_if_not_exist "export XDG_RUNTIME_DIR=/run/user/1000"
add_if_not_exist "# ---- ROS 2 & colcon argcomplete ----"

add_if_not_exist 'if [ -f /usr/share/ros2cli/ros2cli-completion.bash ]; then source /usr/share/ros2cli/ros2cli-completion.bash; fi'
add_if_not_exist 'if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi'

# shellcheck disable=SC1090
source "$BASHRC" || true

echo
echo "=============================="
echo "✔ Environment ready."
echo "✔ ROARM_MODEL=$ROARM_MODEL"
echo "✔ GRIPPER_TYPE=$GRIPPER_TYPE"
echo "✔ Gazebo installed: $GAZEBO_INSTALLED"
echo "✔ GZ_VERSION=${GZ_VERSION:-none}"
if [ "$GZ_VERSION" = "harmonic" ]; then
  echo "✔ Harmonic: gz-harmonic + ros-humble-ros-gzharmonic + ros-humble-gz-ros2-control"
  echo "  Launch tip: gz_sim.launch.py gz_version:=8"
elif [ "$GZ_VERSION" = "classic" ]; then
  echo "✔ Classic: gazebo11 + ros-humble-gazebo-ros-pkgs + gazebo-ros2-control"
fi
echo "=============================="

reload_bashrc
