# Installation

Install **roarm_ws** on **Ubuntu 22.04** with **ROS2 Humble**.

You do **not** install ROS2 and MoveIt package-by-package by hand. After adding the **ROS2 apt repository**, run **`build_first.sh` once** — it installs dependencies, sets `ROARM_MODEL` / `GRIPPER_TYPE` (and optional `GZ_VERSION`), and runs `colcon build`.

**Raspberry Pi**: use Ubuntu 22.04 on the board, add the ROS2 source, clone the repo, run `build_first.sh`, and choose **Skip Gazebo** when prompted.

---

## System Requirements

If you use a VM, install Ubuntu 22.04 with enough disk space. On VirtualBox 7+, **uncheck Skip Unattended Installation** if you need normal sudo access.

**Pi or native Ubuntu:** skip the VM; go straight to [ROS workspace setup](#ros-workspace-setup).

---

## ROS workspace setup

```bash
sudo apt update
sudo apt install -y git
git clone -b ros2-humble-develop-251125 https://github.com/waveshareteam/roarm_ws.git
cd roarm_ws
```

Tutorials assume **`roarm_ws`** at **`/home/ws/roarm_ws`**. If you cloned elsewhere, move the folder or adjust paths in `~/.bashrc` later.

---

## ROS2 Apt Repository

Add the official ROS2 Humble source list (one-time on the machine):

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

Do **not** run `apt install ros-humble-desktop` yourself — **`build_first.sh` installs ROS2, MoveIt, vision-related packages, and build tools** in step **[2/7]**.

---

## Initial Setup with `build_first.sh`

From the **`roarm_ws`** root:

```bash
cd /home/ws/roarm_ws
sudo chmod +x build_first.sh
sudo bash build_first.sh
```

The script prints **`[1/7]` … `[7/7]`**. Overview:

| Script | What happens | You choose? |
|--------|----------------|-------------|
| **[1/7]** | Basic apt deps (net-tools, pip, colcon-argcomplete, screen, GStreamer RTSP, …) | — |
| *(after 1)* | Optional **`pip install -r requirements.txt`** | **y** / **N** |
| **[2/7]** | ROS2 Humble desktop, MoveIt, v4l2_camera, DepthAI ROS packages, build tools | — |
| *(after 2)* | **Gazebo** Classic / Harmonic / **Skip** | **0–2** |
| **[3/7]** | Append `source /opt/ros/humble/setup.bash` to `~/.bashrc` | — |
| **[4/7]** | **`ROARM_MODEL`** (`roarm_m2` / `roarm_m3`) | yes |
| **[5/7]** | **`GRIPPER_TYPE`** (`angular_direct` / `angular_gear`) | yes |
| *(prompt)* | Save model / gripper to **`~/.bashrc`?** | **y** / **N** |
| **[6/7]** | **`colcon build`** (ordered package groups) | — |
| **[7/7]** | `source …/install/setup.bash`, shell completion, finalize `~/.bashrc` | — |

The following sections describe each interactive step.

### Python Dependencies (Optional)

```
Continue pip install requirements.txt? [y/N]:
```

- **`y`** — installs `requirements.txt` at the repo root (`roarm-sdk`, OpenCV, AprilTag, DepthAI, etc.)
- **`N`** — skip (fine if you only need driver / MoveIt without vision)

### Gazebo Installation (Optional)

```
Select Gazebo version to install:
  [1] Gazebo Classic (gazebo11)
  [2] Gazebo Harmonic (gz-sim)
  [0] Skip Gazebo installation
Your choice [0-2]:
```

| Choice | When to use | Sets `GZ_VERSION` |
|--------|-------------|-------------------|
| **0** Skip | Raspberry Pi / low-resource board | *(not set)* |
| **1** Classic | Desktop / VM; [Gazebo Classic](gazebo.md) tutorials | `classic` |
| **2** Harmonic | `gz-sim` stack on Ubuntu 22.04 | `harmonic` |

Gazebo is heavy — skip unless you need simulation. If installed, the script appends `export GZ_VERSION=…` to `~/.bashrc` (Classic also adds `source /usr/share/gazebo/setup.bash`).

### Robot Model (`ROARM_MODEL`)

```
[4/7] Select ROARM model:
1) roarm_m2
2) roarm_m3
```

| Model | Arm | Notes |
|-------|-----|-------|
| **`roarm_m2`** | 3 joints + gripper | RoArm-M2 (direct) and RoArm-M2-**GA** (geared) — same `ROARM_MODEL`; GA uses **`GRIPPER_TYPE=angular_gear`** ([index](index.md#product-names-vs-environment-variables)) |
| **`roarm_m3`** | 5 joints + gripper | Extra wrist / roll |

This selects URDF, MoveIt config, driver joint mapping, and keyboard layout.

### Gripper Type (`GRIPPER_TYPE`)

```
[5/7] Select GRIPPER type:
1) angular_direct
2) angular_gear
```

| Type | Pick this if… |
|------|---------------|
| **`angular_direct`** | roarm_m2: motor **directly** drives the jaw. Loads `gripper.xacro`. |
| **`angular_gear`** | roarm_m2: **geared** fingers / linkage. Loads `gripper_a.xacro`. |

**roarm_m3:** `GRIPPER_TYPE` does not change the M3 URDF, but **launches still require it** — pick either value when prompted; `build_first.sh` can save it to `~/.bashrc`.

On **roarm_m2**, wrong choice → RViz gripper **mesh** does not match hardware (see [Gripper configuration](description.md#gripper-configuration)).

### Environment Variables

```
Save model selection to ~/.bashrc? [y/N]:
```

- **`y`** (recommended) — appends (if missing):
  ```bash
  export ROARM_MODEL=roarm_m2    # or roarm_m3
  export GRIPPER_TYPE=angular_direct   # or angular_gear
  ```
- **`N`** — exports only in the current shell (lost after logout)

Step **[7/7]** also appends when missing:

```bash
source /opt/ros/humble/setup.bash
source /home/ws/roarm_ws/install/setup.bash
export GZ_VERSION=classic   # only if Gazebo was installed
```

Plus ROS2 / colcon tab-completion. The script may add audio/runtime lines used by some vision streaming paths.

### Build with colcon

Build order (may take several minutes):

1. `roarm_msgs`
2. `moveit_servo`, MoveIt Task Constructor packages, `gazebo_ros2_control`, `roarm_moveit_cmd`, `roarm_moveit_ikfast_plugins`, `roarm_moveit_mtc_demo`, `roarm_moveit_servo`, …
3. `roarm_description`, `roarm_driver`, `roarm_moveit`, `roarm_vision`, `roarm_gazebo`

!!! note
    Some packages print stderr during `colcon build`; this is usually safe to ignore if the build ends with **Summary: N packages finished**.

---

## Post-Installation Verification

Open a **new terminal** or run:

```bash
source ~/.bashrc
echo $ROARM_MODEL $GRIPPER_TYPE $GZ_VERSION
```

`GZ_VERSION` empty is normal if you skipped Gazebo.

Full variable reference: [index — environment variables](index.md).

**Next:** [Robot Description](description.md) — model, joints, and RViz sliders (learning path step 4).

---

## Rebuild After Code Changes

From the **`roarm_ws`** root:

```bash
cd /home/ws/roarm_ws
sudo bash build_common.sh
```

Enter package numbers interactively (no model / Gazebo prompts). When the build finishes, the script **sources `install/setup.bash` in that terminal**; other already-open terminals still need `source ~/.bashrc` or a new shell.

---

## Change Model or Gripper

Edit `~/.bashrc` (or export in the current shell):

```bash
export ROARM_MODEL=roarm_m3
export GRIPPER_TYPE=angular_gear
source ~/.bashrc
```

No need to re-run full `build_first.sh` for **`ROARM_MODEL`** or **`GRIPPER_TYPE`** — edit `~/.bashrc` as above.

**Simulation (`GZ_VERSION`):** adding Gazebo or switching backend (Classic ↔ Harmonic) requires installing the matching **apt dependencies**. Re-run `build_first.sh` and select the Gazebo version you need; the script installs packages and can append `GZ_VERSION` to `~/.bashrc`. Setting `GZ_VERSION` alone, without the corresponding Gazebo stack installed, is not enough — see [Gazebo](gazebo.md).
