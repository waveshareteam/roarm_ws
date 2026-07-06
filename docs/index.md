# Roarm_ws Documentation

**roarm_ws** is a **ROS2 Humble** colcon workspace for **Waveshare RoArm ([roarm m2](https://www.waveshare.com/roarm-m2-s.htm) / [roarm m3](https://www.waveshare.com/roarm-m3.htm))**.
It connects **RViz2** and **MoveIt2** to real hardware **over serial**, and optionally to camera-based pick-place and Gazebo simulation.

### Product names vs environment variables

Waveshare labels and this repo use different names for the same hardware. There is **no** separate `ROARM_MODEL=roarm_m2_ga` — **GA** means **RoArm-M2 with the geared gripper**:

| Your hardware (Waveshare) | `ROARM_MODEL` | `GRIPPER_TYPE` |
|---------------------------|---------------|----------------|
| RoArm-M2 (direct-drive gripper) | `roarm_m2` | `angular_direct` |
| RoArm-M2-**GA** / M2 geared gripper | `roarm_m2` | `angular_gear` |
| RoArm-M3 | `roarm_m3` | `angular_direct` or `angular_gear` *(either — launches require it; M3 URDF is unchanged)* |

**Click an image for full-screen view** — click outside, press **Esc**, or **×** to close.

<div class="img-row img-row-3 img-row-equal img-row-h-sm">

<figure>
<img class="img-zoom" alt="RoArm-M2 direct-drive gripper" src="https://github.com/user-attachments/assets/ba61a6d4-547a-453b-9d1e-81bce8445c07" />
<figcaption>RoArm-M2<br/><code>roarm_m2</code> + <code>angular_direct</code></figcaption>
</figure>

<figure>
<img class="img-zoom" alt="RoArm-M2-GA geared gripper" src="https://github.com/user-attachments/assets/e1952ed6-943f-4590-b04c-fe65e742c297" />
<figcaption>RoArm-M2-GA<br/><code>roarm_m2</code> + <code>angular_gear</code></figcaption>
</figure>

<figure>
<img class="img-zoom" alt="RoArm-M3" src="https://github.com/user-attachments/assets/e55a4084-3b64-4a07-a3cb-644f241f0e66" />
<figcaption>RoArm-M3<br/><code>roarm_m3</code></figcaption>
</figure>

</div>

On **M2-GA**, set **`GRIPPER_TYPE=angular_gear`** so RViz mesh, solver real TCP, and pick-place match the geared fingers ([RoArm Basics — angular_gear](roarm_basics.md#angular_gear-tcp-moves-with-gripper), [Gripper configuration](description.md#gripper-configuration)).

**Getting started** — read in this order ([details below](#suggested-reading-order)):

1. **[ROS2 Basics](ros2_basics.md)** — if you are new to ROS2 (topics, services, launch files used in this repo). Skip if you already know ROS2.
2. **[RoArm Basics](roarm_basics.md)** — frames and TCP (`hand_tcp` vs real fingertip).
3. **[Installation](installation.md)** — build and configure **`roarm_ws`** (`build_first.sh`, `ROARM_MODEL`, `GRIPPER_TYPE`).

Before use, set environment variables (via `build_first.sh` or `~/.bashrc`):

| Variable | Values | When | Role |
|----------|--------|------|------|
| `ROARM_MODEL` | `roarm_m2`, `roarm_m3` | **Always** — every launch | URDF, MoveIt config, driver joint layout; must match your hardware |
| `GRIPPER_TYPE` | `angular_direct`, `angular_gear` | **Always** — launches read this env (including roarm_m3) | **roarm_m2 only** — **`angular_gear` = M2-GA** geared gripper ([naming table above](#product-names-vs-environment-variables)); affects URDF mesh and solver real TCP ([RoArm Basics](roarm_basics.md), [gripper config](description.md#gripper-configuration)). roarm_m3: value required but does not change the model |
| `GZ_VERSION` | `classic`, `harmonic` | **Gazebo only** | Simulator backend for [Gazebo](gazebo.md); omit if you skipped Gazebo in `build_first.sh` (real arm / Pi) |

After install or editing `~/.bashrc`, verify:

```bash
echo $ROARM_MODEL $GRIPPER_TYPE $GZ_VERSION
```

`GZ_VERSION` may print empty when you do not use simulation — that is normal.

To change model or gripper later, edit `~/.bashrc` and `source ~/.bashrc`; you usually do not need to re-run full `build_first.sh` ([Installation — change model](installation.md#change-model-or-gripper)).

---

## Overview

Short map of what each package in **`roarm_ws`** does. Step-by-step tutorials are in the sidebar.

### [1. Robot Description](description.md)

URDF/xacro models, RViz configs, joint sliders.

- **`roarm_description`** — xacro for `roarm_m2` and `roarm_m3`, optional camera mounts.
- **`display.launch.py`** — `joint_state_publisher_gui` + `robot_state_publisher` + RViz; sliders publish `/joint_states`.

### [2. Hardware Driver](driver_control.md)

Serial bridge to the physical arm (ESP32 via **`roarm-sdk`**).

- **`roarm_driver`** — subscribes to `/joint_states` and `/led_ctrl`; set `serial_port` to your device (e.g. `/dev/ttyUSB0` on PC).

On a real arm: **joint sliders** (`display.launch.py`) publish **`/joint_states`** directly; **MoveIt / Servo / Cmd** go through **`ros2_control` → `/joint_states` → `roarm_driver` → ESP32**.

### [3. MoveIt2](moveit2.md)

Motion planning and RViz drag-and-plan.

- **`roarm_moveit`** + **`roarm_moveit_ikfast_plugins`** — per-model SRDF, IKFast; group **`hand`**, frame **`hand_tcp`** ([RoArm Basics](roarm_basics.md)).
- **`roarm_moveit.launch.py`** — Motion Planning: drag **`hand_tcp`**, **Plan & Execute** (`roarm_driver` + `ros2_control` on hardware).

### [4. Keyboard & Gamepad Control](keyboard_control.md)

MoveIt Servo teleoperation.

- **`roarm_moveit_servo`** + vendored **`moveit_servo`** — trajectories to `hand_controller`.
- **`servo_control.launch.py`** — MoveIt + Servo + gamepad (`joy_node`).
- **`keyboardcontrol`** — keyboard jogging in a second terminal (`ros2 run roarm_moveit_servo keyboardcontrol`).

### [5. Command Control](command_control.md)

Services through MoveIt, poses use solver **real TCP** in `base_link` ([RoArm Basics](roarm_basics.md)).

- **`roarm_moveit_cmd`** — `roarmserver` motion services; **`setgrippercmd`** bridges **`/gripper_cmd`** to the gripper controller.
- **`roarm_msgs`** — service types; 
  - `/get_pose_cmd` — get current end-effector pose
  - `/move_joint_cmd` — move to pose (roarm_m3: x/y/z/roll/pitch/yaw + `gripper`; roarm_m2: x/y/z + `gripper`)
  - `/move_line_cmd` — straight-line path (roarm_m2 / roarm_m3; roarm_m3 holds roll/pitch during the move)
  - `/move_circle_cmd` — arc via intermediate pose (`x0,y0,z0` → `x1,y1,z1`) + `gripper`
- **`command_control.launch.py`** — MoveIt + `roarmserver` (+ optional RViz).

### [6. MoveIt Task Constructor Demo](mtc_demo.md)

Multi-stage pick/place and Cartesian demos.

- **`roarm_moveit_mtc_demo`** (+ **`moveit_task_constructor`** in `roarm_else`).
- **`demo.launch.py`** + **`run.launch.py exe:=<name>`** — plan in Terminal 2, **Exec** in RViz.
- Executables: `cartesian`, `cartesian_modular`, `pick_place`.

### [7. Vision & Pick / Place](vision.md)

USB perception → TF → **`/pick_place_cmd`** on the real arm. T1: **`command_control.launch.py use_rviz:=true`** (recommended for debugging) + T2: `demo.launch.py`.

### [8. Gazebo Simulation](gazebo.md)

- **`roarm_gazebo`** — Gazebo Classic or GZ Harmonic (`GZ_VERSION`); no `roarm_driver`.
- **`bringup_gazebo.launch.py`** — world + spawn + `ros2_control` (+ optional RViz).
- **`moveit_gazebo.launch.py`** — bringup + **`move_group`** for drag-and-plan in sim.

---

## Typical paths

Use separate terminals for **real-arm** workflows below. Set **`serial_port`** on the driver to your device (e.g. `/dev/ttyUSB0` on PC). **MoveIt / Servo / Cmd / Vision** keep **`roarm_driver` in Terminal 0**; **joint sliders** start **`display.launch.py` first**, then the driver.

| Goal | Commands |
|------|----------|
| Joint sliders on real arm | **T0:** `ros2 launch roarm_description display.launch.py use_rviz:=true` <br> **T1:** `ros2 run roarm_driver roarm_driver --ros-args -p serial_port:=/dev/ttyUSB0` |
| Drag-and-plan in RViz | **T0:** driver (as above) · <br>**T1:** `ros2 launch roarm_moveit roarm_moveit.launch.py use_rviz:=true` |
| Keyboard / gamepad | **T0:** driver · <br>**T1:** `ros2 launch roarm_moveit_servo servo_control.launch.py use_rviz:=true` · <br>**T2 (keyboard only):** `ros2 run roarm_moveit_servo keyboardcontrol` |
| CLI motion | **T0:** driver · <br>**T1:** `ros2 launch roarm_moveit_cmd command_control.launch.py use_rviz:=true` + service calls |
| Vision pick-place | **T0:** driver · <br>**T1:** `ros2 launch roarm_moveit_cmd command_control.launch.py use_rviz:=true add_camera:=true` · <br>**T2:** `ros2 launch roarm_vision demo.launch.py exe:=apriltag_detect base_frame:=base_link` + `/pick_place_cmd` |
| Simulation only | `ros2 launch roarm_gazebo bringup_gazebo.launch.py use_rviz:=true` or `moveit_gazebo.launch.py` |

---

## Suggested reading order

| Step | Page | You learn |
|------|------|-----------|
| 1 | [ROS2 Basics](ros2_basics.md) | ROS2 vocabulary (skip if you already know ROS2) |
| 2 | [RoArm Basics](roarm_basics.md) | Frames, `hand_tcp`, real TCP |
| 3 | [Installation](installation.md) | Build **`roarm_ws`**, set env vars |
| 4 | [Robot Description](description.md) | Model, joints, TF diagrams |
| 5 | [Hardware Driver](driver_control.md) | USB serial, `roarm_driver` |
| 6 | [MoveIt2](moveit2.md) | Plan & Execute on hardware |
| 7+ | Keyboard, Command, MTC, … | See sidebar |
