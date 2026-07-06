# ROS2 Basics

Even if you are not familiar with ROS2, you can follow the RoArm tutorial step by step.
This page lists **only the ROS2 concepts you will meet in `roarm_ws`**.

Already comfortable with ROS2? Skip to [RoArm Basics](roarm_basics.md). Already know ROS2 and RoArm frames/TCP? Skip to [Installation](installation.md).

---

## ROS2 Introduction

**ROS2** is a second-generation robot operating system designed and developed based on ROS. It is a software library and toolset that can help us simplify robot development tasks and accelerate the deployment of robots.

---

## ROS workspace & package

| Term | Meaning |
|------|---------|
| **ROS workspace** | The **`roarm_ws`** folder you build with `colcon` (packages, `install/`, `build/`) — not the arm’s physical surroundings |
| **Package** | One module inside **`roarm_ws`**, e.g. `roarm_driver`, `roarm_description` |
| **Node** | One running program, e.g. `roarm_driver` |
| **Launch file** | Starts several nodes with one command, e.g. `display.launch.py` |
| **Area around the arm** | The physical desk / clearance zone — keep it clear before motion (safety) |
| **Reach** | How far the tool can move in **`base_link`** — e.g. “target out of reach” in MoveIt |

After every **new** terminal, source **`roarm_ws`**:

```bash
source /opt/ros/humble/setup.bash
source /home/ws/roarm_ws/install/setup.bash
```

(`build_first.sh` can add these lines to `~/.bashrc` so new shells pick them up automatically.)

First-time build and env vars: [Installation](installation.md). After code changes, run `sudo bash build_common.sh` in the **`roarm_ws`** root (interactive package picker) — it sources **`install/setup.bash`** in **that shell** when finished; **other already-open terminals** still need the `source` commands above (or open a fresh terminal). See [Installation — rebuild](installation.md#rebuild-after-code-changes).

---

## Nodes

A **node** is a fundamental ROS2 element that serves a single, modular purpose in a robotics system. (e.g. the driver, RViz).

Start a node:

```bash
ros2 run <package_name> <executable_name> [args]
```

Example:

```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=false
```

`use_sim_time:=false` is a **parameter** passed to the node.

List active nodes:

```bash
ros2 node list
```

View node information:

```bash
ros2 node info <node_name>
```

Examples you will see in tutorials:

| Node | Package | Role |
|------|---------|------|
| `roarm_driver` | `roarm_driver` | Serial link to real arm |
| `robot_state_publisher` | ROS standard | URDF + joint angles → TF |
| `joint_state_publisher_gui` | ROS standard | Slider window |
| `rviz2` | RViz | 3D visualization |
| `move_group` | MoveIt | Motion planning |

---

## Launch files

**Launch files** allow you to start up and configure a number of executables containing ROS2 nodes simultaneously.

Running a single launch file with the `ros2 launch` will start up your entire system - all nodes and their configurations - at once.

```bash
ros2 launch <package> <launch_file> [launch_arguments]
```

Examples:

```bash
ros2 launch roarm_description display.launch.py use_rviz:=true
```

Launch arguments (`use_rviz:=true`) configure behavior without editing code.

---

## Communication method

A full robotic system is comprised of many nodes working in concert. In ROS2, a single executable (C++ program, Python program, etc.) can contain one or more nodes. Each node can send and receive data from other nodes via topics, services, or parameters.

**Click an image for full-screen view** — click outside, press **Esc**, or **×** to close.

<img class="img-zoom" alt="Communication" src="https://github.com/user-attachments/assets/c3f90e68-331d-44e7-83f7-d406e742838d" />

### Topics

**Topics** are one of the most commonly used communication methods in ROS2, and topic communication adopts a publish-subscribe model.

**Click an image for full-screen view** — click outside, press **Esc**, or **×** to close.

<img class="img-zoom"  alt="Topics" src="https://github.com/user-attachments/assets/f5029848-90b8-4f73-b673-21cbc2f22c1c" />

List active topics: 

```bash
ros2 topic list
```

View topic information:

```bash
ros2 topic info <topic_name> --verbose
```

Pub topic:

```bash
ros2 topic pub <topic_name> <msg_type> '<args>'
```

The **args** argument is the actual data you’ll pass to the topic

Topics used in `roarm_ws`:

| Topic | Message type | Direction | Used for |
|-------|--------------|-----------|----------|
| `/joint_states` | `sensor_msgs/JointState` | sliders, or `ros2_control` (MoveIt/Servo/Cmd) → driver | Joint angles (radians) |
| `/led_ctrl` | `std_msgs/Float32` | you → driver | Gripper LED brightness 0–255 |
| `/gripper_cmd` | `std_msgs/Float32` | Cmd / Vision / Servo → `setgrippercmd` → gripper controller | Gripper opening (radians) |
| `/tf` | `tf2_msgs/TFMessage` | robot_state_publisher → RViz | Link positions (frames) |

---

### Services

**Services** are based on a call-and-response model versus the publisher-subscriber model of topics. While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.

A service is divided into a **client** and a **server**. The mobile apps we use daily can be considered clients, while the app server is the software's server. The client sends a request to the server, the server processes the request, and then returns the result to the client.

**Click an image for full-screen view** — click outside, press **Esc**, or **×** to close.

<img class="img-zoom"  alt="Services" src="https://github.com/user-attachments/assets/7db35241-45b6-4d2d-8b9a-a6183fc3acee" />

List active services: 

```bash
ros2 service list
```

View type of a service:

```bash
ros2 service type <service_name>
```

Call service:

```bash
ros2 service call <service_name> <service_type> <arguments>
```

RoArm services you will call from the terminal:

| Service | Package | Purpose |
|---------|---------|---------|
| `/get_pose_cmd` | `roarm_moveit_cmd` | Read current tool pose |
| `/move_joint_cmd` | `roarm_moveit_cmd` | Move to target pose |
| `/move_line_cmd` | `roarm_moveit_cmd` | Straight-line path (roarm_m2 / roarm_m3) |
| `/move_circle_cmd` | `roarm_moveit_cmd` | Arc path |
| `/pick_place_cmd` | `roarm_vision` | Vision pick or place |

Custom types live in **`roarm_msgs`** — that is why the service type looks like `roarm_msgs/srv/MoveJointCmd`.

---

## Parameters

Nodes have **parameters** to define their default configuration values.

List parameters:

```bash
ros2 param list
```

View parameters:

```bash
ros2 param get <node_name> <parameter_name>
```

Set parameters:

```bash
ros2 param set <node_name> <parameter_name> <value>
```

| Mechanism | Set where | Examples in RoArm |
|-----------|-----------|-------------------|
| **Launch argument** | `ros2 launch …` | `use_rviz:=true` |
| **Node parameter** | `ros2 run … --ros-args -p` | `serial_port:=/dev/ttyUSB0` |
| **Environment variable** | `~/.bashrc` / `build_first.sh` | `ROARM_MODEL`, `GRIPPER_TYPE`, `GZ_VERSION` (Gazebo only) — [environment variables on index](index.md) |

---

## Tools

### TF2

Every link on the robot has a **frame** (coordinate system). **TF** tracks how frames relate as joints move.

For RoArm frame names, `hand_tcp`, and the difference between planning TCP and real TCP, see [RoArm Basics](roarm_basics.md).

| Frame | Meaning |
|-------|---------|
| `world` / `base_link` | Scene / robot base |
| `link1` … `link3` | Arm links (roarm_m2) |
| `link4`, `link5` | Wrist links (roarm_m3 only) |
| `hand_tcp` | MoveIt / Servo tool frame (fixed on URDF; see TCP section) |

See [Robot Description — TF Tree](description.md#tf-tree) for diagrams.

Debug TF:

Find out if tf2 knows about our transform between **base_link** and **hand_tcp**.

```bash
ros2 run tf2_ros tf2_echo base_link hand_tcp
```

Get a graphical representation.

```bash
ros2 run tf2_tools view_frames
```

Open the generated `frames.pdf` in the current directory to inspect the TF tree.

---

### URDF

**URDF** (Unified Robot Description Format) is a file format for specifying the geometry and organization of robots in ROS. 

- **`URDF`** = robot description (links + joints). Built from **xacro** in `roarm_description`.
- **`joint_states`** = current angle of each movable joint.
- **`robot_state_publisher`** = combines URDF + `joint_states` → TF for RViz.

Full RoArm-specific explanation: [Robot Description](description.md).

---

### RViz

**RViz** is a 3D visualizer for the Robot Operating System (ROS) framework. RViz shows the robot model, TF, cameras, and MoveIt interactive markers.

You do not “program” in RViz — you **watch** state and **click** Plan & Execute in MoveIt tutorials.

Common fixes:

- Empty view → set **Fixed Frame** (e.g. `base_link`)
- No robot → check `joint_states` and `robot_state_publisher` are running
- MoveIt marker missing → add **MotionPlanning** display

---

## Related Tutorials

| Chapter | What it adds |
|---------|----------------|
| [ROS2 Basics](ros2_basics.md) | ROS2 words, TF tools (this page) |
| [RoArm Basics](roarm_basics.md) | Frames, `hand_tcp`, real TCP |
| [Installation](installation.md) | Install & build |
| [Robot Description](description.md) | Model, joints, TF diagrams |
| [Hardware Driver](driver_control.md) | USB + driver node |
| [MoveIt2](moveit2.md) | Motion planning |
| Further tutorials | Servo, services, vision, sim — see [index](index.md#suggested-reading-order) |

---

## Learn more (official)

General ROS2 and MoveIt references (not required for the RoArm tutorials):

- [ROS2 Humble tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [MoveIt2 documentation](https://moveit.picknik.ai/humble/index.html)

This repo targets **ROS2 Humble** on **Ubuntu 22.04** (RViz, real arm or Gazebo, MoveIt2, CLI services).

**Next:** [RoArm Basics](roarm_basics.md) — frames, `hand_tcp`, and real TCP on RoArm hardware.