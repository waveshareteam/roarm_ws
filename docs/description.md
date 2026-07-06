# Robot Description

This chapter introduces the **`roarm_description`** package: where the robot model files live, which environment variables select your arm, and how to view and move the model in RViz.

---

## Prerequisites

Before launching **`display.launch.py`**:

1. **Build and source** **`roarm_ws`** ([Installation](installation.md)).
2. Set **`ROARM_MODEL`** and **`GRIPPER_TYPE`** to match your hardware — see [Environment variables](installation.md#environment-variables). (`GRIPPER_TYPE` selects the roarm_m2 gripper mesh; on roarm_m3 either value is fine but must be set.)

---

## Package File Layout

```
roarm_description/
├── urdf/
│   ├── bases/
│   │   ├── roarm_m2.xacro      # roarm_m2 arm body + gripper include
│   │   └── roarm_m3.xacro      # roarm_m3 arm body + built-in gripper
│   ├── gripper/
│   │   ├── gripper.xacro       # roarm_m2 — angular_direct
│   │   └── gripper_a.xacro     # roarm_m2 — angular_gear (mimic finger joints)
│   ├── camera/                 # optional RGB camera mounts
│   ├── gazebo/                 # Gazebo plugins & transmissions
│   └── materials.xacro
├── meshes/
│   ├── roarm_m2/
│   ├── roarm_m3/
│   └── gripper/
├── launch/
│   └── display.launch.py       # joint_state_publisher_gui (RViz: use_rviz:=true)
└── rviz/
    └── view_description.rviz
```

| Path | Purpose |
|------|---------|
| `urdf/bases/*.xacro` | Main robot model loaded by `ROARM_MODEL` |
| `urdf/gripper/` | roarm_m2 gripper variant selected by `GRIPPER_TYPE` |
| `meshes/` | STL visual/collision geometry |
| `launch/display.launch.py` | Joint sliders + `robot_state_publisher`; RViz when **`use_rviz:=true`** |

MoveIt adds planning and simulation settings under `roarm_moveit/config/<ROARM_MODEL>/` using the **same link and joint names**.

---

## Environment Variables

Set during [Installation](installation.md) (`build_first.sh`) or in `~/.bashrc`:

```bash
echo $ROARM_MODEL $GRIPPER_TYPE
```

| Variable | Values | Effect |
|----------|--------|--------|
| `ROARM_MODEL` | `roarm_m2`, `roarm_m3` | Loads `urdf/bases/<model>.xacro` |
| `GRIPPER_TYPE` | `angular_direct`, `angular_gear` | **roarm_m2** — selects gripper xacro; **roarm_m3** — must be set for launches but does not change the model |

| Hardware | `ROARM_MODEL` | `GRIPPER_TYPE` |
|----------|---------------|----------------|
| RoArm-M2, direct gripper | `roarm_m2` | `angular_direct` |
| RoArm-M2-**GA** (geared gripper) | `roarm_m2` | `angular_gear` |
| RoArm-M3 | `roarm_m3` | `angular_direct` or `angular_gear` *(either — required by launches; does not change roarm_m3 URDF)* |

See also [index — Product names vs environment variables](index.md#product-names-vs-environment-variables).

**roarm_m3:** The gripper is built into `roarm_m3.xacro`. `GRIPPER_TYPE` does not switch meshes, but **`display.launch.py` and other launches still read it from the environment** — set either value during [Installation](installation.md) or in `~/.bashrc`.

If the roarm shape in RViz does not match your hardware, switch `ROARM_MODEL` and relaunch.

### Gripper Configuration

**roarm_m2 only.** `GRIPPER_TYPE` selects which gripper xacro is merged into the URDF. It affects:

1. **URDF / RViz / Gazebo** — gripper mesh and mimic joints (`gripper.xacro` vs `gripper_a.xacro`)
2. **Solver real TCP** — on `angular_gear`, fingertip offset changes with `gripper_joint` ([RoArm Basics](roarm_basics.md#real-tcp-the-hardware))

It does **not** change the hardware joint name — **`roarm_driver`** always reads **`gripper_joint`** on roarm_m2 and **`link5_to_gripper_link`** on roarm_m3.

**roarm_m3:** Gripper geometry is fixed in `roarm_m3.xacro`; `GRIPPER_TYPE` does not apply here [Environment variables](installation.md#environment-variables).

| `GRIPPER_TYPE` | Description | URDF |
|----------------|-------------|------|
| `angular_direct` | Direct angular drive — single jaw | `gripper.xacro` |
| `angular_gear` | Geared drive — multiple fingers (mimic joints in RViz) | `gripper_a.xacro` |

On **roarm_m2**, if the gripper shape in RViz does not match your hardware, switch `GRIPPER_TYPE` and relaunch.

---

## RViz Visualization

### RViz Navigation Controls

| Action | Mouse |
|--------|-------|
| Rotate view | Left-click + drag |
| Zoom | Scroll wheel / right-click + drag |
| Pan | Middle-click + drag |

### Joint State Publisher Gui Controls

| Control | Function |
|---------|----------|
| **Joint sliders** | One row per actuated joint. Value is in **radians**. Moving a slider immediately publishes a new `joint_states` message. |
| **Randomize** | Sets all sliders to random values within each joint’s URDF limits. **Do not use on a real arm** unless the area around the arm is clear. |

### Launch RViz Visualization

Launch the model with RViz and the joint slider window:

```bash
ros2 launch roarm_description display.launch.py use_rviz:=true
```

If the program no longer needs to run, please use **`Ctrl+C`** to close the running session.

---

### Starts Nodes

| Component | Role |
|-----------|------|
| `joint_state_publisher_gui` | Joint slider window (default **`gui:=true`**) |
| `joint_state_publisher` | Publishes `joint_states` without a GUI when **`gui:=false`** |
| `robot_state_publisher` | Publishes TF from URDF + `joint_states` (always started) |
| `rviz2` | 3D visualization when **`use_rviz:=true`** (default is `false`) |

Sliders publish on **`/joint_states`**. **`robot_state_publisher`** updates TF immediately; with **`use_rviz:=true`**, the RViz model follows the same angles.  
With **`roarm_driver`** running, the physical arm follows the same angles — see [Hardware Driver](driver_control.md).

!!! warning
    Use the **Joint State Publisher sliders in the GUI** to move the model — do **not** push or rotate the physical arm by hand while the driver is running. If **`roarm_driver` is active**, slider changes are sent to the motors immediately; keep the area around the arm clear before adjusting sliders.

---

**Data Transfer Process**

```
Joint State Publisher GUI  →  /joint_states  →  robot_state_publisher  →  TF  →  RViz model
                              ↓
                         roarm_driver (if running)  →  real arm
```

1. Drag a **slider** → the node publishes joint angles on **`/joint_states`** (`sensor_msgs/JointState`).
2. **`robot_state_publisher`** reads URDF + `joint_states` and updates **TF** so the RViz model moves.
3. If **`roarm_driver`** is running, it reads the same topic and sends angles to the hardware.

Only **revolute** joints from the URDF get a slider. **Fixed** and **mimic** joints do not appear. On **roarm_m2**, **`gripper_joint`** is the sole gripper slider (geared fingers are mimic joints). On **roarm_m3**, use **`link5_to_gripper_link`**.

---

## URDF with Xacro

RoArm models in **`roarm_ws`** are written as **Xacro** (XML macros), not a single flat URDF file.  
Launch files expand xacro at runtime using `ROARM_MODEL` and `GRIPPER_TYPE`, then pass the result to `robot_state_publisher`, RViz, MoveIt, and Gazebo.

Main entry files:

```text
roarm_description/urdf/bases/roarm_m2.xacro   # if ROARM_MODEL=roarm_m2
roarm_description/urdf/bases/roarm_m3.xacro   # if ROARM_MODEL=roarm_m3
```

`roarm_m2.xacro` includes gripper xacro via `<xacro:if value="${gripper_type == 'angular_direct'}">` / `angular_gear`. 

### Link

A **link** is a rigid body segment of the robot.

Example — `link1` in `roarm_m2.xacro`:

```xml
<link name="link1">
  <visual>
    <geometry>
      <mesh filename="file://$(find roarm_description)/meshes/roarm_m2/link1.stl" scale="1.0 1.0 1.0"/>
    </geometry>
  </visual>
  <collision>...</collision>
  <inertial>...</inertial>
</link>
```

Each link can define:

| Element | Purpose |
|---------|---------|
| `<visual>` | What you see in RViz (usually an STL mesh) |
| `<collision>` | Simplified geometry for planning / simulation |
| `<inertial>` | Mass and inertia (used by Gazebo) |

#### RoArm Links

**Click an image for full-screen view** — click outside, press **Esc**, or **×** to close.

##### roarm_m2

###### angular_direct

<img class="img-zoom" alt="roarm_m2 angular_direct link" src="https://github.com/user-attachments/assets/fb70c183-db77-4d64-97e8-df27b2e85dc9" />

---

###### angular_gear

**`angular_gear`** — adds finger links `gripper_left1` … `gripper_right3` (mimic joints; RViz only).

<img class="img-zoom" alt="roarm_m2 angular_gear link" src="https://github.com/user-attachments/assets/7d3a3378-8d1b-4f21-bd19-102edfecc97e" />

---

##### roarm_m3

<img class="img-zoom" alt="roarm_m3 angular_direct link" src="https://github.com/user-attachments/assets/bd3af4ac-ada3-4e3c-b06b-5a790e54eb76" />

---

### Joint

A **joint** connects a **parent link** to a **child link** and defines how they move relative to each other.

RoArm **mostly** uses **`parent_link_to_child_link`**, for example `link1_to_link2` connects `link1` → `link2`.  
Exceptions include **`gripper_joint`** on roarm_m2 (`gripper_base` → `gripper_link`) and fixed joints such as **`link3_to_hand_tcp`** / **`link5_to_hand_tcp`**.  
The same names appear in `/joint_states`, the Joint State Publisher sliders, and `roarm_driver`.

Example — shoulder joint in `roarm_m2.xacro`:

```xml
<joint name="link1_to_link2" type="revolute">
  <parent link="link1" />
  <child link="link2" />
  <origin xyz="0 0 0" rpy="-1.5708 -1.5708 0" />
  <axis xyz="0 0 1" />
  <limit lower="-1.5708" upper="1.5708" effort="0" velocity="0" />
</joint>
```

| XML field | Meaning |
|-----------|---------|
| `type` | How the joint moves (`revolute`, `fixed`, `mimic`, …) |
| `parent` / `child` | Which links this joint connects |
| `origin` | Pose of the child link in the parent frame |
| `axis` | Rotation axis (revolute joints) |
| `limit` | Min/max angle in **radians** |

Fixed joints have no slider and no entry in `/joint_states`.

Only **revolute** joints listed below are sent to the hardware through **`roarm_driver`**.

#### RoArm Joints

The same joint name is used in **URDF**, the **Joint State Publisher** sliders, **`/joint_states`**, and the **ESP32 firmware** (via `roarm_driver`).  

##### roarm_m2

Arm joints are identical for **`angular_direct`** and **`angular_gear`**. The geared gripper adds **mimic** finger joints in URDF only — the real arm still has four motors: base, shoulder, elbow, gripper.

| Joint (URDF) | Type | Connects (parent → child) | Real arm |
|--------------|------|-------------------------|----------|
| `base_link_to_link1` | revolute | `base_link` → `link1` | Base |
| `link1_to_link2` | revolute | `link1` → `link2` | Shoulder |
| `link2_to_link3` | revolute | `link2` → `link3` | Elbow |
| `gripper_joint` | revolute | `gripper_base` → `gripper_link` | EoAT |

**`angular_gear` only (URDF / RViz, not separate motors):** `gripper_left_joint1` … `gripper_right_joint3` — **mimic** `gripper_joint`.

<img class="img-zoom" alt="roarm_m2 angular_direct joint_remap"  src="https://github.com/user-attachments/assets/0da4f4c5-44fb-42cb-aaa3-efa420898fba" />

---

##### roarm_m3

| Joint (URDF) | Type | Connects (parent → child) | Real arm |
|--------------|------|-------------------------|----------|
| `base_link_to_link1` | revolute | `base_link` → `link1` | Base |
| `link1_to_link2` | revolute | `link1` → `link2` | Shoulder |
| `link2_to_link3` | revolute | `link2` → `link3` | Elbow |
| `link3_to_link4` | revolute | `link3` → `link4` | Wrist1 |
| `link4_to_link5` | revolute | `link4` → `link5` | Wrist2 |
| `link5_to_gripper_link` | revolute | `link5` → `gripper_link` | EoAT |

---

<img class="img-zoom" alt="roarm_m3 angular_direct joint_remap" src="https://github.com/user-attachments/assets/5e65d82a-ab28-40c1-ace7-2ad3746d7ebd" />

---

## TF Tree

The diagram below shows the **parent → child** chain from URDF.  

**Click an image for full-screen view** — click outside, press **Esc**, or **×** to close.

### roarm_m2

#### angular_direct

<img class="img-zoom" alt="roarm_m2 angular_direct TF tree" src="https://github.com/user-attachments/assets/41080e46-c435-4827-81cb-56cb40b2b85f" />

#### angular_gear

<img class="img-zoom" alt="roarm_m2 angular_gear TF tree" src="https://github.com/user-attachments/assets/89ac841e-848e-4421-a66f-4a6278409da1" />

### roarm_m3

<img class="img-zoom" alt="roarm_m3 TF tree" src="https://github.com/user-attachments/assets/d6bd262e-52c8-4341-9ead-1c01b214663e" />

---

For frame names, **`hand_tcp`** vs **real TCP**, and which tutorials use which — see **[RoArm Basics](roarm_basics.md)**.

---

## Demo Videos

### roarm_m2 

#### angular_direct

<video src="https://github.com/user-attachments/assets/8b7ef1ee-8cd9-4aae-9853-71dc8ffa59fd" controls width="500"></video>

---

#### angular_gear

<video src="https://github.com/user-attachments/assets/484064ea-c7b3-4591-b89b-7b2f59aba9de" controls width="500"></video>

---

### roarm_m3

<video src="https://github.com/user-attachments/assets/41367549-25dc-44ab-8a38-8fa46ed5b781" controls width="500"></video>

---

## Related Tutorials

| Goal | Go to |
|------|-------|
| Frames & TCP | [RoArm Basics](roarm_basics.md) |
| Control real arm over USB | [Hardware Driver](driver_control.md) |
| Drag end-effector with MoveIt | [MoveIt2](moveit2.md) |
| Simulation | [Gazebo](gazebo.md) |
