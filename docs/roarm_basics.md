# RoArm Basics

RoArm-specific concepts used in **`roarm_ws`**: **coordinate frames**, **`hand_tcp`**, and **TCP** (tool center point). This is not generic ROS2 material — see [ROS2 Basics](ros2_basics.md) for nodes, topics, and TF tools.

For URDF link/joint diagrams and TF tree images, see [Robot Description — TF Tree](description.md#tf-tree).

Already built **`roarm_ws`**? Skip to [Robot Description](description.md) or [Hardware Driver](driver_control.md).

---

## Frames and TF

A **frame** is a 3D coordinate system (origin + X/Y/Z). Each URDF **link** has one. **TF** publishes how frames move relative to each other as joints rotate.

```text
world → base_link → link1 → link2 → link3 → … → hand_tcp
```

| Frame | Role |
|-------|------|
| `world` | Fixed scene frame — common RViz **Fixed Frame** in MoveIt |
| `base_link` | Robot base on the desk; most poses are expressed relative to this |
| `link1` … `link3` | Arm links (roarm_m2) |
| `link4`, `link5` | Wrist links (roarm_m3 only) |
| `gripper_*` | Gripper meshes (`gripper_base`, fingers on **`angular_gear`**, etc.) |
| `hand_tcp` | Named **tool frame** for MoveIt and Servo |

`robot_state_publisher` reads `/joint_states` + URDF and broadcasts TF. RViz and MoveIt draw the arm from that tree.

### `base_link` axes (right-hand rule)

Most Cartesian commands in `roarm_ws` — Servo jog, `/move_joint_cmd` x/y/z, solver poses — are expressed in **`base_link`**. The frame follows the usual **right-hand rule** (same convention as ROS / RViz axis colors: **red = X**, **green = Y**, **blue = Z**):

**Click an image for full-screen view** — click outside, press **Esc**, or **×** to close.

<img class="img-zoom img-center" width="200" height="100" alt="right-hand rule" src="https://github.com/user-attachments/assets/2e4a131d-0831-4852-bd01-9f2012706ee2" />

<div class="img-row img-row-2 img-row-equal img-row-h-sm">

<figure>
<img class="img-zoom" alt="roarm_m2 axes" src="https://github.com/user-attachments/assets/4779154d-fdea-4cf8-85a1-058c3f37d3ad" />
<figcaption>roarm_m2</figcaption>
</figure>

<figure>
<img class="img-zoom" alt="roarm_m3 axes" src="https://github.com/user-attachments/assets/c23eb06a-86b3-429c-9ade-d17b589fc88a" />
<figcaption>roarm_m3</figcaption>
</figure>

</div>

---

## Two TCP concepts

**TCP** = the point you want to move — where the tool approaches or touches an object. A service call like `move_joint_cmd` with `x, y, z` means “move the TCP to that position in **`base_link`**”.

In `roarm_ws` there are two layers:

| | **`hand_tcp`** | **Real TCP** |
|---|----------------|--------------|
| **What** | Frame defined in URDF / MoveIt | Physical contact point on the hardware |
| **Defined in** | `roarm_description` (fixed joint) | Analytical solver in `roarm_moveit_cmd` (`solver.hpp`); **roarm_vision (roarm_solver)** for pick-place |
| **Moves when gripper opens?** | **No** — always fixed on the arm link | **Depends on `GRIPPER_TYPE`** (roarm_m2 only) |
| **Used by** | MoveIt drag marker, Servo frame, IKFast tip | `/get_pose_cmd`, `/move_*_cmd`, vision `/pick_place_cmd` |

On **roarm_m2 `angular_direct`**, the two coincide in practice. On **roarm_m2 `angular_gear`**, `hand_tcp` stays on `link3` but the real fingertip moves when `gripper_joint` changes.

---

### `hand_tcp` — planning frame (URDF / MoveIt)

`hand_tcp` is a **fixed** joint in URDF. It does **not** follow finger motion.

| Model | Parent link | URDF joint |
|-------|-------------|------------|
| **roarm_m2** | `link3` | `link3_to_hand_tcp` |
| **roarm_m3** | `link5` | `link5_to_hand_tcp` |

MoveIt group **`hand`** plans to `hand_tcp`. The RViz drag marker uses **`hand_tcp`**; in Servo, **`e`** jogs in **`hand_tcp`**, **`w`** in **`base_link`**.

Check the live transform:

```bash
ros2 run tf2_ros tf2_echo base_link hand_tcp
```

---

### Real TCP — the hardware

The solver computes XYZ in **`base_link`** from joint angles. For **roarm_m2**, the model depends on **`GRIPPER_TYPE`** (`echo $GRIPPER_TYPE`).

**Click an image for full-screen view** — click outside, press **Esc**, or **×** to close.

---

#### roarm_m2

##### angular_direct (fixed real TCP)

- Jaw rotates around `gripper_joint`, but the **contact point offset from `link3` is treated as fixed**.
- **`hand_tcp` ≈ real TCP** — RViz marker and physical tip stay aligned for pick-place.

<img class="img-zoom img-center" width="200" height="100" alt="roarm_m2 angular_direct tcp"  src="https://github.com/user-attachments/assets/5bc8d7f7-5d82-4b31-bd76-990fd8a13a03" />

---

##### angular_gear (TCP moves with gripper)

- Geared fingers change the **fingertip position** as `gripper_joint` opens or closes.
- Inverse kinematics also adjusts the effective forearm length when `gripper ≠ 0`.
- **`hand_tcp` ≠ real TCP** when the gripper is not fully closed — the RViz marker stays on `link3`, but the real fingers extend further or shorter.

| | `angular_direct` | `angular_gear` |
|---|------------------|----------------|
| RViz / MoveIt marker | Fixed on `link3` | Fixed on `link3` (same URDF) |
| Real fingertip | Fixed offset from `link3` | Moves with `gripper_joint` |
| Trust for pick-place XYZ | Marker or `/get_pose_cmd` | **`/get_pose_cmd`** / solver — not only the marker |
| `move_joint_cmd` `gripper` field | Optional for IK; not jaw motion → **`/gripper_cmd`** | Pass current `gripper` so IK matches fingertip |

Gripper mesh selection: [Gripper Configuration](description.md#gripper-configuration).

<div class="img-row img-row-2 img-row-equal">

<img class="img-zoom" alt="roarm_m2 angular_gear tcp (1)" src="https://github.com/user-attachments/assets/d98bf586-0563-4fb7-83d9-f4557cfb222d" />

<img class="img-zoom" alt="roarm_m2 angular_gear tcp (2)" src="https://github.com/user-attachments/assets/0e33761f-abb6-4b37-aad1-f0c670d4bb34" />

</div>

---

#### roarm_m3

- Built-in gripper on **`link5`**; `hand_tcp` is fixed on that link.
- Wrist joints give **roll** and **pitch** — included in `/get_pose_cmd` and `/move_joint_cmd` ( `yaw` in `.srv` is unused by the solver today).
- No `GRIPPER_TYPE` split; real TCP tracking follows the roarm_m3 solver (single model).

Service examples: [Command Control](command_control.md).

<img class="img-zoom img-center" width="200" height="100" alt="roarm_m3 angular_direct tcp"  src="https://github.com/user-attachments/assets/ea2a3a43-6de6-4a7b-bfaf-4b44eb6adabb" />

---

## Which feature uses which frame?

| Feature | Frame / TCP |
|---------|-------------|
| MoveIt — drag & **Plan & Execute** | `hand_tcp` (URDF) |
| MoveIt Servo — **`w`** / **`e`** or gamepad **X** / **Y** | `base_link` / `hand_tcp` |
| `/get_pose_cmd` | Solver real TCP in `base_link` |
| `/move_joint_cmd`, `/move_line_cmd`, … | Solver IK target → MoveIt executes arm joints |
| Vision pick-place | Solver in `roarm_vision` (roarm_m2 `angular_gear`: pass consistent `gripper`; M3: single model) |

**Units:** TF, URDF, MoveIt, and service responses use **meters** and **radians**.

---

## Related

| Goal | Go to |
|------|-------|
| TF tree diagrams & joint tables | [Robot Description](description.md#tf-tree) |
| Drag marker in RViz | [MoveIt2](moveit2.md) |
| Keyboard / gamepad Cartesian jog | [Keyboard & Gamepad Control](keyboard_control.md) |
| Services & poses | [Command Control](command_control.md) |
| Camera pick-place & frames | [Vision](vision.md) |
| Debug TF with ROS tools | [ROS2 Basics — TF2](ros2_basics.md#tf2) |

**Next:** [Installation](installation.md) — build `roarm_ws` and set `ROARM_MODEL`, `GRIPPER_TYPE` (and `GZ_VERSION` if you use Gazebo).
