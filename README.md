![GitHub top language](https://img.shields.io/github/languages/top/waveshareteam/roarm_ws) ![GitHub language count](https://img.shields.io/github/languages/count/waveshareteam/roarm_ws)
![GitHub code size in bytes](https://img.shields.io/github/languages/code-size/waveshareteam/roarm_ws)
![GitHub repo size](https://img.shields.io/github/repo-size/waveshareteam/roarm_ws) ![GitHub](https://img.shields.io/github/license/waveshareteam/roarm_ws) ![GitHub last commit](https://img.shields.io/github/last-commit/waveshareteam/roarm_ws/ros2-humble-develop-251125)

# ROS2 + MoveIt2 for RoArm

**roarm_ws** is a **ROS2 Humble** colcon workspace for **Waveshare RoArm** ([M2](https://www.waveshare.com/roarm-m2-s.htm) / [M3](https://www.waveshare.com/roarm-m3.htm)). It connects **RViz2** and **MoveIt2** to real hardware over serial, with optional vision pick-place and Gazebo simulation.

## Documentation

Full tutorials live under [`docs/`](docs/) and are built with **MkDocs**.

| | |
|---|---|
| **Read online** | [roarm-ws.readthedocs.io](https://roarm-ws.readthedocs.io/) |
| **Build locally** | `pip install -r docs/requirements.txt` then `mkdocs build` (output in `site/`) |
| **Preview** | `mkdocs serve -a 0.0.0.0:8000` → http://ip:8000 |
| **Host on RTD** | [`.readthedocs.yaml`](.readthedocs.yaml) |

### Chapters

| Section | Page |
|---------|------|
| Overview | [docs/index.md](docs/index.md) |
| ROS2 Basics | [docs/ros2_basics.md](docs/ros2_basics.md) |
| RoArm Basics | [docs/roarm_basics.md](docs/roarm_basics.md) |
| Installation | [docs/installation.md](docs/installation.md) |
| Robot Description | [docs/description.md](docs/description.md) |
| Hardware Driver | [docs/driver_control.md](docs/driver_control.md) |
| MoveIt2 | [docs/moveit2.md](docs/moveit2.md) |
| Keyboard & Gamepad | [docs/keyboard_control.md](docs/keyboard_control.md) |
| Command Control | [docs/command_control.md](docs/command_control.md) |
| MTC Demo | [docs/mtc_demo.md](docs/mtc_demo.md) |
| Vision & Pick / Place | [docs/vision.md](docs/vision.md) |
| Gazebo Simulation | [docs/gazebo.md](docs/gazebo.md) |

Suggested order: [docs/index.md — Suggested reading order](docs/index.md#suggested-reading-order).

## Quick start

**Ubuntu 22.04** + **ROS2 Humble**. On a Raspberry Pi, use Ubuntu 22.04 on the board and skip Gazebo when prompted.

```bash
git clone -b ros2-humble-develop-251125 https://github.com/waveshareteam/roarm_ws.git
cd roarm_ws
sudo chmod +x build_first.sh
./build_first.sh
```

`build_first.sh` installs ROS2 / MoveIt dependencies, sets **`ROARM_MODEL`** and **`GRIPPER_TYPE`**, and runs `colcon build`. Details: [Installation](docs/installation.md).

### Model settings

| Hardware | `ROARM_MODEL` | `GRIPPER_TYPE` |
|----------|---------------|----------------|
| RoArm-M2 (direct gripper) | `roarm_m2` | `angular_direct` |
| RoArm-M2-GA (geared gripper) | `roarm_m2` | `angular_gear` |
| RoArm-M3 | `roarm_m3` | `angular_direct` or `angular_gear` |

There is no separate `roarm_m2_ga` model — **GA** means M2 + **`angular_gear`**. See [RoArm Basics](docs/roarm_basics.md).

### Typical real-arm workflow

1. **T0** — `ros2 run roarm_driver roarm_driver --ros-args -p serial_port:=/dev/ttyUSB0`
2. **T1** — tutorial launch (MoveIt, Servo, Cmd, vision, …) — see [Typical paths](docs/index.md#typical-paths)

Do not run **`roarm_driver`** and **Gazebo** at the same time.

## License

See repository license badge above.
