<div align="center">

<img src="assets/banner.svg" alt="Autonomous Mobile Robots" width="100%"/>

# Autonomous Mobile Robots

End-to-end autonomous navigation for a differential-drive mobile robot: localization, planning and control on ROS 2.

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)
![CoppeliaSim](https://img.shields.io/badge/Simulator-CoppeliaSim-2E8BC0)
![Docker](https://img.shields.io/badge/Dev%20Container-Docker-2496ED?logo=docker&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-lightgrey)

</div>

---

## Overview

This repository contains the labs and final project of the Autonomous Mobile Robots course (BSc in Mathematical Engineering and Artificial Intelligence, ICAI – Universidad Pontificia Comillas). It implements the complete navigation stack of a differential-drive robot, a TurtleBot3 Burger, in the CoppeliaSim simulator, with a parallel workspace for the physical robot.

| Layer | Technique | Package |
|-------|-----------|---------|
| Localization | Monte Carlo particle filter with LiDAR, supporting global and local localization | `amr_localization` |
| Planning | Probabilistic Roadmap (PRM) with grid sampling, obstacle inflation and path smoothing | `amr_planning` |
| Reactive control | State-machine wall follower | `amr_control` |
| Path tracking | Pure Pursuit with configurable lookahead distance | `amr_control` |
| Simulation | ROS 2 to CoppeliaSim bridge, scenes and robot model | `amr_simulation` |
| Orchestration | Launch files and node lifecycle manager | `amr_bringup` |

## Architecture

```mermaid
flowchart LR
    SIM[CoppeliaSim<br/>TurtleBot3 + LiDAR] -- scan, odometry --> PF[Particle filter]
    SIM -- scan --> WF[Wall follower]
    PF -- estimated pose --> PRM[PRM<br/>planner]
    PF -- estimated pose --> PP[Pure Pursuit]
    PRM -- smoothed path --> PP
    PP -- v, ω --> SIM
    WF -- v, ω --> SIM
    LM[Lifecycle manager] -. configure / activate .-> PF & PRM & WF & PP & SIM
```

All nodes are ROS 2 lifecycle nodes. The `lifecycle_manager` configures and activates them in a fixed order, with the simulator always last.

## Repository structure

```
|
├── sim_ws/                     # ROS 2 workspace (simulation)
│   ├── .devcontainer/          # Dev Container (ROS 2 Humble + xrdp desktop)
│   └── src/
│       ├── amr_bringup/        # Launch files (lab02, lab03, lab04, project) and lifecycle manager
│       ├── amr_control/        # Wall follower and Pure Pursuit
│       ├── amr_localization/   # Particle filter, maps and ray tracing
│       ├── amr_planning/       # Probabilistic Roadmap
│       ├── amr_simulation/     # CoppeliaSim node, robot and scenes (.ttt)
│       ├── amr_msgs/           # Custom messages
│       └── tutorial_pubsub_py/ # Publisher/subscriber tutorial
├── tb3_ws/                     # ROS 2 workspace (real TurtleBot3)
├── final_reports/              # Lab reports (P1-P4) and final presentation
└── Proyecto.pdf                # Final project statement
```

## Getting started

### Requirements

- [Docker](https://www.docker.com/) and [Visual Studio Code](https://code.visualstudio.com/) with the Dev Containers extension.
- [CoppeliaSim](https://www.coppeliarobotics.com/) on the host machine. The container reaches it through `host.docker.internal`.

### Installation

```bash
git clone <repository-url>
code .    # open sim_ws/ and choose "Reopen in Container"
```

Then, inside the container (ROS 2 Humble):

```bash
cd sim_ws
colcon build --symlink-install
source install/setup.bash
```

### Usage

Start CoppeliaSim on the host and launch the desired scenario:

```bash
ros2 launch amr_bringup lab02.launch.py      # Lab 2: reactive control (wall follower)
ros2 launch amr_bringup lab03.launch.py      # Lab 3: particle filter localization
ros2 launch amr_bringup lab04.launch.py      # Lab 4: PRM planning
ros2 launch amr_bringup project.launch.py    # Final project: integrated navigation
```

In `project.launch.py` the robot starts at `(-1, -1, 90°)`, localizes itself globally with 1000 particles and navigates to the goal `(-0.6, 1.0)` by combining PRM planning and Pure Pursuit tracking, with the wall follower as a reactive behavior.

## Key parameters

Default values used in the final project launch file.

| Node | Parameter | Value |
|------|-----------|-------|
| `particle_filter` | `particles` / `global_localization` | `1000` / `True` |
| `particle_filter` | `sigma_v`, `sigma_w`, `sigma_z` | `0.05`, `0.1`, `0.2` |
| `probabilistic_roadmap` | `grid_size` / `connection_distance` | `0.2` m / `0.3` m |
| `probabilistic_roadmap` | `obstacle_safety_distance` | `0.15` m |
| `pure_pursuit` | `lookahead_distance` | `0.3` m |

## Documentation

- Lab reports: [P1](final_reports/RobotsMovilesAutonomos_P1_G21.pdf), [P2](final_reports/RobotsMovilesAutonomos_P2_G21.pdf), [P3](final_reports/RobotsMovilesAutonomos_P3_G21.pdf), [P4](final_reports/RobotsMovilesAutonomos_P4_G21.pdf)
- [Final project statement](Proyecto.pdf)
- [Final presentation](final_reports/Proyecto-Final-Robots-Moviles-Autonomos.pptx)
- [Suggested improvements](sim_ws/future_improvements.md)

## Acknowledgements and license

This repository builds on base material provided by ICAI – Universidad Pontificia Comillas for the course: the ROS 2 workspace skeleton, the CoppeliaSim scenes and TurtleBot3 model, the `intersect` ray-casting library and the project statement. This material was authored by Jaime Boal (ICAI) and remains the property of its owners.

The original work of the authors (particle filter, PRM planner, wall follower, Pure Pursuit, launch configuration and documentation) is released under the [MIT License](LICENSE), which does not cover the base material above.

## Authors

- Liam Esgueva González
- Jorge Carnicero Príncipe
- Andrés Gil Vicente
