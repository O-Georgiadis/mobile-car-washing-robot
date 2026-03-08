# Mobile Car Washing Robot 🤖🚗

An autonomous mobile robotics project for home car washing, built with ROS2 and Gazebo simulation.

- ROS2
- Python
- Gazebo

📋 Overview

This project implements an autonomous mobile robot designed to perform car washing operations in residential garages and driveways. The robot uses a differential drive mobile base with a 6-DOF robotic arm, LiDAR-based navigation, and touchless cleaning technology.

Project Type: MSc AI Course Project

Team: 4 members


───

🎯 Key Features

- Autonomous Navigation: Scripted FSM-based navigation with obstacle detection capability
- 6-DOF Robotic Arm: Programmable manipulation for surface cleaning
- Touchless Cleaning: High-pressure water spray system (no brushes → no scratches)
- Safety First: Emergency stop functionality, collision avoidance
- Simulation-First: Full Gazebo simulation before hardware deployment

───

🛠️ Tech Stack

| Category   | Technology                    |
| ---------- | ----------------------------- |
| Framework  | ROS2 Jazzy                    |
| Language   | Python                        |
| Simulation | Gazebo, RViz                  |
| Modeling   | URDF/XACRO                    |
| Sensors    | LiDAR, RGB-D Camera           |
| Navigation | Custom FSM + Reactive Control |

📁 Project Structure
```
mobile-car-washing-robot/
└── src/
    ├── car_washing_robot_bringup/
    ├── car_washing_robot_controller/
    └── car_washing_robot_description/

```

🚀 Getting Started

Prerequisites

- Ubuntu 22.04+
- ROS2 Jazzy
- Gazebo Harmonic

```bash
mkdir -p ~/ws_robot/src
cd ~/ws_robot/src
git clone https://github.com/O-Georgiadis/mobile-car-washing-robot.git
cd ~/ws_robot
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch car_washing_robot_bringup car_washing_robot_gazebo.launch.xml
```

📊 Implementation Highlights

Navigation FSM

1. APPROACH — Close distance to vehicle
2. FOLLOW — Maintain 0.5m offset
3. TURNING — Corner maneuvers
4. COMPLETE — End of wash cycle

Safety Features

- LiDAR-based obstacle detection
- Emergency stop within 0.3m threshold

───

📈 Future Roadmap

- [ ] Q-SLAM for 3D mapping
- [ ] Computer vision for dirt detection
- [ ] MoveIt2 for IK-based arm control
- [ ] Physical prototype

───

📄 Documentation

- Part 1: Introduction, Literature Review, Requirements Analysis (36 slides)
- Part 2: Implementation, Testing, Ethics, Feasibility Analysis (55 slides)
  
───

🏆 Course

AI645 - Robotics & Perception
European University Cyprus, 2025-2026
