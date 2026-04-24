# TurtleBot3 AutoRace — Autonomous Driving System

A fully integrated autonomous driving system for the TurtleBot3 platform using ROS, featuring lane following, traffic light detection, YOLO-based sign recognition, tunnel navigation, obstacle avoidance, and situation-based behavior control.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [System Architecture](#2-system-architecture)
3. [Hardware Requirements](#3-hardware-requirements)
4. [Software Requirements](#4-software-requirements)
5. [Repository Structure](#5-repository-structure)
6. [Prerequisites and Environment Setup](#6-prerequisites-and-environment-setup)
7. [Installation and Setup](#7-installation-and-setup)
8. [Running the System](#8-running-the-system)
9. [Mission Modules](#9-mission-modules)
10. [Parameter Tuning](#10-parameter-tuning)
11. [Results](#11-results)
12. [Known Limitations](#12-known-limitations)
13. [License](#13-license)
14. [Author](#14-author)
15. [Project Context](#15-project-context)

---

## 1. Project Overview

This project implements and optimizes an autonomous driving system for the **TurtleBot3 AutoRace** competition framework using the **Robot Operating System (ROS)**. The robot navigates a structured race track containing multiple driving challenges, including:

- Lane following with HSV-based color segmentation
- Traffic light detection with multi-frame confirmation
- Real-time traffic sign recognition using a trained **YOLOv8** model
- Intersection handling with corrected angular motion control
- Obstacle avoidance using LiDAR data
- Tunnel navigation using AMCL localization and DWA path planning
- Speed limit adaptation based on detected signs
- Level crossing (stop sign) detection and safe stopping behavior

A key focus of this work is the optimization of tunnel navigation, where costmap parameters, robot footprint settings, and LiDAR-based localization were carefully tuned to achieve stable and reliable traversal through narrow environments.

---

## 2. System Architecture

The system follows a three-layer autonomous driving pipeline:

```
┌─────────────────────────────────────────────────────────────┐
│                      PERCEPTION LAYER                       │
│   Camera (lane, traffic light, signs) + LiDAR (obstacles)   │
└───────────────────────────────┬─────────────────────────────┘
                                │
┌───────────────────────────────▼─────────────────────────────┐
│              SITUATION ANALYSIS & DECISION LAYER            │
│         core_mode_decider → core_node_controller            │
│   (Selects active mission based on perception outputs)      │
└───────────────────────────────┬─────────────────────────────┘
                                │
┌───────────────────────────────▼─────────────────────────────┐
│                   CONTROL & NAVIGATION LAYER                │
│     control_lane / control_move / ROS Navigation Stack      │
│         (Generates cmd_vel commands for the robot)          │
└─────────────────────────────────────────────────────────────┘
```

All modules communicate via **ROS topics** using the publish-subscribe model. Each node operates independently and asynchronously, enabling real-time decision-making.

---

## 3. Hardware Requirements

| Component | Details |
|---|---|
| Robot Platform | TurtleBot3 Burger |
| Onboard Computer | Raspberry Pi (running ROS on Ubuntu) |
| Microcontroller | OpenCR (32-bit ARM Cortex-M7) |
| Drive System | Dynamixel servo motors (differential drive) |
| Camera | Forward-facing RGB camera module |
| LiDAR | 360° LiDAR sensor (e.g. LDS-01) |
| Remote PC | Ubuntu 20.04 with ROS Noetic |
| Power | Li-Po battery |

---

## 4. Software Requirements

| Dependency | Version / Notes |
|---|---|
| Python | 3.8+ |
| OpenCV | 4.x |
| PyTorch | For YOLO inference |
| Ultralytics YOLOv8 | `pip install ultralytics` |
| ROS Navigation Stack | `sudo apt install ros-noetic-navigation` |
| ROS AMCL | Included in navigation stack |
| TurtleBot3 Packages | `sudo apt install ros-noetic-turtlebot3*` |
| rqt / RViz | Included with ROS desktop-full |

> **Note:** For OS and ROS version requirements, see the [Prerequisites and Environment Setup](#6-prerequisites-and-environment-setup) section below.

---

## 5. Repository Structure

```
turtlebot3_autorace/
│
├── turtlebot3_autorace_2020/           # Meta-package: package definitions and launch coordination
│
├── turtlebot3_autorace_camera/         # Camera calibration and image preprocessing
│   ├── launch/
│   ├── nodes/
│   └── config/
│
├── turtlebot3_autorace_core/           # Central decision-making: mode decider and node controller
│   ├── launch/
│   └── nodes/
│       ├── core_mode_decider.py        # Manages mission switching based on perception outputs
│       └── core_node_controller.py     # Activates mission behavior modules
│
├── turtlebot3_autorace_detect/         # All perception nodes
│   ├── launch/
│   └── nodes/
│       ├── detect_lane.py              # HSV-based lane marker detection
│       ├── detect_traffic_light.py     # Traffic light color detection with multi-frame confirmation
│       ├── detect_intersection.py      # Intersection sign detection and direction classification
│       ├── detect_tunnel.py            # Tunnel environment detection
│       ├── detect_construction.py      # LiDAR-based obstacle detection
│       ├── detect_parking.py           # Parking sign and area detection
│       ├── detect_level_crossing.py    # Stop sign and crossbar detection
│       └── yolo/
│           ├── best.pt                 # YOLOv8 trained weights (PyTorch)
│           ├── best.onnx               # ONNX export for inference
│           └── best.engine             # TensorRT engine (optimized for Jetson/RPi)
│
├── turtlebot3_autorace_driving/        # Motion control nodes
│   ├── launch/
│   ├── nodes/
│   │   ├── control_lane.py             # PID-based lane-following steering control
│   │   ├── control_move.py             # Discrete motion commands (turns, stops, parking)
│   │   └── control_tunnel.py           # Tunnel-specific motion handling
│   └── maps/
│       ├── tunnel.pgm                  # Pre-built map for tunnel navigation
│       └── tunnel.yaml                 # Map metadata
│
├── turtlebot3_autorace_msgs/           # Custom ROS message definitions
│   └── msg/
│       └── MovingParam.msg
│
├── param/                              # Navigation stack parameters
│   ├── costmap_common_params_burger.yaml
│   ├── local_costmap_params.yaml
│   └── global_costmap_params.yaml
│
├── .gitignore
├── LICENSE
└── README.md
```

---

## 6. Prerequisites and Environment Setup

Before proceeding with the installation, ensure your environment meets the following requirements. This project was developed and tested on a specific hardware and software stack. Deviations may require additional configuration.

### 6.1 Operating System and ROS

| Requirement | Details |
|---|---|
| OS | Ubuntu 20.04 LTS (running natively or in Oracle VM VirtualBox 7.0) |
| ROS Distribution | ROS Noetic Ninjemys |
| Python | 3.8 (pre-installed with Ubuntu 20.04) |

> **Why Ubuntu 20.04?** ROS Noetic is the only ROS 1 distribution that supports Ubuntu 20.04 and Python 3. The TurtleBot3 AutoRace 2020 package is built for ROS Noetic and is not officially supported on other distributions.

### 6.2 Installing ROS Noetic

If you do not have ROS Noetic installed, follow the official ROBOTIS quick-start guide, which covers both the Remote PC and the TurtleBot3 SBC (Raspberry Pi) setup:

- [TurtleBot3 Quick Start Guide (ROS Noetic)](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
- [Official ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu)

Alternatively, you can use the ROBOTIS automated install script:

```bash
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
chmod 755 ./install_ros_noetic.sh
bash ./install_ros_noetic.sh
```

### 6.3 TurtleBot3 SBC (Raspberry Pi) Access

The TurtleBot3 Raspberry Pi runs Ubuntu and is accessed via SSH from the Remote PC. The default credentials are:

```bash
ssh ubuntu@<ROBOT_IP>
# Default password: turtlebot
```

> **Note:** The default SSH password is the ROBOTIS factory default. It is strongly recommended to change this after initial setup.

### 6.4 Network Configuration

Both the Remote PC and the TurtleBot3 must be on the same local network. The ROS master runs on the Remote PC. Add the following to your `~/.bashrc` on each machine:

**Remote PC:**
```bash
export ROS_MASTER_URI=http://<REMOTE_PC_IP>:11311
export ROS_HOSTNAME=<REMOTE_PC_IP>
```

**TurtleBot3 (via SSH):**
```bash
export ROS_MASTER_URI=http://<REMOTE_PC_IP>:11311
export ROS_HOSTNAME=<ROBOT_IP>
```

> Replace `<REMOTE_PC_IP>` and `<ROBOT_IP>` with the actual IP addresses on your network. You can find them by running `hostname -I` in a terminal on each machine.

For a full walkthrough of the TurtleBot3 platform setup, refer to the official documentation:

- [TurtleBot3 Overview and Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [TurtleBot3 Basic Operations and Teleoperation](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/)

---

## 7. Installation and Setup

### Step 1: Set Up Your ROS Workspace

On your Remote PC, open a terminal and run:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

### Step 2: Clone This Repository

```bash
git clone https://github.com/charizardmigo/turtlebot3-autorace.git
```

### Step 3: Install Dependencies

```bash
sudo apt update
sudo apt install ros-noetic-navigation ros-noetic-turtlebot3 ros-noetic-turtlebot3-msgs
pip install ultralytics opencv-python torch torchvision
```

### Step 4: Build the Workspace

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Add the source command to your `.bashrc` so it loads automatically:

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 5: Set the TurtleBot3 Model

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

---

## 8. Running the System

### 8.1 On the TurtleBot3 Robot (via SSH)

```bash
ssh ubuntu@<ROBOT_IP>
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

### 8.2 On the Remote PC

**Step 1 — Start the camera node:**

```bash
roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch
```

**Step 2 — Start the image processing pipeline:**

```bash
roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch
roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch
```

**Step 3 — Launch all detection nodes:**

```bash
roslaunch turtlebot3_autorace_detect detect_traffic_light.launch
roslaunch turtlebot3_autorace_detect detect_lane.launch
roslaunch turtlebot3_autorace_detect detect_sign.launch
```

**Step 4 — Launch the core controller:**

```bash
roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch
```

**Step 5 — Launch the driving controller:**

```bash
roslaunch turtlebot3_autorace_driving turtlebot3_autorace_driving.launch
```

> **Tip:** You can monitor all active topics using `rostopic list` and visualize the robot's state in RViz using `roslaunch turtlebot3_navigation turtlebot3_navigation.launch`.

---

## 9. Mission Modules

### 9.1 Lane Following

- Implemented in `detect_lane.py` and `control_lane.py`
- Uses HSV color segmentation to detect white and yellow lane markers
- Separate binary masks are generated for each lane color
- Lane center is estimated and used to compute steering corrections
- Parameters are tunable in real time via `rqt_reconfigure`

**Calibrated HSV Parameters:**

| Lane Color | Hue | Saturation | Lightness |
|---|---|---|---|
| White | 0 – 179 | 0 – 60 | 180 – 255 |
| Yellow | 18 – 35 | 120 – 255 | 120 – 255 |

### 9.2 Traffic Light Detection

- Implemented in `detect_traffic_light.py`
- Detects red, yellow, and green signals using HSV blob detection
- Multi-frame confirmation (3 consecutive detections) prevents false positives
- Integrated into `core_mode_decider` to allow simultaneous lane following
- Speed is reduced to 50% of base speed during detection phase

### 9.3 Traffic Sign Recognition (YOLO)

- Implemented using a trained **YOLOv8** model
- Detects: Left, Right, Stop, Parking, Speed Limit 50, Speed Limit 100, Construction
- Inference runs on the robot using `.pt`, `.onnx`, or `.engine` weights
- Confidence threshold filtering removes low-quality detections

### 9.4 Tunnel Navigation

- Implemented in `detect_tunnel.py` and `control_tunnel.py`
- Switches from camera-based to LiDAR-based navigation upon tunnel detection
- Uses **AMCL** for localization and **DWA planner** for local path planning
- Pre-built map (`tunnel.pgm`) is loaded for navigation

**Optimized Costmap Parameters:**

| Parameter | Value | Purpose |
|---|---|---|
| `robot_radius` | 0.055 – 0.06 m | Accurate robot footprint |
| `inflation_radius` | 0.015 m | Reduced obstacle inflation |
| `obstacle_range` | 3.0 m | Improved obstacle detection |
| `raytrace_range` | 3.5 m | Better free-space clearing |

### 9.5 Intersection Handling

- Detects left/right directional signs using YOLO
- Angular velocity corrected from `+45` to `-45` to fix incorrect turning direction
- Two-stage speed control: reduced speed during detection, normal speed during execution

### 9.6 Construction Zone (Obstacle Avoidance)

- LiDAR-based obstacle detection
- Full stop enforced by setting both linear and angular velocity to zero
- Explicit stop command published before transitioning to avoidance maneuver

### 9.7 Speed Limit Adaptation

- Integrated directly into `core_mode_decider` (no separate node)
- Speed Limit 50 sign: reduces speed to `0.5 × base_speed`
- Speed Limit 100 sign: restores `base_speed`

### 9.8 Level Crossing (Stop Sign)

- Detects stop sign via YOLO
- Monitors crossbar using HSV-based red/white color segmentation
- Robot halts and waits until crossbar is no longer detected before resuming

---

## 10. Parameter Tuning

All detection parameters can be tuned in real time using the ROS dynamic reconfiguration tool:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

Key tunable parameters include:

- HSV thresholds for white and yellow lane detection
- HSV thresholds for traffic light colors (red, yellow, green)
- Blob detection parameters (minimum area, circularity)
- Navigation costmap parameters

---

## 11. Results

| Mission | Completion Time |
|---|---|
| Tunnel | 57 s |
| Traffic Light | 16 s |
| Intersection | 15 s |
| Construction | 67 s |
| Parking | 63 s |
| Speed Limit (50 to 100) | 30 s |
| Level Crossing | 29 s |

The system successfully completed all missions within a single integrated pipeline. Key improvements over the baseline AutoRace implementation include:

- Smoother lane following with reduced oscillation in curved sections
- Reliable traffic light detection with multi-frame confirmation
- Stable tunnel traversal after costmap parameter optimization
- Corrected intersection turning direction
- Complete and stable stopping behavior in the construction mission

---

## 12. Known Limitations

- **Lighting sensitivity:** HSV-based detection is sensitive to ambient lighting changes and requires recalibration in new environments.
- **Manual parameter tuning:** Color thresholds must be manually adjusted for each deployment environment.
- **Limited sensor fusion:** Camera and LiDAR data are used independently rather than fused, reducing robustness in complex scenarios.
- **Residual motion artifacts:** Minor rotational drift may occur during stopping in the construction mission.
- **Computational constraints:** YOLO inference speed is limited by the onboard hardware of the TurtleBot3 platform.

---

## 13. License

This project is licensed under the **MIT License**. See the [LICENSE](LICENSE) file for details.

> **Note:** The YOLOv8 model architecture is developed by [Ultralytics](https://github.com/ultralytics/ultralytics) and is subject to its own license terms. The TurtleBot3 AutoRace framework is developed by [ROBOTIS](https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020) and is licensed under the Apache License 2.0. This repository contains only the custom modifications and optimizations developed as part of this project.

---

## 14. Author

**Priestley Fomeche Njukang**  
Elektro- und Informationstechnik (Electrical and Computer Engineering)  
Hochschule Anhalt (Anhalt University of Applied Sciences)

📧 **Email:** fomechepriestly7@gmail.com  
🔗 **LInkedIn:** [Priestley Fomeche](https://linkedin.com/in/priestley-fomeche)  
💻 **GitHub:** [@charizardmigo](https://github.com/charizardmigo)

---

## 15. Project Context

**Course:** Autonomous Systems  
**Institution:** Hochschule Anhalt, Department of Electrical, Mechanical and Industrial Engineering  
**Date:** March 2026  
**Supervisor:** Prof. Dr. Stefan Twieg

**Skills Demonstrated:**
- Autonomous robot system integration (ROS Noetic)
- Computer vision and real-time image processing (OpenCV, HSV segmentation)
- Deep learning model training and deployment (YOLOv8)
- PID-based motion control and parameter tuning
- LiDAR-based localization and navigation (AMCL, DWA planner)
- Costmap optimization for constrained environments
- ROS topic-based publish-subscribe architecture
- Multi-sensor perception pipeline design

---         

**⭐ If you find this project useful, please consider giving it a star!**                                                                           