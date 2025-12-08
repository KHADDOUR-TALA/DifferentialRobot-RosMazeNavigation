# 🚗 Differential Robot – ROS Package

A complete ROS package of a **differential-drive robot**, including URDF/Xacro description, simulated LiDAR, maze visualization, TF broadcasting, autonomous navigation, and a reset-pose ROS service.
Runs entirely in RViz — **no Gazebo required**.

---

## 📌 Table of Contents

* [Overview](#overview)
* [Features](#features)
* [Package Structure](#package-structure)
* [URDF Robot Model](#urdf-robot-model)
* [Simulated LiDAR](#simulated-lidar)
* [Maze Visualization](#maze-visualization)
* [Autonomous Navigation Node](#autonomous-navigation-node)
* [Reset Pose Service](#reset-pose-service)
* [RViz Visualization](#rviz-visualization)
* [How to Run](#how-to-run)
* [Demo Video](#demo-video)
* [Installation](#installation)
* [Future Improvements](#future-improvements)

---

## ⭐ Overview

This ROS package implements a full differential-drive robot simulation, including:

* Custom URDF robot model
* LiDAR simulation with ray-casting
* Multi-layer maze generated from RViz markers
* Autonomous navigation with obstacle avoidance
* TF tree simulation
* Robot path visualization
* Reset pose service

---

## ✨ Features

* ✔ Complete URDF & Xacro modular robot model
* ✔ Simulated Hokuyo-style LiDAR
* ✔ Multi-layer maze using visualization markers
* ✔ Custom TF broadcaster simulating odometry
* ✔ Path visualization (`nav_msgs/Path`)
* ✔ Reset pose service (`reset.srv`)
* ✔ RViz config included
* ✔ Arduino emulator receiving `/cmd_vel`

---

## 📁 Package Structure

```
differential_robot/
│
├── launch/
│   └── rviz.launch
│
├── robot/
│   ├── materials.xacro
│   └── robot.xacro
│
├── rviz/
│   └── robot.rviz
│
├── src/
│   ├── arduino.py
│   ├── broad_lidar.py
│   ├── broad_robot.py
│   ├── lidar_data.py
│   └── client.py
│
├── srv/
│   └── reset.srv
│
├── CMakeLists.txt
└── package.xml
```

---

## 🤖 URDF Robot Model

The robot description includes:

* `base_link`
* chassis
* left & right wheels
* caster wheel
* hokuyo LiDAR
* continuous & fixed joints
* modular materials via Xacro

Loaded automatically via:

```xml
<param name="robot_description"
       command="$(find xacro)/xacro '$(find differential_robot)/robot/robot.xacro'"/>
```

---

## 🔦 Simulated LiDAR

**Node:** `broad_lidar.py`

* Ray-casting against maze walls
* Publishes `sensor_msgs/LaserScan`
* TF: `base_link` → `hokuyo_link`
* 360° scanning from –π to +π
* Detects intersection of beams with wall segments

---

## 🧱 Maze Visualization

**Node:** `lidar_data.py`

* Publishes RViz **LINE_LIST** markers
* Multi-layer maze
* Shows clear environment boundaries

---

## 🤖 Autonomous Navigation Node

**Node:** `broad_robot.py`

Implements:

* Front obstacle avoidance
* Right-wall following
* Adjustable speed & safety distance
* TF broadcasting
* Path publishing: `/robot_path`

Simulates:

```
world → base_link
```

---

## 🔄 Reset Pose Service

`reset.srv`:

```
float32 x
float32 y
float32 yaw
---
bool success
```

Call it:

```bash
rosrun differential_robot client.py
```

Resets:

* robot pose
* path
* TF
* controller internal state

---

## 🛰 RViz Visualization

Included file: `rviz/robot.rviz`

Shows:

* URDF robot
* LiDAR data
* Maze
* TF tree
* Local path
* Velocity commands

---

## 🚀 How to Run

### 1️⃣ Source your workspace

```bash
source devel/setup.bash
```

### 2️⃣ Start the whole system

```bash
roslaunch differential_robot rviz.launch
```

This launches:

* URDF robot
* TF broadcaster
* LiDAR simulation
* Maze marker publisher
* Navigation node
* RViz

---

## 🎥 Demo Video

Add your link:

```
https://youtu.be/YOUR_VIDEO_HERE
```

---

## 🔧 Installation

Clone inside your ROS workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/YOUR_USERNAME/differential_robot.git
cd ..
catkin_make
```

---

## 🚀 Future Improvements

* Add real differential-drive kinematics
* Add Gazebo simulation
* Add SLAM (Cartographer / GMapping)
* Export ROS2 version
* Add interactive markers for manual teleop

---

### ✔ DONE!

Everything above is **ready to paste directly**, and all links work correctly.

If you want:

* a **badge section**
* a **header image**
* or a **GIF of the robot moving** inside the README

Just tell me and I'll generate it!
