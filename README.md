# 🚗 Differential Robot – ROS Package

A complete ROS package for a **differential-drive robot**, including URDF/Xacro description, simulated LiDAR, maze visualization, TF broadcasting, autonomous navigation, and a reset-pose ROS service. Runs entirely in RViz — **no Gazebo required**.

---

## 📌 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Package Structure](#package-structure)
- [Installation](#installation)
- [How to Run](#how-to-run)
- [URDF Robot Model](#urdf-robot-model)
- [Simulated LiDAR](#simulated-lidar)
- [Maze Visualization](#maze-visualization)
- [Autonomous Navigation Node](#autonomous-navigation-node)
- [Reset Pose Service](#reset-pose-service)
- [RViz Visualization](#rviz-visualization)
- [Nodes Description](#nodes-description)
- [Future Improvements](#future-improvements)

---

## ⭐ Overview

This ROS package implements a full differential-drive robot simulation, including:

- Custom URDF robot model with Xacro
- LiDAR simulation with ray-casting
- Multi-layer maze generated from RViz markers
- Autonomous navigation with obstacle avoidance
- TF tree simulation
- Robot path visualization
- Reset pose service

---

## ✨ Features

- ✅ Complete URDF & Xacro modular robot model
- ✅ Simulated Hokuyo-style LiDAR (360° scanning)
- ✅ Multi-layer maze using visualization markers
- ✅ Custom TF broadcaster simulating odometry
- ✅ Path visualization (`nav_msgs/Path`)
- ✅ Reset pose service (`reset.srv`)
- ✅ RViz configuration included
- ✅ Arduino emulator receiving `/cmd_vel`
- ✅ Right-wall following algorithm
- ✅ Obstacle avoidance

---

## 📁 Package Structure

---

## 🔧 Installation

1. **Clone the repository** into your ROS workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/YOUR_USERNAME/differential_robot.git
