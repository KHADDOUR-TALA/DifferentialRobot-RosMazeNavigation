Differential Robot – ROS Package

A full ROS project implementing a differential-drive robot, URDF model, simulated LiDAR, maze visualization, autonomous navigation, and a reset-pose ROS service.
The robot drives through a multi-layer maze, uses simulated LiDAR readings, and visualizes motion, TF tree, and traveled path in RViz.

📌 Table of Contents

Overview

Features

Package Structure

URDF Robot Model

Simulated LiDAR

Maze Visualization

Autonomous Navigation Node

Reset Pose Service

RViz Visualization

How to Run

Demo Video

Installation

Future Improvements

⭐ Overview

This ROS package models a differential-drive robot with:

A URDF/Xacro robot description

A simulated LiDAR sensor

A multi-layer maze generated using RViz markers

Autonomous navigation using obstacle avoidance & wall following

TF broadcast for robot movement

Robot path visualization

A custom ROS service to reset the robot pose

The project runs entirely inside RViz using TF transforms — no Gazebo required.

✨ Features

✔ URDF model including chassis, wheels, caster wheel, and lidar

✔ Custom materials (RGB colors)

✔ Fully simulated LiDAR with ray-casting

✔ Maze generated with LINE_LIST markers

✔ Autonomous robot navigation using wall-following

✔ TF broadcaster simulating odometry

✔ Robot path visualization (nav_msgs/Path)

✔ ROS service to reset pose (reset.srv)

✔ RViz configuration file included

✔ Simple Arduino emulator receiving /cmd_vel

📁 Package Structure
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

🤖 URDF Robot Model

The robot is defined using modular Xacro files:

materials.xacro defines reusable colors

robot.xacro defines:

base_link

chassis (blue)

Hokuyo-style LiDAR (white)

left/right wheels (blue)

caster wheel (black)

fixed and continuous joints

URDF is loaded automatically in rviz.launch using:

<param name="robot_description"
       command="$(find xacro)/xacro '$(find differential_robot)/robot/robot.xacro'"/>

🔦 Simulated LiDAR

Node: broad_lidar.py

Features:

Ray-casting against maze wall segments

360° scan from −π to +π

Publishes /scan as sensor_msgs/LaserScan

Publishes TF from base_link → hokuyo_link

The LiDAR uses walls defined as (x1,y1,x2,y2) line segments and computes the nearest intersection.

🧱 Maze Visualization

Node: lidar_data.py

Publishes a LINE_LIST marker to /visualization_marker

Multi-layer maze (outer → inner)

Visible in RViz under the "maze" namespace

🤖 Autonomous Navigation Node

Node: broad_robot.py

Implements maze navigation using LiDAR sectors:

front sector for collision avoidance

right sector for wall following

Adjustable parameters:

base_speed

turn_speed

safety_distance

wall_follow_distance

Simulated odometry is generated manually and published through TF:

world → base_link


A robot path is stored and published as:

/robot_path   (nav_msgs/Path)

🔄 Reset Pose Service

Service file: reset.srv

float32 x
float32 y
float32 yaw
---
bool success


Client: client.py

Call example:

rosrun differential_robot client.py


The service resets:

robot pose

robot path history

🛰 RViz Visualization

RViz file: rviz/robot.rviz

Visualizes:

URDF robot

Laser scan

Maze markers

TF tree (world → base_link → sensor)

Path traced by the robot

Velocity commands & interactions

🚀 How to Run
1️⃣ Source your workspace
source devel/setup.bash

2️⃣ Launch RViz + robot + all nodes
roslaunch differential_robot rviz.launch


This automatically launches:

URDF robot

robot_state_publisher

joint_state_publisher

RViz

maze marker publisher

simulated LiDAR

navigator (robot controller)

Arduino emulator

🎥 Demo Video

Add your demo link here

Example:

https://youtu.be/your_video_link_here

🔧 Installation

Clone inside your ROS workspace:

cd ~/catkin_ws/src
git clone https://github.com/yourusername/differential_robot.git
cd ..
catkin_make

🚀 Future Improvements

Add Gazebo compatibility

Add SLAM (gmapping or cartographer)

Real differential-drive kinematics

RViz interactive controls

Real robot hardware integration with Arduino/ESP32
