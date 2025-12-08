# Differential Robot - ROS Maze Navigation

A ROS-based simulation of a differential drive robot navigating through a procedurally generated maze using simulated LiDAR and custom control algorithms.

## 📋 Table of Contents
- [Overview](#overview)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Nodes Description](#nodes-description)
- [Launch Files](#launch-files)
- [Custom Messages](#custom-messages)
- [Robot Model](#robot-model)
- [Simulation Details](#simulation-details)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## 📖 Overview

This project simulates a differential drive robot equipped with a LiDAR sensor autonomously navigating through a multi-layered maze. The system includes:

- **Robot Model**: URDF description with visual and collision properties
- **Sensor Simulation**: Simulated LiDAR with ray-casting against maze walls
- **Navigation Algorithm**: Wall-following algorithm with obstacle avoidance
- **Visualization**: RViz integration for real-time visualization
- **Arduino Emulation**: Simulates motor control interface

## 📁 Project Structure

```
differential_robot/
├── launch/
│   └── rviz.launcher          # Main launch file for visualization
├── robot/
│   ├── materials.xacro        # Color definitions for robot components
│   └── robot.xacro            # Complete robot URDF model
├── rviz/
│   └── robot.rviz             # RViz configuration preset
├── src/
│   ├── arduino.py             # Arduino motor controller emulator
│   ├── broad_lidar.py         # Simulated LiDAR publisher
│   ├── broad_robot.py         # Main navigation controller
│   ├── client.py              # Service client for pose reset
│   └── lidar_data.py          # Maze visualization publisher
├── srv/
│   └── reset.srv              # Custom service for pose reset
└── CMakeLists.txt & package.xml
```

## ⚙️ Prerequisites

- **ROS Noetic** (recommended) or Melodic
- **Ubuntu 20.04** or **18.04** (depending on ROS version)
- **Python 3** (with rospy installed)
- **RViz** for visualization
- **tf**, **sensor_msgs**, **geometry_msgs**, **visualization_msgs** ROS packages

## 🔧 Installation

### Step 1: Set Up ROS Workspace

1. **Open a terminal** and create a catkin workspace if you don't have one:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

2. **Clone this repository** into your ROS workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/YOUR_USERNAME/differential_robot.git
```

### Step 2: Build the Package

1. **Navigate to your workspace root**:
```bash
cd ~/catkin_ws
```

2. **Build the package** using catkin_make:
```bash
catkin_make
```

3. **Source the setup file** to make the package available:
```bash
source devel/setup.bash
```

### Step 3: Verify Installation

1. **Check if the package is recognized** by ROS:
```bash
rospack find differential_robot
```
This should return the path: `/home/YOUR_USERNAME/catkin_ws/src/differential_robot`

2. **Make Python scripts executable**:
```bash
cd ~/catkin_ws/src/differential_robot/src
chmod +x *.py
```

## 🚀 Usage

### Starting the Simulation

1. **Open a terminal** and source your ROS workspace:
```bash
source ~/catkin_ws/devel/setup.bash
```

2. **Launch the main simulation**:
```bash
roslaunch differential_robot rviz.launcher
```

This will launch:
- RViz with preconfigured visualization
- All necessary nodes (robot, LiDAR, navigation, etc.)
- TF transforms for robot components

### Interacting with the Simulation

- **View robot pose** in RViz (the robot model should appear)
- **Observe LiDAR scans** as red lines in RViz
- **Watch the robot navigate** through the maze autonomously
- **View the path history** as a blue line behind the robot

### Resetting Robot Position

To reset the robot to a specific pose (x, y, yaw):

1. **In a new terminal** (after sourcing ROS):
```bash
rosrun differential_robot client.py
```
This resets to position (0, 0, 0) by default.

2. **For custom positions**, modify the client.py file or create a service call:
```bash
rosservice call /reset_pose "x: 1.0 y: 2.0 yaw: 0.5"
```

## 🧠 Nodes Description

### 1. `broad_robot.py` - Main Navigation Controller
- **Node name**: `maze_navigator`
- **Function**: Implements wall-following algorithm, publishes velocity commands
- **Subscribes**: `/scan` (LaserScan)
- **Publishes**: `/cmd_vel` (Twist), `/robot_path` (Path), TF transforms
- **Services**: `/reset_pose` (custom reset service)

### 2. `broad_lidar.py` - Simulated LiDAR
- **Node name**: `simulated_lidar`
- **Function**: Simulates LiDAR sensor using ray-casting against maze walls
- **Publishes**: `/scan` (LaserScan), TF transforms (hokuyo_link)
- **Features**: 360° scan, configurable range (0.1-10m)

### 3. `lidar_data.py` - Maze Visualization
- **Node name**: `maze_marker_publisher`
- **Function**: Publishes maze walls as visualization markers
- **Publishes**: `/visualization_marker` (Marker)
- **Purpose**: Visual representation of the maze in RViz

### 4. `arduino.py` - Arduino Emulator
- **Node name**: `arduino_emulator`
- **Function**: Simulates Arduino motor controller interface
- **Subscribes**: `/cmd_vel` (Twist)
- **Purpose**: Logs velocity commands (extendable for actual motor control)

### 5. `client.py` - Service Client
- **Node name**: `reset_pose_client`
- **Function**: Calls reset service to reposition robot
- **Usage**: `rosrun differential_robot client.py`

## 🚀 Launch Files

### `rviz.launcher`
Launches the complete simulation system:

```xml
<launch>
    <!-- Arguments -->
    <arg name="use_sim_time" default="false"/>
    
    <!-- Load robot description -->
    <param name="robot_description" 
           command="$(find xacro)/xacro '$(find differential_robot)/robot/robot.xacro'"/>
    
    <!-- Robot state publisher -->
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" output="screen">
        <param name="use_sim_time" value="$(arg use_sim_time)"/>
    </node>
    
    <!-- Joint state publisher -->
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" output="screen">
        <param name="use_sim_time" value="$(arg use_sim_time)"/>
    </node>
    
    <!-- RViz visualization -->
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find differential_robot)/rviz/robot.rviz" output="screen"/>
    
    <!-- All custom nodes -->
    <node name="maze_marker_publisher" pkg="differential_robot" type="lidar_data.py" output="screen"/>
    <node name="simulated_lidar" pkg="differential_robot" type="broad_lidar.py" output="screen"/>
    <node name="maze_navigator" pkg="differential_robot" type="broad_robot.py" output="screen"/>
    <node name="arduino_emulator" pkg="differential_robot" type="arduino.py" output="screen"/>
</launch>
```

## 📨 Custom Messages

### Service: `reset.srv`
Used to reset the robot's pose during simulation:

**Request:**
```yaml
float32 x   # X coordinate
float32 y   # Y coordinate
float32 yaw # Orientation in radians
```

**Response:**
```yaml
bool success # True if reset successful
```

## 🤖 Robot Model

The robot is defined in URDF/Xacro format with the following components:

- **Base Link**: Reference coordinate frame
- **Chassis**: Main body (blue box: 0.3×0.3×0.15m)
- **LiDAR Mount**: Hokuyo sensor mount (white cylinder)
- **Wheels**: Two drive wheels (blue cylinders)
- **Caster Wheel**: Support wheel (black sphere)

**Joints:**
- `chassis_joint`: Fixed, connects base to chassis
- `laser_joint`: Fixed, mounts LiDAR
- `left_wheel_joint`: Continuous, left drive wheel
- `right_wheel_joint`: Continuous, right drive wheel
- `caster_wheel_joint`: Fixed, support wheel

## 🎮 Simulation Details

### Maze Structure
The maze consists of 5 concentric layers with openings at strategic locations:
- Outer boundary: 10×10 meters
- Four inner walls creating corridors
- Openings allow robot passage

### Navigation Algorithm
1. **Front obstacle detection**: Stops and turns if obstacle within 0.5m
2. **Right wall following**: Maintains 0.3m distance from right wall
3. **Turn decision**: Turns toward more open space when blocked
4. **Odometry**: Simple integration of velocity commands for pose estimation

### LiDAR Simulation
- **Range**: 0.1 to 10.0 meters
- **Field of view**: 360° (-π to π)
- **Resolution**: 1° increments (360 rays)
- **Update rate**: 10 Hz

## 🐛 Troubleshooting

### Common Issues and Solutions:

1. **"Package not found" error**:
   ```bash
   source ~/catkin_ws/devel/setup.bash
   rospack profile  # Rebuild package cache
   ```

2. **Python scripts not executable**:
   ```bash
   cd ~/catkin_ws/src/differential_robot/src
   chmod +x *.py
   ```

3. **TF lookup errors**:
   - Ensure all nodes are running
   - Check launch file order
   - Wait a few seconds for TF tree to stabilize

4. **RViz shows no robot**:
   ```bash
   # Check robot description
   rosparam get /robot_description | head -5
   # Check TF frames
   rosrun tf view_frames
   ```

5. **No LiDAR data**:
   - Verify `/scan` topic is publishing: `rostopic echo /scan -n1`
   - Check maze walls are published: `rostopic echo /visualization_marker -n1`

### Debugging Tips:
- Use `rqt_graph` to visualize node connections
- Check individual nodes with `rosrun differential_robot NODE_NAME.py`
- View TF tree: `rosrun tf view_frames && evince frames.pdf`

## 📄 License

This project is open-source. See the LICENSE file for details.

---

## 🎯 Quick Start Summary

1. **Clone and build**:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/YOUR_USERNAME/differential_robot.git
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

2. **Make scripts executable**:
   ```bash
   cd ~/catkin_ws/src/differential_robot/src
   chmod +x *.py
   ```

3. **Run simulation**:
   ```bash
   roslaunch differential_robot rviz.launcher
   ```

4. **Watch robot navigate** through the maze in RViz!

---

**Note**: Replace `YOUR_USERNAME` in the clone URL with your actual GitHub username when setting up the repository.
