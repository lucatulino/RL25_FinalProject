# RL25_FinalProject: Collaborative Dual-Arm Pick and Place System

This repository contains the final project for the Robotics Lab 2025 course. The system implements a collaborative workspace where two KUKA IIWA arms and a Turtlebot3 Burger interact to perform vision-guided pick and place tasks using ROS 2 Humble.

## Project Structure

The system is composed of three interconnected ROS 2 packages:

1. **ros2_pickandplace**: Core logic and control. It manages the finite state machine for the task and implements KDL-based solvers for kinematics.
2. **ros2_iiwa**: Robot description (URDF/Xacro) for the dual-arm setup, Gazebo simulation worlds, and 3D models.
3. **aruco_ros**: Official ROS package for ArUco marker detection, providing real-time pose estimation for visual servoing.

## Installation and Setup

### 1. Workspace Configuration

Create a standard ROS 2 workspace and place these packages inside

### 2. Install Dependencies

Use rosdep to install any remaining system dependencies:

```bash
cd ~/ros2_ws
rosdep update
sudo apt update
rosdep install --from-paths src --ignore-src -y

```

### 3. Build

```bash
cd ~/ros2_ws
colcon build 
source install/setup.bash

```

## Execution

To launch the full simulation (Gazebo, Robots, and Logic Nodes):

```bash
ros2 launch ros2_pickandplace collaborative_setup.launch.py

```

## Visualization with RViz

To inspect the dual arm robot model, URDF structure, and TF frames independently from the full simulation, use the provided visualization launch file.

### Launch Command

To open RViz with the specific configuration for the dual-arm setup (tf and robot model enabled):

```bash
ros2 launch iiwa_description view_robot.launch.py use_save:=true

```

* `use_save:=true`: Loads the `dual_arm.rviz` save file.
* `use_save:=false` (default): Loads a clean RViz session.
