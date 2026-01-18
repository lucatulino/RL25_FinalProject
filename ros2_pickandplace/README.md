# ros2_pickandplace

This package contains the core logic for a collaborative multi-robot system featuring two KUKA iiwa arms and a Turtlebot3 mobile base. It implements a task-level coordinator via a Finite State Machine (FSM) and high-performance robot control using the Orocos KDL library.

## Overview

The package coordinates a complex Pick & Place workflow:

* **Autonomous Navigation**: Turtlebot3 moves between workstations using odometry and ArUco-based alignment.
* **Advanced Manipulation**: Dual KUKA iiwa arms perform picking/dropping tasks using Cartesian control with Null Space projection.
* **Collaboration**: Robots synchronize via logic triggers to ensure safe object handover.
* **Dynamic Configuration**: All control gains and logic thresholds are managed via external YAML files.

## Key Components

### 1. KDL Robot Wrapper (kdl_robot.cpp)

A robust abstraction for the Orocos KDL library:

* Dynamically builds the kinematic chain from the ROS 2 Parameter Server (`robot_description`).
* Computes real-time Jacobians, Forward Kinematics, and Gravity compensation.

### 2. Robot Controller (robot_control.cpp)

Implements two distinct control strategies:

* **Cartesian Control + Null Space**: Drives the end-effector to a 6D pose while simultaneously using the 7th joint (redundancy) to maximize distance from joint limits.
* **Trapezoidal Joint Control**: Generates smooth, S-curve-like trajectories in joint space for large-scale movements, respecting physical velocity and acceleration limits.

### 3. Collaborative Logic Nodes

* **iiwa_collaborative_node.cpp**: Manages the arm states (WAIT, SEARCH, VISUAL_SERVOING, PICK, DROP).
* **turtlebot_patrol_node.cpp**: Manages mobile base navigation and fine alignment with markers.

## Parameters and YAML Configuration

The system is fully configurable via `config/params.yaml`.

### Motion Control (`control`)

* `max_vel_joint` / `max_acc_joint`: Physical limits for the trapezoidal profiles.
* `kp_joint`: Proportional gain for joint space tracking.
* `kp_cartesian` / `ko_cartesian`: Gains for position and orientation.
* `null_space_gain`: Priority weight for the joint-limit avoidance task.

### Application Logic (`logic`)

* `tool_offset`: Length of the attached gripper/tool.
* `bot_height`: Landing surface height of the Turtlebot3.
* `search_amplitude`: Angular range for the ArUco scanning phase.
* `gripper_open` / `gripper_close`: Command values for the gripper actuators.

### Mobile Patrol (`patrol`)

* `max_linear_vel`: Maximum speed of the Turtlebot.
* `visual_servo_kp`: Proportional gain for ArUco-based orientation alignment.

## Launch and Execution

The package includes a centralized launch file that manages the entire simulation environment:

```bash
ros2 launch ros2_pickandplace collaborative_setup.launch.py

```

This launch file:

1. Spawns the world and robots in Gazebo/Ignition.
2. Initializes `ros2_control` with `dual_controllers.yaml`.
3. Loads `params.yaml` and injects parameters into the active nodes.
4. Starts the ArUco perception pipeline and logic nodes.

