# Robotics Manipulation Project

A comprehensive ROS 2 manipulation system for the UR3E collaborative robot with integrated object detection, motion planning, and gripper control capabilities.

## Project Overview

This project provides a complete manipulation pipeline for robotic pick-and-place operations, combining:

- **Motion Planning**: MoveIt2-based trajectory planning and execution
- **Object Detection**: Point cloud-based surface and object detection
- **Gripper Control**: Robotiq 85 gripper manipulation with width/radian control
- **Simulation & Real Hardware**: Support for both Gazebo simulation and real UR3E robot

## Project Structure

```
manipulation_project/
├── moveit2_scripts/          # Core manipulation executables (C++)
├── object_detection/         # Point cloud detection system (Python)
├── detection_interfaces/     # Custom ROS 2 message definitions
├── my_moveit_config/         # MoveIt2 configuration for simulation
├── real_moveit_config/       # MoveIt2 configuration for real robot
└── README.md                 # This file
```

## Key Components

### 1. **moveit2_scripts** - Motion Planning & Control

C++ executables for robot manipulation:

- `pick_and_place`: Hardcoded pick-and-place sequence
- `pick_and_place_perception`: Vision-guided pick-and-place
- `cartesian_move`: Sequential Cartesian movements (Z then XY)
- `gripper_control`: Gripper control with width/radian modes
- `move_joints`: Direct joint angle control
- `move_to_pose`: Move to target end-effector pose

### 2. **object_detection** - Vision System

Python-based point cloud processing:

- Plane segmentation (table surface detection)
- Euclidean clustering (object detection)
- Marker visualization in RViz
- Custom message publishing (DetectedObjects, DetectedSurfaces)

### 3. **detection_interfaces** - Message Definitions

Custom ROS 2 message types:

- `DetectedObjects`: Object position, dimensions, ID
- `DetectedSurfaces`: Surface position, dimensions, ID

### 4. **MoveIt2 Configurations**

- `my_moveit_config`: Simulation environment (Gazebo)
- `real_moveit_config`: Real UR3E robot configuration

## Quick Start

### Build the Project

```bash
colcon build
source install/setup.bash
```

### Run Simulation

```bash
# Terminal 1: Start MoveIt2 with Gazebo
ros2 launch my_moveit_config demo.launch.py

# Terminal 2: Run pick-and-place
ros2 run moveit2_scripts pick_and_place
```

### Run with Object Detection

```bash
# Start MoveIt2 with object detection and RViz
ros2 launch moveit2_scripts pick_and_place_perception.launch.py
```

### Control Gripper

```bash
# Open gripper (0 radians)
ros2 run moveit2_scripts gripper_control 0.0 radians

# Close gripper (0.8 radians)
ros2 run moveit2_scripts gripper_control 0.8 radians

# Gripper for 30mm object (width mode)
ros2 run moveit2_scripts gripper_control 0.03 width
```

### Move Robot

```bash
# Cartesian move to position (meters)
ros2 run moveit2_scripts cartesian_move --ros-args -p x:=0.3 -p y:=0.0 -p z:=0.2

# Move to joint angles (radians)
ros2 run moveit2_scripts move_joints "0.0,-1.5708,0.0,-1.5708,0.0,0.0"
```

## Key Features

- **Sequential Cartesian Movements**: Z-axis first, then XY plane (better control)
- **Meter-based Units**: All measurements in meters (not mm/cm)
- **Dual Mode Gripper Control**: Radians or object width
- **Real-time Visualization**: RViz integration with markers
- **Perception-guided Manipulation**: Automatic object detection and grasping

## Dependencies

- ROS 2 (Humble or later)
- MoveIt2
- PCL (Point Cloud Library)
- UR3E URDF/description packages
- Gazebo (for simulation)

## Documentation

See individual package READMEs for detailed information:

- [moveit2_scripts/README.md](moveit2_scripts/README.md)
- [object_detection/README.md](object_detection/README.md)
- [detection_interfaces/README.md](detection_interfaces/README.md)

## Author

Dmitri Manajev (dmitri.manajev@protonmail.com)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
