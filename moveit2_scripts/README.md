# MoveIt2 Scripts

C++ executables for UR3E robot manipulation using MoveIt2 motion planning framework.

## Overview

This package provides command-line tools for robot control, including motion planning, gripper manipulation, and perception-guided pick-and-place operations.

## Executables

### 1. **pick_and_place**

Hardcoded pick-and-place sequence for simulation.

**Features:**

- Moves to pregrasp position
- Opens gripper
- Approaches object (Cartesian down)
- Closes gripper
- Retreats (Cartesian up)
- Moves to place position
- Opens gripper
- Returns to home

**Usage:**

```bash
ros2 run moveit2_scripts pick_and_place
```

### 2. **pick_and_place_real**

Real robot version with adjusted parameters for UR3E hardware.

**Usage:**

```bash
ros2 run moveit2_scripts pick_and_place_real
```

### 3. **pick_and_place_perception**

Vision-guided pick-and-place using object detection.

**Features:**

- Subscribes to `/object_detected` topic
- Waits for object detection
- Automatically calculates grasp pose
- Performs pick-and-place based on detected object
- Supports multiple objects

**Usage:**

```bash
ros2 launch moveit2_scripts pick_and_place_perception.launch.py
```

### 4. **pick_and_place_perception_real**

Real robot version of perception-guided pick-and-place.

**Usage:**

```bash
ros2 launch moveit2_scripts pick_and_place_perception_real.launch.py
```

### 5. **cartesian_move**

Sequential Cartesian movements with Z-first strategy.

**Features:**

- Moves Z-axis first (up/down)
- Then moves XY plane (horizontal)
- Maintains end-effector orientation
- Straight-line paths

**Parameters:**

- `x`: Target X position (meters, default: 0.25)
- `y`: Target Y position (meters, default: 0.25)
- `z`: Target Z position (meters, default: 0.25)

**Usage:**

```bash
ros2 run moveit2_scripts cartesian_move --ros-args -p x:=0.3 -p y:=0.0 -p z:=0.2
```

### 6. **gripper_control**

Robotiq 85 gripper manipulation with dual control modes.

**Modes:**

- `radians`: Direct joint angle (0.0 = open, 0.804 = closed)
- `width`: Object width in meters (0.0 = open, 0.085 = closed)

**Parameters:**

- `value`: Gripper value (radians or meters)
- `mode`: Control mode (default: "radians")

**Usage:**

```bash
# Open gripper
ros2 run moveit2_scripts gripper_control 0.0 radians

# Close gripper
ros2 run moveit2_scripts gripper_control 0.8 radians

# Gripper for 30mm object
ros2 run moveit2_scripts gripper_control 0.03 width

# Gripper for 60mm object
ros2 run moveit2_scripts gripper_control 0.06 width
```

### 7. **move_joints**

Direct joint angle control for UR3E.

**Joint Order:**

1. Shoulder Pan (j0)
2. Shoulder Lift (j1)
3. Elbow (j2)
4. Wrist 1 (j3)
5. Wrist 2 (j4)
6. Wrist 3 (j5)

**Usage:**

```bash
# Home position
ros2 run moveit2_scripts move_joints "0.0,-1.5708,0.0,-1.5708,0.0,0.0"

# Rotated shoulder
ros2 run moveit2_scripts move_joints "1.5708,-1.5708,0.0,-1.5708,0.0,0.0"
```

### 8. **move_to_pose**

Move end-effector to target pose with orientation.

**Usage:**

```bash
ros2 run moveit2_scripts move_to_pose
```

### 9. **add_ur3_sim_scene**

Add collision objects to simulation scene.

**Usage:**

```bash
ros2 run moveit2_scripts add_ur3_sim_scene
```

## Launch Files

- `pick_and_place.launch.py`: Simulation pick-and-place
- `pick_and_place_real.launch.py`: Real robot pick-and-place
- `pick_and_place_perception.launch.py`: Simulation with perception
- `pick_and_place_perception_real.launch.py`: Real robot with perception
- `cartesian_move.launch.py`: Cartesian movement launcher
- `gripper_control.launch.py`: Gripper control launcher
- `move_joints.launch.py`: Joint movement launcher
- `move_to_pose.launch.py`: Pose movement launcher

## Key Design Decisions

1. **Sequential Cartesian Moves**: Z-axis first, then XY plane for better control
2. **Meter-based Units**: All distances in meters (not mm/cm)
3. **Slow Gripper Motion**: 1-2.5% velocity scaling for precise grasping
4. **Perception Integration**: Automatic object detection and grasp planning

## Dependencies

- `rclcpp`: ROS 2 C++ client library
- `moveit_core`: MoveIt2 core library
- `moveit_ros_planning_interface`: MoveIt2 planning interface
- `control_msgs`: Control message definitions
- `detection_interfaces`: Custom detection messages

## Building

```bash
colcon build --packages-select moveit2_scripts
```

## Author

Dmitri Manajev
