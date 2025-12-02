# MoveIt2 Configuration (Simulation)

MoveIt2 configuration package for UR3E robot simulation with Gazebo.

## Quick Start

```bash
# Start simulation with MoveIt2
ros2 launch my_moveit_config demo.launch.py

# In another terminal, run manipulation scripts
ros2 run moveit2_scripts pick_and_place
```

## Main Launch File

### demo.launch.py

Complete simulation environment with Gazebo, MoveIt2, and RViz.

**Usage:**

```bash
ros2 launch my_moveit_config demo.launch.py
```

## Integration with moveit2_scripts

After launching `demo.launch.py`, use these commands in another terminal:

```bash
# Pick-and-place
ros2 run moveit2_scripts pick_and_place

# Gripper control
ros2 run moveit2_scripts gripper_control 0.0 radians

# Cartesian movement
ros2 run moveit2_scripts cartesian_move --ros-args -p x:=0.3 -p y:=0.0 -p z:=0.2

# Joint movement
ros2 run moveit2_scripts move_joints "0.0,-1.5708,0.0,-1.5708,0.0,0.0"
```

See [moveit2_scripts/README.md](../moveit2_scripts/README.md) for all available commands.

## Robot Configuration

- **Robot:** UR3E (6-DOF collaborative arm)
- **Gripper:** Robotiq 85 (parallel gripper)
- **Planner:** OMPL (RRTConnect)
- **IK Solver:** KDL

## Author

Dmitri Manajev
