# MoveIt2 Configuration (Real Robot)

MoveIt2 configuration package for UR3E robot hardware integration.

## Quick Start

```bash
# Terminal 1: Start UR driver
ros2 launch ur_robot_driver ur_control.launch.py robot_ip:=192.168.1.100

# Terminal 2: Start MoveIt2
ros2 launch real_moveit_config move_group.launch.py

# Terminal 3: Run manipulation scripts
ros2 run moveit2_scripts pick_and_place_real
```

## Prerequisites

- UR3E robot with Robotiq 85 gripper
- Depth camera (e.g., RealSense D435)
- Control PC on same network as robot
- `ur_robot_driver` package installed

## Integration with moveit2_scripts

After launching the UR driver and MoveIt2, use these commands:

```bash
# Pick-and-place (real robot)
ros2 run moveit2_scripts pick_and_place_real

# Perception-guided pick-and-place
ros2 launch moveit2_scripts pick_and_place_perception_real.launch.py

# Gripper control
ros2 run moveit2_scripts gripper_control 0.0 radians

# Cartesian movement
ros2 run moveit2_scripts cartesian_move --ros-args -p x:=0.3 -p y:=0.0 -p z:=0.2
```

See [moveit2_scripts/README.md](../moveit2_scripts/README.md) for all available commands.

## Safety

⚠️ **IMPORTANT: Real Robot Safety**

1. **Always enable teach pendant** for emergency stop
2. **Start with slow speeds** (velocity scaling 0.1-0.2)
3. **Clear workspace** before running autonomous tasks
4. **Monitor robot continuously** during operation
5. **Test in simulation first** before real robot

## Robot Configuration

- **Robot:** UR3E (6-DOF collaborative arm)
- **Gripper:** Robotiq 85 (parallel gripper)
- **Planner:** OMPL (RRTConnect)
- **IK Solver:** KDL

## Author

Dmitri Manajev
