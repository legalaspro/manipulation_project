# Detection Interfaces

Custom ROS 2 message definitions for object and surface detection.

## Overview

This package defines custom message types used for communication between the object detection system and manipulation controllers.

## Message Types

### DetectedObjects.msg

Represents a single detected object with position and dimensions.

**Fields:**

```
uint32 object_id          # Unique identifier for the object
geometry_msgs/Point position  # 3D position (x, y, z) in base_link frame
float32 height            # Height dimension (X-axis extent)
float32 width             # Width dimension (Y-axis extent)
float32 thickness         # Thickness dimension (Z-axis extent)
```

**Example Usage (C++):**

```cpp
#include <detection_interfaces/msg/detected_objects.hpp>

auto object_msg = detection_interfaces::msg::DetectedObjects();
object_msg.object_id = 0;
object_msg.position.x = 0.3;
object_msg.position.y = 0.2;
object_msg.position.z = 0.1;
object_msg.height = 0.05;
object_msg.width = 0.04;
object_msg.thickness = 0.03;
```

**Example Usage (Python):**

```python
from detection_interfaces.msg import DetectedObjects

object_msg = DetectedObjects()
object_msg.object_id = 0
object_msg.position.x = 0.3
object_msg.position.y = 0.2
object_msg.position.z = 0.1
object_msg.height = 0.05
object_msg.width = 0.04
object_msg.thickness = 0.03
```

### DetectedSurfaces.msg

Represents a detected surface (e.g., table top).

**Fields:**

```
uint32 surface_id         # Unique identifier for the surface
geometry_msgs/Point position  # 3D position (x, y, z) in base_link frame
float32 height            # Height dimension (X-axis extent)
float32 width             # Width dimension (Y-axis extent)
```

**Example Usage (C++):**

```cpp
#include <detection_interfaces/msg/detected_surfaces.hpp>

auto surface_msg = detection_interfaces::msg::DetectedSurfaces();
surface_msg.surface_id = 0;
surface_msg.position.x = 0.3;
surface_msg.position.y = 0.2;
surface_msg.position.z = 0.0;
surface_msg.height = 1.0;
surface_msg.width = 0.8;
```

**Example Usage (Python):**

```python
from detection_interfaces.msg import DetectedSurfaces

surface_msg = DetectedSurfaces()
surface_msg.surface_id = 0
surface_msg.position.x = 0.3
surface_msg.position.y = 0.2
surface_msg.position.z = 0.0
surface_msg.height = 1.0
surface_msg.width = 0.8
```

## Message Topics

**Published by object_detection node:**

- `/object_detected` (DetectedObjects)

  - Detected object with position and dimensions
  - Used by: pick_and_place_perception

- `/surface_detected` (DetectedSurfaces)
  - Detected surface with position and dimensions
  - Used by: perception-guided manipulation

## Coordinate Frame

All positions are in the **base_link** frame:

- **X-axis**: Forward (toward robot)
- **Y-axis**: Left (from robot perspective)
- **Z-axis**: Up (vertical)

## Units

All measurements are in **meters**:

- Position: meters
- Dimensions (height, width, thickness): meters

## Building

```bash
colcon build --packages-select detection_interfaces
```

## Dependencies

- `geometry_msgs`: Standard ROS 2 geometry message types
- `rosidl_default_generators`: ROS 2 message generation

## Integration

These messages are used by:

1. **object_detection** package

   - Publishes DetectedObjects and DetectedSurfaces

2. **moveit2_scripts** package
   - Subscribes to DetectedObjects for perception-guided manipulation
   - Uses object position and dimensions for grasp planning

## Message Flow

```
Camera (PointCloud2)
    ↓
object_detection node
    ↓
DetectedObjects / DetectedSurfaces
    ↓
pick_and_place_perception node
    ↓
Grasp Planning & Execution
```

## Author

Dmitri Manajev
