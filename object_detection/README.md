# Object Detection

Python-based point cloud processing system for detecting surfaces and objects using PCL (Point Cloud Library).

## Overview

This package processes 3D point clouds from depth cameras to detect:

- **Surfaces**: Flat planes (e.g., table surfaces) using RANSAC plane segmentation
- **Objects**: Individual objects on surfaces using Euclidean clustering

## Architecture

### Main Components

1. **ObjectDetection Node** (`object_detection.py`)

   - Subscribes to point cloud data
   - Performs plane segmentation
   - Extracts object clusters
   - Publishes markers and detection messages

2. **StaticTransformPublisher** (`static_transform_publisher.py`)
   - Publishes static transforms between frames
   - Configurable for different camera setups

## Processing Pipeline

```
PointCloud2 Input
    ↓
Transform to base_link frame
    ↓
Filter by Y and Z bounds
    ↓
Plane Segmentation (RANSAC)
    ↓
Euclidean Clustering
    ↓
Calculate Centroids & Dimensions
    ↓
Publish Markers & Messages
```

## Nodes

### object_detection

Main detection node processing point clouds.

**Subscriptions:**

- `/wrist_rgbd_depth_sensor/points` (PointCloud2): Input point cloud

**Publications:**

- `surface_markers` (MarkerArray): Detected surfaces as green cubes
- `surface_detected` (DetectedSurfaces): Surface information
- `object_markers` (MarkerArray): Detected objects as red cubes
- `object_detected` (DetectedObjects): Object information

**Parameters:**

- `pointcloud_topic`: Input point cloud topic (default: `/wrist_rgbd_depth_sensor/points`)
- `plane_min_y`: Minimum Y coordinate for plane detection (default: -0.1)
- `objects_min_y`: Minimum Y coordinate for object detection (default: -0.1)

### static_transform_publisher

Publishes static transforms between coordinate frames.

**Parameters (Real Robot):**

- `header_frame_id`: Parent frame (default: `base_link`)
- `child_frame_id`: Child frame (default: `camera_depth_optical_frame`)
- `translation_x/y/z`: Translation offsets
- `rotation_x/y/z/w`: Rotation quaternion

## Launch Files

### object_detection.launch.py

Simulation environment launch.

**Starts:**

- Static transform publisher
- Object detection node
- RViz with visualization

**Usage:**

```bash
ros2 launch object_detection object_detection.launch.py
```

### object_detection_real.launch.py

Real robot environment launch.

**Configuration:**

- Camera frame: `camera_depth_optical_frame`
- Base frame: `base_link`
- Transform: Position (0.5, 0.605, 0.1) with rotation

**Usage:**

```bash
ros2 launch object_detection object_detection_real.launch.py
```

## Algorithms

### Plane Segmentation

- **Method**: RANSAC (Random Sample Consensus)
- **Model**: Planar model
- **Distance Threshold**: 0.01 m
- **Purpose**: Detect table surfaces

### Euclidean Clustering

- **Tolerance**: 0.02 m (cluster points within 2cm)
- **Min Cluster Size**: 100 points
- **Max Cluster Size**: 80,000 points
- **Purpose**: Group points into individual objects

## Output Messages

### DetectedSurfaces

```
uint32 surface_id
geometry_msgs/Point position
float32 height
float32 width
```

### DetectedObjects

```
uint32 object_id
geometry_msgs/Point position
float32 height
float32 width
float32 thickness
```

## Visualization

RViz displays:

- **Green Cubes**: Detected surfaces (table)
- **Red Cubes**: Detected objects
- **Point Cloud**: Raw input data

## Key Features

1. **Frame Transformation**: Automatic transform from camera to base_link
2. **Filtering**: Y and Z bounds filtering for region of interest
3. **Robust Detection**: RANSAC for plane detection, clustering for objects
4. **Real-time Processing**: Continuous point cloud processing
5. **Dual Output**: Both markers (visualization) and messages (control)

## Dependencies

- `rclpy`: ROS 2 Python client library
- `python-pcl`: Point Cloud Library Python bindings
- `sensor_msgs`: Sensor message definitions
- `visualization_msgs`: Visualization message definitions
- `tf2_ros`: Transform library
- `detection_interfaces`: Custom detection messages

## Installation

```bash
# Install PCL Python bindings
pip install python-pcl

# Build the package
colcon build --packages-select object_detection
```

## Usage Examples

### Run Detection (Simulation)

```bash
ros2 launch object_detection object_detection.launch.py
```

### Run Detection (Real Robot)

```bash
ros2 launch object_detection object_detection_real.launch.py
```

### Monitor Detection Output

```bash
# View detected objects
ros2 topic echo /object_detected

# View detected surfaces
ros2 topic echo /surface_detected

# View markers in RViz
# (Automatically launched with detection)
```

## Troubleshooting

**No objects detected:**

- Check point cloud bounds with `compute_bounds()` output
- Adjust `plane_min_y` and `objects_min_y` parameters
- Verify camera transform is correct

**Noisy detections:**

- Increase cluster tolerance (0.02 → 0.03)
- Increase minimum cluster size (100 → 200)
- Adjust plane distance threshold (0.01 → 0.015)

## Author

Dmitri Manajev
