# CT-ICP ROS2 Migration - Build and Test Instructions

## Overview
This document provides instructions for building and testing the ROS2 (Humble) port of CT-ICP.

## Prerequisites
- **ROS2 Humble** installed on Ubuntu 22.04
- **CT-ICP core libraries** already built and installed (via superbuild)
- Required development tools: CMake 3.14+, C++17 compiler, colcon

## Directory Structure
```
ct_icp/
├── install/                    # CT-ICP and SlamCore libraries (from Phase 1+2)
└── ros2_ws/src/
    ├── ros2core/               # ROS2 bridge library
    └── ct_icp_odometry/        # ROS2 odometry nodes
        ├── src/                # C++ source files (3 nodes)
        ├── launch/             # Python launch files
        └── params/             # Config and RViz files
```

## Build Instructions

### Step 1: Source ROS2 Environment
```bash
source /opt/ros/humble/setup.bash
```

### Step 2: Build ROS2 Packages
```bash
cd ct_icp/ros2_ws

# Full build with superbuild dependencies
colcon build \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DSUPERBUILD_INSTALL_DIR=/absolute/path/to/ct_icp/install

# With verbose output (optional)
colcon build --cmake-args -DSUPERBUILD_INSTALL_DIR=/path/to/install --verbose
```

### Step 3: Source the Workspace
```bash
source install/setup.bash
```

## Running CT-ICP ROS2 Nodes

### Option A: CT-ICP SLAM from Live Point Cloud Topic
```bash
# Launch the odometry node waiting for point clouds on /ct_icp/pointcloud
ros2 launch ct_icp_odometry ct_icp_slam.launch.py \
    config:=/path/to/config.yaml \
    rviz:=true
```

### Option B: CT-ICP on a Dataset
```bash
# Launch dataset node + odometry (e.g., KITTI dataset)
ros2 launch ct_icp_odometry ct_icp_on_dataset.launch.py \
    dataset:=kitti \
    root_path:=/path/to/kitti \
    sequence:=00 \
    config:=/path/to/config.yaml
```

**Available datasets:** kitti, hilti_2021, hilti_2022, nclt, etc.

### Option C: CT-ICP on a ROS2 Bag
```bash
# Playback a ROS2 bag and run SLAM
ros2 launch ct_icp_odometry ct_icp_on_rosbag.launch.py \
    rosbag:=/path/to/bag.db3 \
    topic:=/velodyne_points \
    config:=/path/to/config.yaml \
    rate:=1.0 \
    start_sec:=0.0
```

## Testing

### Unit Tests
The core C++ libraries (SlamCore, CT_ICP) have their own test suites:
```bash
# From ct_icp/cmake-build-release
ctest -j$(nproc)
ctest --verbose
```

### ROS2 Package Check
```bash
cd ct_icp/ros2_ws
ros2 pkg create --rosdistro humble --dependencies ct_icp_odometry --help
colcon test
```

## Key Configuration Files

- **YAML config examples:** `ros2_ws/src/ct_icp_odometry/params/ct_icp/`
  - `ct_icp_driving.yaml` - Default driving profile
  - `ct_icp_driving_fast.yaml` - Faster profile (fewer keyframes)
  - Dataset-specific configs in subdirectories

- **RViz config:** `ros2_ws/src/ct_icp_odometry/params/ct_icp_odometry.rviz`
  - Automatically loaded when `rviz:=true`
  - Visualizes odometry, key points, and world map

## Troubleshooting

### Build Fails: "Could not find CT_ICP"
- Verify CT-ICP Phase 1 and 2 build completed successfully
- Confirm `SUPERBUILD_INSTALL_DIR` is set correctly and points to the `install/` directory
- Check that `install/CT_ICP/lib/cmake/` exists

### No Point Clouds Published
- Ensure input topic matches the remapping in launch file
- Use `ros2 topic list` and `ros2 topic echo /ct_icp/pointcloud` to verify topic flow
- Check debug output: `debug_print:=true`

### RViz Crashes / No Markers
- Verify RViz2 is installed: `apt install ros-humble-rviz2`
- Load the rviz config manually: `rviz2 -d params/ct_icp_odometry.rviz`
- Check that topic remappings match published topics

## Important Notes

1. **No PCL dependency:** Unlike ROS1 version, ROS2 uses `sensor_msgs::msg::PointCloud2` directly (no PCL auto-publish)
2. **ROS1 custom message removed:** `slam_roscore::monitor_entry` is deferred to Phase 2. Evaluation node publishes metrics only if explicit logging is added later
3. **Time fields:** Use `.nanosec` (not `.nsec`) for `builtin_interfaces::msg::Time`
4. **Frame IDs:** Default TF frames are `odom` (world) → `base_link` (sensor)

## Next Steps (Phase 2 - Future)

- Add `slam_roscore::monitor_entry` custom message
- Port monitoring/evaluation node fully
- Add robot state publisher for dynamic TF
- Performance optimization and profiling
- Integration tests with ROS2 test framework

