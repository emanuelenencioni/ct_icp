# Docker Setup for CT-ICP ROS2 Migration

## Quick Start

### Option 1: Using an Existing ROS2 Humble Docker Image

If you have a ROS2 Humble image, create and run a container:

```bash
# Create a new Docker container with ROS2 Humble
docker run -it \
  --name ros2 \
  -v /home/emanuele/Documenti/Codice/ct_icp:/home/emanuele/Documenti/Codice/ct_icp \
  osrf/ros:humble-full \
  /bin/bash
```

Then inside the container, build CT-ICP and the ROS2 packages:

```bash
cd /home/emanuele/Documenti/Codice/ct_icp

# Phase 1: Build superbuild (dependencies)
mkdir -p .cmake-build-superbuild
cd .cmake-build-superbuild
cmake ../superbuild
cmake --build . --config Release -j$(nproc)
cd ..

# Phase 2: Build CT-ICP core libraries
mkdir -p cmake-build-release
cd cmake-build-release
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --target install --config Release -j$(nproc)
cd ..

# Phase 3: Build ROS2 packages
bash build_ros2.sh
```

### Option 2: If You Already Have a ros2 Container

```bash
# Start the container
docker start ros2
docker exec -it ros2 /bin/bash

# Then run the build script
cd /home/emanuele/Documenti/Codice/ct_icp
bash build_ros2.sh
```

### Option 3: Direct Build in Docker

Run everything in one command:

```bash
docker run -it \
  --name ros2 \
  -v /home/emanuele/Documenti/Codice/ct_icp:/workspace \
  osrf/ros:humble-full \
  bash -c "
    cd /workspace && \
    mkdir -p .cmake-build-superbuild && \
    cd .cmake-build-superbuild && \
    cmake ../superbuild && \
    cmake --build . --config Release -j\$(nproc) && \
    cd .. && \
    mkdir -p cmake-build-release && \
    cd cmake-build-release && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    cmake --build . --target install --config Release -j\$(nproc) && \
    cd .. && \
    bash build_ros2.sh
  "
```

## Available ROS2 Docker Images

- `osrf/ros:humble-full` - Full ROS2 Humble (recommended)
- `osrf/ros:humble-ros-core` - Minimal ROS2 core
- `ros:humble` - Alternative minimal image

## After Build Completes

### Test the Build

```bash
# In the container, verify packages are built
ros2 pkg list | grep ct_icp

# List executables
ls -la ros2_ws/install/lib/ct_icp_odometry/
```

### Run a Quick Test

```bash
# Source the workspace
cd /workspace/ros2_ws
source install/setup.bash

# Check if nodes can be found
ros2 run ct_icp_odometry ct_icp_odometry_node --help

# Or test with a launch file (needs data)
ros2 launch ct_icp_odometry ct_icp_slam.launch.py \
  config:=/workspace/ros2_ws/src/ct_icp_odometry/params/ct_icp/ct_icp_driving.yaml \
  rviz:=false
```

## Troubleshooting Docker Build

### Error: "Could not find CT_ICP"

This means Phase 1+2 didn't complete. Check:
```bash
ls -la /workspace/install/CT_ICP/lib/cmake/
```

If empty, rebuild:
```bash
cd /workspace/.cmake-build-superbuild
cmake --build . --config Release -j$(nproc)
cd /workspace/cmake-build-release
cmake --build . --target install --config Release -j$(nproc)
```

### Error: "colcon: command not found"

Install colcon in the container:
```bash
apt update && apt install -y python3-colcon-common-extensions
```

### Out of Memory

Reduce parallel jobs:
```bash
colcon build --cmake-args -DSUPERBUILD_INSTALL_DIR=/workspace/install -j2
```

## Running GUI (RViz) from Docker

If you want to run RViz visualization, use:

```bash
docker run -it \
  --name ros2 \
  -v /home/emanuele/Documenti/Codice/ct_icp:/workspace \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  osrf/ros:humble-full \
  /bin/bash
```

Then in the container:
```bash
ros2 launch ct_icp_odometry ct_icp_on_dataset.launch.py \
  dataset:=kitti \
  root_path:/path/to/kitti \
  sequence:=00
```

## Exiting Docker

```bash
exit  # Leave the container
docker stop ros2  # Stop container
docker rm ros2  # Remove container (if not needed again)
```

## Stopping and Restarting

```bash
# Stop without removing
docker stop ros2

# Start again later
docker start ros2
docker exec -it ros2 /bin/bash
```

