#!/bin/bash
# Build and test CT-ICP ROS2 migration in Docker
# Usage: bash build_ros2.sh [--test]

set -e  # Exit on error

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== CT-ICP ROS2 Migration Build Script ===${NC}"

# Detect script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$SCRIPT_DIR"

echo "Project root: $PROJECT_ROOT"

# Step 1: Source ROS2 environment
echo -e "${BLUE}Step 1: Sourcing ROS2 Humble environment...${NC}"
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}✓ ROS2 Humble sourced${NC}"
else
    echo -e "${RED}✗ ROS2 Humble not found at /opt/ros/humble${NC}"
    exit 1
fi

# Step 2: Verify CT-ICP installation
echo -e "${BLUE}Step 2: Verifying CT-ICP installation...${NC}"
INSTALL_DIR="$PROJECT_ROOT/install"

if [ ! -d "$INSTALL_DIR/CT_ICP/lib/cmake" ]; then
    echo -e "${RED}✗ CT-ICP not found at $INSTALL_DIR${NC}"
    echo "Please build CT-ICP Phase 1+2 first:"
    echo "  mkdir .cmake-build-superbuild && cd .cmake-build-superbuild"
    echo "  cmake ../superbuild"
    echo "  cmake --build . --config Release"
    exit 1
fi
echo -e "${GREEN}✓ CT-ICP installation verified at $INSTALL_DIR${NC}"

# Step 3: Create and navigate to ROS2 workspace
echo -e "${BLUE}Step 3: Setting up ROS2 workspace...${NC}"
cd "$PROJECT_ROOT/ros2_ws"

if [ ! -f "src/ros2core/CMakeLists.txt" ]; then
    echo -e "${RED}✗ ros2core package not found${NC}"
    exit 1
fi
echo -e "${GREEN}✓ ROS2 workspace verified${NC}"

# Step 4: Build ROS2 packages
echo -e "${BLUE}Step 4: Building ROS2 packages...${NC}"
echo "Build directory: $(pwd)/build"

# Set CMake environment to relax Eigen version checking (superbuild uses 3.3.7, system may have 3.4.0)
export CERES_ALLOW_CMINPACK_IMPLICIT_QR_SHIFTS=1

colcon build \
    --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DSUPERBUILD_INSTALL_DIR="$INSTALL_DIR" \
    -DCMAKE_PREFIX_PATH="$INSTALL_DIR/Eigen3/share/eigen3/cmake" \
    --event-handlers console_direct+

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Build successful${NC}"
else
    echo -e "${RED}✗ Build failed${NC}"
    exit 1
fi

# Step 5: Source the workspace
echo -e "${BLUE}Step 5: Sourcing ROS2 workspace...${NC}"
source install/setup.bash
echo -e "${GREEN}✓ Workspace sourced${NC}"

# Step 6: List built packages
echo -e "${BLUE}Step 6: Verifying built packages...${NC}"
echo "Installed executables:"
find install/lib/ct_icp_odometry -type f -executable 2>/dev/null | while read exe; do
    echo "  ✓ $(basename $exe)"
done

# Step 7: Run tests if requested
if [[ "$1" == "--test" ]]; then
    echo -e "${BLUE}Step 7: Running package tests...${NC}"
    colcon test --event-handlers console_direct+
fi

echo -e "${GREEN}=== Build Complete ===${NC}"
echo ""
echo "Next steps:"
echo "  1. Source the workspace: source ros2_ws/install/setup.bash"
echo "  2. Run SLAM on a dataset:"
echo "     ros2 launch ct_icp_odometry ct_icp_on_dataset.launch.py \\"
echo "       dataset:=kitti root_path:/path/to/kitti sequence:=00"
echo ""
