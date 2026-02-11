# AGENTS.md — CT-ICP

CT-ICP (Continuous-Time ICP) is a LiDAR odometry/SLAM system. Two core C++ libraries:
**SlamCore** (generic SLAM utilities) and **CT_ICP** (ICP odometry, registration, maps).
Both are pure C++17 with zero ROS dependencies. ROS wrapping lives entirely in `ros/`.

## Build Commands

The project uses a two-phase CMake build (superbuild for deps, then main project).

### Phase 1: Superbuild (install external dependencies)
```bash
mkdir .cmake-build-superbuild && cd .cmake-build-superbuild
cmake ../superbuild          # add -DWITH_VIZ3D=ON for GUI
cmake --build . --config Release
```
This creates `install/` with `superbuild_import.cmake` at the root.

### Phase 2: Build CT-ICP
```bash
mkdir cmake-build-release && cd cmake-build-release
cmake .. -DCMAKE_BUILD_TYPE=Release   # add -DWITH_VIZ3D=ON for GUI
cmake --build . --target install --config Release --parallel $(nproc)
```

### CMake Options
| Option               | Default | Description                    |
|----------------------|---------|--------------------------------|
| `WITH_VIZ3D`         | OFF     | 3D visualization (OpenGL)      |
| `WITH_GTSAM`         | OFF     | GTSAM graph optimization       |
| `WITH_PYTHON_BINDING` | OFF    | Python bindings (pybind11)     |

### Key Dependencies
Eigen3, Ceres, glog, yaml-cpp, OpenMP, GTest, tsl::robin_map, nanoflann,
tinyply, cereal, tclap, colormap. All managed by the superbuild (ExternalProject).

## Test Commands

Framework: **Google Test**. All unit tests compile into a single `all_tests` binary.

```bash
# Run all tests via CTest (from build dir)
ctest -j$(nproc)
ctest --verbose

# Run the combined test binary directly
./test/unit/all_tests

# Run a single test suite (GTest filter)
./test/unit/all_tests --gtest_filter="CostFunctions.*"

# Run a single test case
./test/unit/all_tests --gtest_filter="CostFunctions.PointToPlane"

# Run individual test executables
./test/unit/SlamCore/test_pointcloud
./test/unit/ct_icp/test_cost_functions
./test/unit/ct_icp/test_cost_functions --gtest_filter="CostFunctions.PointToPlane"

# Integration tests (standalone, not GTest-registered)
./test/integration/testint_odometry
./test/integration/testint_dataset -c <config_file>

# Regression tests
./test/regression/regression_run -c <path-to-yaml-config>
```

Test files: `test/unit/SlamCore/` (~25 tests), `test/unit/ct_icp/` (4 tests).
Integration tests: `test/integration/`. Regression: `test/regression/`.

## Code Style

No `.clang-format` or `.clang-tidy` exists. Follow these observed conventions:

### Language & Standard
- **C++17** required. Uses `if constexpr`, `std::optional`, structured bindings, `<filesystem>`.
- **4 spaces** indentation, no tabs. Namespace content IS indented.

### Naming Conventions
| Element              | Convention          | Example                                  |
|----------------------|---------------------|------------------------------------------|
| Classes/Structs      | PascalCase          | `TrajectoryFrame`, `OdometryOptions`     |
| Abstract base        | `A` prefix          | `AMotionModel`, `ANeighborhoodStrategy`  |
| Interface structs    | `I` prefix          | `ISlamMap`, `IMapOptions`                |
| Methods              | PascalCase          | `RegisterFrame()`, `GetMapPointCloud()`  |
| Private members      | snake_case + `_`    | `options_`, `trajectory_`, `map_`        |
| Public struct fields | snake_case           | `begin_pose`, `timestamp`, `num_points`  |
| Local variables      | snake_case           | `sample_size`, `voxel_block`             |
| Local constants      | `k` + PascalCase    | `kFrameIndex`, `kMaxDistance`            |
| Enums (type+values)  | ALL_CAPS             | `MOTION_COMPENSATION::CONTINUOUS`        |
| Namespaces           | lowercase            | `slam`, `ct_icp`, `slam::config`         |
| Type aliases (`_t`)  | snake_case           | `frame_id_t`, `pair_distance_t`          |
| Pointer typedefs     | PascalCase + `Ptr`   | `PointCloudPtr`, `ReactorFactoryPtr`     |
| Template params      | Single letter/Pascal | `T`, `ScalarT`, `DataT`, `ReactorT`     |

### File Naming
- Headers: `.h` (not `.hpp`). Sources: `.cxx` (SlamCore) or `.cpp` (CT_ICP).
- Lowercase with underscores: `cost_functions.h`, `motion_model.h`.
- Test files: `test_<name>.cxx`. Integration tests: `testint_<name>.cpp`.
- Headers in `include/<module>/`, sources in `src/<module>/`.

### Header Guards
Use `#ifndef`/`#define`/`#endif` (NOT `#pragma once`). Pattern: `MODULE_FILENAME_H`:
```cpp
#ifndef CT_ICP_ODOMETRY_H
#define CT_ICP_ODOMETRY_H
// ...
#endif
```

### Includes
Order: (1) own header, (2) C stdlib, (3) C++ stdlib, (4) third-party with `<>`,
(5) SlamCore with `<SlamCore/...>`, (6) ct_icp with `"ct_icp/..."`.

### Brace Style
K&R — opening brace on same line for classes, functions, and control flow.
Single-statement bodies sometimes omit braces (inconsistent — prefer adding them).

### Error Handling
- **glog CHECK macros** (primary): `CHECK(cond) << "msg"`, `CHECK_EQ`, `CHECK_LE`
- **Custom macros**: `SLAM_CHECK_STREAM(cond, msg)`, `SLAM_LOG(WARNING)`
- **Exceptions** for unimplemented/unrecoverable: `throw std::runtime_error("...")`
- **Boolean return fields** in result structs: `summary.success = false`

### Patterns
- `explicit` on single-argument constructors. `= default` for trivial ctors.
- `[[nodiscard]]` on methods returning important values.
- `std::shared_ptr` (dominant), `std::unique_ptr` for exclusive ownership.
- `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` in structs with fixed-size Eigen types.
- OpenMP `#pragma omp parallel for` for performance-critical loops.
- Macro-generated accessors: `REF_GETTER`, `CONST_REF_GETTER`, `GETTER_SETTER`.
- Section separators: `/* ---- ... ---- */` between functions, `/// ====` for sections.

## Project Structure

```
ct_icp/
├── CMakeLists.txt          # Top-level build (SlamCore + CT_ICP libs)
├── cmake/                  # Custom CMake modules (SLAM_ADD_LIBRARY, SLAM_ADD_TEST, etc.)
├── include/
│   ├── SlamCore/           # SlamCore public headers
│   └── ct_icp/             # CT_ICP public headers
├── src/
│   ├── SlamCore/           # SlamCore sources (.cxx)
│   └── ct_icp/             # CT_ICP sources (.cpp)
├── command/                # CLI executables (run_odometry)
├── test/                   # GTest unit + integration + regression tests
├── config/                 # YAML config files for odometry/datasets
├── superbuild/             # ExternalProject dependency management
├── install/                # Superbuild output (libraries, cmake configs)
├── ros/                    # ROS1 wrapping (catkin — being replaced by ros2_ws/)
└── ros2_ws/                # ROS2 Humble workspace (see migration plan below)
```

## ROS2 Migration Plan (from ROS1)

**Goal**: Replace ROS1 (catkin) wrapping with ROS2 Humble (ament_cmake).
Core libraries (SlamCore, CT_ICP) require ZERO changes — all ROS code is isolated.

### Decisions
- Target: **ROS2 Humble** (Ubuntu 22.04)
- Replace ROS1 entirely (no dual support)
- Drop PCL dependency; publish `sensor_msgs::msg::PointCloud2` directly
- Use Python `.launch.py` files
- Class-based `rclcpp::Node` subclasses for all nodes
- Phase 1 scope: core odometry nodes only (slam_roscore utilities deferred)

### Target structure: `ros2_ws/src/`
```
ros2_ws/src/
├── ros2core/                        # Bridge library: slam types <-> ROS2 messages
│   ├── CMakeLists.txt               # ament_cmake, builds shared lib ROS2Core
│   ├── package.xml                  # format 3, deps: rclcpp, sensor/nav/geometry_msgs
│   ├── include/ROS2Core/
│   │   ├── pc2_conversion.h         # PointCloud2 <-> slam::PointCloud (NO PCL)
│   │   ├── nav_msgs_conversion.h    # Odometry/Imu <-> slam::Pose/ImuData
│   │   └── point_types.h            # XYZTPoint struct (plain C++, no PCL macros)
│   └── src/
│       ├── pc2_conversion.cxx
│       └── nav_msgs_conversion.cxx
└── ct_icp_odometry/                 # Odometry nodes (class-based rclcpp::Node)
    ├── CMakeLists.txt               # ament_cmake, 3 node executables
    ├── package.xml                  # deps: rclcpp, tf2_ros, sensor/nav_msgs, ros2core
    ├── src/
    │   ├── ct_icp_odometry_node.cxx # Main SLAM node (rclcpp::Node subclass)
    │   ├── ct_icp_dataset_node.cxx  # Dataset publisher node
    │   ├── ct_icp_evaluation_node.cxx
    │   └── utils.h                  # Helpers (PointCloud2 publishing, TF broadcasting)
    ├── launch/                      # Python .launch.py files
    └── params/                      # YAML configs (copy from ros/catkin_ws/.../params)
```

### Step-by-step implementation instructions

#### Step 1: Create ros2core package skeleton
Create `ros2_ws/src/ros2core/package.xml` (format 3, ament_cmake) with dependencies:
`rclcpp`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `std_msgs`, `builtin_interfaces`,
`tf2_eigen`, `Eigen3`. Create `CMakeLists.txt` using `ament_cmake`. The library must
find SlamCore and superbuild deps via the existing `SUPERBUILD_INSTALL_DIR` mechanism
(include `cmake/includes.cmake` and call `SLAM_INCLUDE_SUPERBUILD()`).

#### Step 2: Port pc2_conversion.h/.cxx
Copy from `ros/roscore/include/ROSCore/pc2_conversion.h` and `ros/roscore/src/pc2_conversion.cxx`.
Changes needed:
- `sensor_msgs::PointCloud2` -> `sensor_msgs::msg::PointCloud2`
- `sensor_msgs::PointField` -> `sensor_msgs::msg::PointField`
- `sensor_msgs::PointField::FLOAT32` -> `sensor_msgs::msg::PointField::FLOAT32` (same for all type constants)
- `sensor_msgs::PointCloud2Ptr` -> `sensor_msgs::msg::PointCloud2::SharedPtr`
- `sensor_msgs::PointCloud2ConstPtr` -> `sensor_msgs::msg::PointCloud2::ConstSharedPtr`
- Remove `boost::shared_ptr` usage, use `std::shared_ptr`
- **Add new function** `SlamPointCloudToROSCloud2()` that converts `slam::PointCloud` -> `sensor_msgs::msg::PointCloud2` (reverse direction needed because ROS2 cannot auto-publish PCL types). Reference the existing `ROSCloud2ToSlamPointCloudShallow/Deep` logic for field mapping.

#### Step 3: Port nav_msgs_conversion.h/.cxx
Copy from `ros/roscore/include/ROSCore/nav_msgs_conversion.h` and `.cxx`.
Changes needed:
- `nav_msgs::Odometry` -> `nav_msgs::msg::Odometry`
- `sensor_msgs::Imu` -> `sensor_msgs::msg::Imu`
- `geometry_msgs::Vector3` -> `geometry_msgs::msg::Vector3`
- `geometry_msgs::Quaternion` -> `geometry_msgs::msg::Quaternion`
- `ros::Time` -> `builtin_interfaces::msg::Time`
- Field rename: `.nsec` -> `.nanosec`
- `ROSTimeToSeconds()` / `SecondsToROSTime()`: update to use `.nanosec` field
- `#include <ros/ros.h>` -> remove (not needed; time comes from builtin_interfaces)

#### Step 4: Port point_types.h
Copy from `ros/roscore/include/ROSCore/point_types.h`.
Remove all PCL macros (`PCL_ADD_POINT4D`, `POINT_CLOUD_REGISTER_POINT_STRUCT`).
Keep `slam::XYZTPoint` and `slam::LidarPoint` as plain C++ structs with the same
fields (x, y, z, t/timestamp, intensity, ring, etc.). These are used for internal
conversion only.

#### Step 5: Create ct_icp_odometry package skeleton
Create `ros2_ws/src/ct_icp_odometry/package.xml` (format 3, ament_cmake) with deps:
`rclcpp`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `tf2_ros`, `tf2_eigen`, `ros2core`.
Create `CMakeLists.txt` that finds CT_ICP, SlamCore, ROS2Core via cmake paths and
builds 3 executables. Use `ament_target_dependencies()` for ROS2 deps.

#### Step 6: Port ct_icp_odometry_node.cxx (main SLAM node)
Source: `ros/catkin_ws/ct_icp_odometry/src/ct_icp_odometry_node.cxx`.
This is the most important node. Convert from free functions + globals to a class:
```cpp
class CtIcpOdometryNode : public rclcpp::Node { ... };
```
Key changes:
- `ros::init()` -> `rclcpp::init()`, spin a `std::make_shared<CtIcpOdometryNode>()`
- Parameters: `declare_parameter("config", "")` then `get_parameter("config").as_string()`
- Publisher: `create_publisher<nav_msgs::msg::Odometry>("/ct_icp/pose/odom", 5)`
- Subscriber: `create_subscription<sensor_msgs::msg::PointCloud2>("/ct_icp/pointcloud", 200, callback)`
- The callback receives `sensor_msgs::msg::PointCloud2::SharedPtr` (std, not boost)
- Use ROS2Core `ROSCloud2ToSlamPointCloud*()` for input conversion
- Use ROS2Core `SlamPointCloudToROSCloud2()` for output (replaces PCL auto-publish)
- `ROS_INFO_STREAM(x)` -> `RCLCPP_INFO_STREAM(this->get_logger(), x)`
- `tf2_ros::TransformBroadcaster` API is nearly identical in ROS2
- `ros::shutdown()` -> `rclcpp::shutdown()`
- Remove `boost::bind`, use lambdas or `std::bind`
- Remove `pcl_ros/point_cloud.h` and `pcl_conversions/pcl_conversions.h` includes

#### Step 7: Port utils.h
Source: `ros/catkin_ws/ct_icp_odometry/src/utils.h`.
- `WPointsToROSPointCloud()`: return `sensor_msgs::msg::PointCloud2` instead of `pcl::PointCloud`. Use ROS2Core `SlamPointCloudToROSCloud2()` or manually fill PointCloud2 fields from the WPoint3D vector.
- `RegisterPointCloudPublisher()`: return `rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr`
- `PublishPoints()`: accept rclcpp publisher, publish PointCloud2
- `SlamPoseToROSOdometry()`: use `nav_msgs::msg::Odometry`, `rclcpp::Time`
- `TransformFromPose()`: `tf2::eigenToTransform()` works the same in ROS2
- Remove all PCL includes

#### Step 8: Port ct_icp_dataset_node.cxx
Source: `ros/catkin_ws/ct_icp_odometry/src/ct_icp_dataset_node.cxx`.
Same class-based pattern. This node publishes pointclouds from disk datasets.
- Loop with `rclcpp::Rate` instead of `ros::Rate` / manual sleep
- `ros::ok()` -> `rclcpp::ok()`
- Publish `sensor_msgs::msg::PointCloud2` instead of `pcl::PointCloud<XYZTPoint>`
- `pcl_conversions::toPCL(ros::Time())` -> use `rclcpp::Clock().now()` for stamps

#### Step 9: Port ct_icp_evaluation_node.cxx
Source: `ros/catkin_ws/ct_icp_odometry/src/ct_icp_evaluation_node.cxx`.
Same class-based pattern. Note: this node uses `boost::bind()` with extra args for
the subscription callback — replace with a lambda capturing the extra context.
Remove the `slam_roscore::monitor_entry` publisher for now (deferred to phase 2).

#### Step 10: Create Python launch files
Convert `ros/catkin_ws/ct_icp_odometry/launch/*.launch` XML to Python `.launch.py`.
Key conversions:
- `$(find pkg)` -> `get_package_share_directory('pkg')`
- `<arg name="x" default="y">` -> `DeclareLaunchArgument('x', default_value='y')`
- `<node pkg="p" type="t" name="n">` -> `Node(package='p', executable='t', name='n')`
- `<param name="k" value="v">` -> `parameters=[{'k': v}]`
- `<remap from="a" to="b">` -> `remappings=[('a', 'b')]`
- `rosbag play` -> `ros2 bag play` (use `ExecuteProcess`)
Priority launch files: `ct_icp_slam.launch.py`, `ct_icp_on_dataset.launch.py`, `ct_icp_on_rosbag.launch.py`

#### Step 11: Copy config/params files
Copy `ros/catkin_ws/ct_icp_odometry/params/` to `ros2_ws/src/ct_icp_odometry/params/`.
The YAML algorithm configs are NOT ROS-dependent and work unchanged.
The rviz config (`ct_icp_odometry.rviz`) needs minor updates: rviz1 plugin names
to rviz2 equivalents (e.g., `rviz/PointCloud2` -> `rviz_default_plugins/PointCloud2`).

#### Step 12: Build and test
```bash
# Prerequisites: CT_ICP + SlamCore already built and installed (Phase 1+2 from top)
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --cmake-args -DSUPERBUILD_INSTALL_DIR=<absolute-path-to-install-dir>
source install/setup.bash

# Test with a dataset
ros2 launch ct_icp_odometry ct_icp_on_dataset.launch.py \
    dataset:=kitti root_path:=/path/to/kitti sequence:=00

# Test with a rosbag
ros2 launch ct_icp_odometry ct_icp_on_rosbag.launch.py \
    rosbag:=/path/to/bag.db3 config:=/path/to/config.yaml
```

### Common pitfalls for agents implementing this migration
1. **Do NOT modify** anything in `src/`, `include/`, `cmake/`, `test/`, `command/`, or `superbuild/`. The core libraries are ROS-free.
2. The `ros::Time` field `.nsec` is renamed to `.nanosec` in ROS2 — easy to miss.
3. ROS2 message types are under `::msg::` sub-namespace (e.g., `sensor_msgs::msg::PointCloud2`).
4. ROS2 uses `std::shared_ptr` everywhere, not `boost::shared_ptr`.
5. `pcl_ros` auto-serialization of `pcl::PointCloud<T>` does NOT work in ROS2. You must manually convert to `sensor_msgs::msg::PointCloud2`.
6. `find_package(catkin ...)` is replaced by individual `find_package()` calls per dep.
7. The `slam_roscore` custom message (`monitor_entry.msg`) is deferred — do not add it in phase 1. Remove monitor publishers from odometry/evaluation nodes for now.
8. Header guards for new files should follow the existing `MODULE_FILENAME_H` pattern.
9. Use 4-space indentation in all new C++ code to match the existing codebase.
