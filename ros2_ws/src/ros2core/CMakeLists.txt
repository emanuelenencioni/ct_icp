cmake_minimum_required(VERSION 3.14)
project(ros2core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

# Find CT_ICP and SlamCore (pre-installed libraries)
# Assumes SUPERBUILD_INSTALL_DIR is set
if (NOT SUPERBUILD_INSTALL_DIR)
    set(SUPERBUILD_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../install")
    get_filename_component(SUPERBUILD_INSTALL_DIR ${SUPERBUILD_INSTALL_DIR} ABSOLUTE)
    message(STATUS "[ros2core] SUPERBUILD_INSTALL_DIR not specified. Trying: ${SUPERBUILD_INSTALL_DIR}")
endif ()

# Include superbuild cmake config to set up all dependencies
set(SUPERBUILD_IMPORT "${SUPERBUILD_INSTALL_DIR}/superbuild_import.cmake")
if (EXISTS ${SUPERBUILD_IMPORT})
    message(STATUS "[ros2core] Including superbuild_import.cmake from ${SUPERBUILD_INSTALL_DIR}")
    include(${SUPERBUILD_IMPORT})
else()
    message(FATAL_ERROR "[ros2core] superbuild_import.cmake not found at ${SUPERBUILD_IMPORT}")
endif()

# Now find SlamCore (dependencies already set up by superbuild_import.cmake)
set(CT_ICP_CMAKE_DIR ${SUPERBUILD_INSTALL_DIR}/CT_ICP/lib/cmake)
set(SlamCore_DIR ${CT_ICP_CMAKE_DIR})

find_package(SlamCore REQUIRED PATHS ${CT_ICP_CMAKE_DIR} NO_DEFAULT_PATH)

# Build ROS2Core library
add_library(ROS2Core SHARED
    src/pc2_conversion.cxx
    src/nav_msgs_conversion.cxx
)

target_include_directories(ROS2Core
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ROS2Core
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    builtin_interfaces
    tf2_eigen
    Eigen3
    OpenMP
)

target_link_libraries(ROS2Core Slam::SlamCore)

# Install library
install(TARGETS ROS2Core
    EXPORT export_ROS2Core
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib
)

# Install headers
install(DIRECTORY include/ROS2Core
    DESTINATION include
)

# Export library target and dependencies for downstream packages
ament_export_libraries(ROS2Core)
ament_export_targets(export_ROS2Core HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    builtin_interfaces
    tf2_eigen
    Eigen3
    OpenMP
)

ament_package()
