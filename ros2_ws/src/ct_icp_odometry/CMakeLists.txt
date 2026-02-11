cmake_minimum_required(VERSION 3.14)
project(ct_icp_odometry)

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
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(ros2core REQUIRED)  # IMPORTANT: Find ros2core BEFORE using CT_ICP

# Find CT_ICP and SlamCore (pre-installed libraries)
# Assumes SUPERBUILD_INSTALL_DIR is set
if (NOT SUPERBUILD_INSTALL_DIR)
    set(SUPERBUILD_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../install")
    get_filename_component(SUPERBUILD_INSTALL_DIR ${SUPERBUILD_INSTALL_DIR} ABSOLUTE)
    message(STATUS "[ct_icp_odometry] SUPERBUILD_INSTALL_DIR not specified. Trying: ${SUPERBUILD_INSTALL_DIR}")
endif ()

# Include superbuild cmake config to set up all dependencies
set(SUPERBUILD_IMPORT "${SUPERBUILD_INSTALL_DIR}/superbuild_import.cmake")
if (EXISTS ${SUPERBUILD_IMPORT})
    message(STATUS "[ct_icp_odometry] Including superbuild_import.cmake from ${SUPERBUILD_INSTALL_DIR}")
    include(${SUPERBUILD_IMPORT})
else()
    message(FATAL_ERROR "[ct_icp_odometry] superbuild_import.cmake not found at ${SUPERBUILD_IMPORT}")
endif()

# Now find SlamCore and CT_ICP (dependencies already set up by superbuild_import.cmake)
set(CT_ICP_CMAKE_DIR ${SUPERBUILD_INSTALL_DIR}/CT_ICP/lib/cmake)
set(SlamCore_DIR ${CT_ICP_CMAKE_DIR})

find_package(SlamCore REQUIRED PATHS ${CT_ICP_CMAKE_DIR} NO_DEFAULT_PATH)
find_package(CT_ICP REQUIRED PATHS ${CT_ICP_CMAKE_DIR} NO_DEFAULT_PATH)

# Build ct_icp_odometry_node executable
add_executable(ct_icp_odometry_node
    src/ct_icp_odometry_node.cxx
)

target_include_directories(ct_icp_odometry_node
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../ros2core/include
)

ament_target_dependencies(ct_icp_odometry_node
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    tf2_ros
    tf2_eigen
    ros2core
)

target_link_libraries(ct_icp_odometry_node
    Slam::SlamCore
    Slam::CT_ICP
    ros2core::ROS2Core
)

# Build ct_icp_dataset_node executable
add_executable(ct_icp_dataset_node
    src/ct_icp_dataset_node.cxx
)

target_include_directories(ct_icp_dataset_node
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../ros2core/include
)

ament_target_dependencies(ct_icp_dataset_node
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    tf2_ros
    tf2_eigen
    ros2core
)

target_link_libraries(ct_icp_dataset_node
    Slam::SlamCore
    Slam::CT_ICP
    ros2core::ROS2Core
)

# Build ct_icp_evaluation_node executable
add_executable(ct_icp_evaluation_node
    src/ct_icp_evaluation_node.cxx
)

target_include_directories(ct_icp_evaluation_node
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../ros2core/include
)

ament_target_dependencies(ct_icp_evaluation_node
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    tf2_ros
    tf2_eigen
    ros2core
)

target_link_libraries(ct_icp_evaluation_node
    Slam::SlamCore
    Slam::CT_ICP
    ros2core::ROS2Core
)

# Install executables
install(TARGETS
    ct_icp_odometry_node
    ct_icp_dataset_node
    ct_icp_evaluation_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# Install config/params files
install(DIRECTORY params
    DESTINATION share/${PROJECT_NAME}
)

# Export dependencies for downstream packages
ament_export_dependencies(
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    tf2_ros
    tf2_eigen
    ros2core
    SlamCore
    CT_ICP
)

ament_package()
