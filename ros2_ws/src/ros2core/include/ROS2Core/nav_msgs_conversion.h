#ifndef ROS2CORE_NAV_MSGS_CONVERSION_H
#define ROS2CORE_NAV_MSGS_CONVERSION_H

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <SlamCore/types.h>
#include <SlamCore/imu.h>

namespace slam {

    inline double ROSTimeToSeconds(const builtin_interfaces::msg::Time &time) {
        return double(time.sec) + (1.e-9 * double(time.nanosec));
    }

    inline builtin_interfaces::msg::Time SecondsToROSTime(double secs) {
        builtin_interfaces::msg::Time time;
        time.sec = int32_t(secs);
        time.nanosec = uint32_t((secs - double(time.sec)) * 1.e9);
        return time;
    }

    inline geometry_msgs::msg::Vector3 EigenToROSVec3(const Eigen::Vector3d &xyz) {
        geometry_msgs::msg::Vector3 vec;
        vec.x = xyz.x();
        vec.y = xyz.y();
        vec.z = xyz.z();
        return vec;
    }

    inline geometry_msgs::msg::Quaternion EigenToROSQuat(const Eigen::Quaterniond &quat) {
        geometry_msgs::msg::Quaternion vec;
        vec.x = quat.x();
        vec.y = quat.y();
        vec.z = quat.z();
        vec.w = quat.w();
        return vec;
    }

    inline Eigen::Vector3d ROSToEigenVec3(const geometry_msgs::msg::Vector3 &xyz) {
        return {xyz.x, xyz.y, xyz.z};
    }

    inline Eigen::Quaterniond ROSToEigenQuat(const geometry_msgs::msg::Quaternion &quat) {
        Eigen::Quaterniond _quat;
        _quat.x() = quat.x;
        _quat.y() = quat.y;
        _quat.z() = quat.z;
        _quat.w() = quat.w;
        return _quat;
    }


    // Converts an odometry message to a slam::Pose
    slam::Pose ROSOdometryToPose(const nav_msgs::msg::Odometry &odometry, slam::frame_id_t frame_id = 0);

    // Converts a slam::Pose to an odometry message with SE3 input
    nav_msgs::msg::Odometry SE3ToROSOdometry(const slam::SE3 &pose,
                                              const std::string &src_frame_id = "odom",
                                              const std::string &dest_frame_id = "base_link");

    // Converts a slam::Pose to an odometry message
    nav_msgs::msg::Odometry PoseToROSOdometry(const slam::Pose &pose,
                                               const std::string &src_frame_id = "odom",
                                               const std::string &dest_frame_id = "base_link");

    // Converts slam::ImuData to a ROS2 Imu message
    sensor_msgs::msg::Imu SlamToROSImu(const slam::ImuData &imu);

    // Converts a ROS2 Imu message to slam::ImuData
    slam::ImuData ROSToSlamImu(const sensor_msgs::msg::Imu &imu);

} // namespace slam

#endif // ROS2CORE_NAV_MSGS_CONVERSION_H
