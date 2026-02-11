#include "ROS2Core/nav_msgs_conversion.h"

#include <chrono>
#include <Eigen/Geometry>

namespace slam {

    /* ----------------------------------------------------------------------------------------------------------- */
    slam::Pose ROSOdometryToPose(const nav_msgs::msg::Odometry &odometry, slam::frame_id_t frame_id) {
        slam::Pose pose;
        pose.pose.quat.x() = odometry.pose.pose.orientation.x;
        pose.pose.quat.y() = odometry.pose.pose.orientation.y;
        pose.pose.quat.z() = odometry.pose.pose.orientation.z;
        pose.pose.quat.w() = odometry.pose.pose.orientation.w;

        pose.pose.tr.x() = odometry.pose.pose.position.x;
        pose.pose.tr.y() = odometry.pose.pose.position.y;
        pose.pose.tr.z() = odometry.pose.pose.position.z;
        pose.dest_frame_id = frame_id;
        pose.dest_timestamp = double(odometry.header.stamp.sec) + double(odometry.header.stamp.nanosec) / 1.e9;

        return pose;
    }

    /* ----------------------------------------------------------------------------------------------------------- */
    nav_msgs::msg::Odometry
    PoseToROSOdometry(const Pose &pose, const std::string &src_frame_id, const std::string &dest_frame_id) {
        nav_msgs::msg::Odometry odometry;
        odometry.pose.pose.orientation.x = pose.pose.quat.x();
        odometry.pose.pose.orientation.y = pose.pose.quat.y();
        odometry.pose.pose.orientation.z = pose.pose.quat.z();
        odometry.pose.pose.orientation.w = pose.pose.quat.w();

        odometry.pose.pose.position.x = pose.pose.tr.x();
        odometry.pose.pose.position.y = pose.pose.tr.y();
        odometry.pose.pose.position.z = pose.pose.tr.z();

        odometry.header.stamp.sec = int32_t(pose.dest_timestamp);
        odometry.header.stamp.nanosec = uint32_t((pose.dest_timestamp - double(odometry.header.stamp.sec)) * 1.e9);
        odometry.header.frame_id = src_frame_id;
        odometry.child_frame_id = dest_frame_id;

        return odometry;
    }

    /* ----------------------------------------------------------------------------------------------------------- */
    sensor_msgs::msg::Imu SlamToROSImu(const ImuData &imu) {
        sensor_msgs::msg::Imu ros_imu;
        ros_imu.header.stamp.sec = int32_t(imu.time_seconds);
        ros_imu.header.stamp.nanosec = uint32_t((imu.time_seconds - double(ros_imu.header.stamp.sec)) * 1.e9);

        if (imu.state & slam::ImuData::ORIENTATION) {
            ros_imu.orientation = EigenToROSQuat(imu.orientation);
        }

        if (imu.state & slam::ImuData::ANGULAR_VELOCITY) {
            ros_imu.angular_velocity = EigenToROSVec3(imu.angular_velocity);
        }

        if (imu.state & slam::ImuData::LINEAR_ACCELERATION) {
            ros_imu.linear_acceleration = EigenToROSVec3(imu.linear_acceleration);
        }

        // TODO The same for Orientation covariance matrices

        return ros_imu;
    }

    /* ----------------------------------------------------------------------------------------------------------- */
    slam::ImuData ROSToSlamImu(const sensor_msgs::msg::Imu &imu) {
        slam::ImuData slam_imu;
        slam_imu.time_seconds = ROSTimeToSeconds(imu.header.stamp);
        slam_imu.state = slam::ImuData::ALL_DATA_POINTS;
        slam_imu.orientation = ROSToEigenQuat(imu.orientation);
        slam_imu.angular_velocity = ROSToEigenVec3(imu.angular_velocity);
        slam_imu.linear_acceleration = ROSToEigenVec3(imu.linear_acceleration);
        return slam_imu;
    }

    /* ----------------------------------------------------------------------------------------------------------- */
    nav_msgs::msg::Odometry SE3ToROSOdometry(const SE3 &pose, const std::string &src_frame_id, 
                                             const std::string &dest_frame_id) {
        nav_msgs::msg::Odometry odometry;
        // SE3 already stores the quaternion
        odometry.pose.pose.orientation.x = pose.quat.x();
        odometry.pose.pose.orientation.y = pose.quat.y();
        odometry.pose.pose.orientation.z = pose.quat.z();
        odometry.pose.pose.orientation.w = pose.quat.w();
        
        // Translation vector
        odometry.pose.pose.position.x = pose.tr.x();
        odometry.pose.pose.position.y = pose.tr.y();
        odometry.pose.pose.position.z = pose.tr.z();
        
        // Timestamp (use current time if not available)
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        auto nanos = duration - seconds;
        odometry.header.stamp.sec = int32_t(seconds.count());
        odometry.header.stamp.nanosec = uint32_t(std::chrono::duration_cast<std::chrono::nanoseconds>(nanos).count());
        
        odometry.header.frame_id = src_frame_id;
        odometry.child_frame_id = dest_frame_id;
        
        return odometry;
    }

} // namespace slam
