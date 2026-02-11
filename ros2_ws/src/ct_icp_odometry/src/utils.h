#ifndef CT_ICP_ODOMETRY_UTILS_H
#define CT_ICP_ODOMETRY_UTILS_H

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Geometry>

#include <SlamCore/types.h>
#include <ROS2Core/pc2_conversion.h>
#include <ROS2Core/nav_msgs_conversion.h>

namespace ct_icp {

    enum TIME_UNIT {
        SECONDS,
        NANO_SECONDS,
        MILLI_SECONDS
    };

    inline TIME_UNIT TimeUnitFromNode(YAML::Node &node, const std::string &key) {
        TIME_UNIT unit;
        slam::config::FindEnumOption(node, (int &) unit, key, {
                {"NANO_SECONDS",  ct_icp::NANO_SECONDS},
                {"MILLI_SECONDS", ct_icp::MILLI_SECONDS},
                {"SECONDS",       ct_icp::SECONDS}
        });
        return unit;
    }

    // Returns the multiplication factor to apply to values expressed in unit to convert them to seconds
    inline double ToSecondsFactor(TIME_UNIT unit) {
        switch (unit) {
            case SECONDS:
                return 1.;
            case MILLI_SECONDS:
                return 1.e-3;
            case NANO_SECONDS:
                return 1.e-9;
            default:
                return -1.;
        }
    }

    // The Frame Ids on which the published topics depend
    const std::string main_frame_id = "odom";
    const std::string child_frame_id = "base_link";

    // Converts a ct_icp Point Cloud to a ROS2 PointCloud2 message
    inline sensor_msgs::msg::PointCloud2 WPointsToROSPointCloud(
            const std::vector<slam::WPoint3D> &points,
            const std::string &frame_id,
            const builtin_interfaces::msg::Time &stamp) {
        return slam::WPointsToROSCloud2(points, frame_id);
    }

    inline nav_msgs::msg::Odometry SlamPoseToROSOdometry(
            const slam::SE3 &pose,
            const builtin_interfaces::msg::Time &stamp,
            const std::string &_frame_id = ct_icp::main_frame_id,
            const std::string &_child_frame_id = ct_icp::child_frame_id) {
        return slam::SE3ToROSOdometry(pose, _frame_id, _child_frame_id);
    }

    inline geometry_msgs::msg::TransformStamped TransformFromPose(
            const slam::SE3 &pose,
            const builtin_interfaces::msg::Time &stamp,
            const std::string &_frame_id = ct_icp::main_frame_id,
            const std::string &_child_frame_id = ct_icp::child_frame_id) {
        // Convert SE3 pose to transform stamped
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = _frame_id;
        transform.child_frame_id = _child_frame_id;
        
        transform.transform.translation.x = pose.tr.x();
        transform.transform.translation.y = pose.tr.y();
        transform.transform.translation.z = pose.tr.z();
        
        // Use the quaternion directly from SE3
        transform.transform.rotation.x = pose.quat.x();
        transform.transform.rotation.y = pose.quat.y();
        transform.transform.rotation.z = pose.quat.z();
        transform.transform.rotation.w = pose.quat.w();
        
        return transform;
    }

} // namespace ct_icp

#endif //CT_ICP_ODOMETRY_UTILS_H
