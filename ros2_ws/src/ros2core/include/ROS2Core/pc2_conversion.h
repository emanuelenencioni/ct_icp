#ifndef ROS2CORE_PC2_CONVERSION_H
#define ROS2CORE_PC2_CONVERSION_H

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <SlamCore/pointcloud.h>

namespace slam {

    // Converts ROS2 PointField datatype to Slam Data types
    slam::PROPERTY_TYPE ROSPointFieldDTypeToSlamDType(int ros_datatype);

    // Builds a default Item Schema from a PointCloud2 fields layout
    // Note: The layout in memory might contain padding (unused bytes) which are added as padding_<i> char properties
    slam::ItemSchema::Builder SchemaBuilderFromCloud2(const sensor_msgs::msg::PointCloud2 &cloud);

    // A PointCloud2 Pointer Wrapper
    // It allows a BufferWrapper to keep a shared pointer to the PointCloud2, ensuring that no data is deallocated prior
    // to the destruction of the BufferWrapper
    struct PointCloud2PtrWrapper : slam::BufferWrapper::SmartDataPtrWrapper {

        ~PointCloud2PtrWrapper() override = default;

        explicit PointCloud2PtrWrapper(sensor_msgs::msg::PointCloud2::SharedPtr _ptr) : ptr(_ptr) {}

        explicit PointCloud2PtrWrapper(sensor_msgs::msg::PointCloud2::ConstSharedPtr _ptr) : const_ptr(_ptr) {}

        sensor_msgs::msg::PointCloud2::SharedPtr ptr = nullptr;
        sensor_msgs::msg::PointCloud2::ConstSharedPtr const_ptr = nullptr;
    };

    // Returns a shallow copy of a point cloud from ROS2 PointCloud2
    // Note: Once the data of `cloud` is deleted, the copy will
    slam::PointCloudPtr ROSCloud2ToSlamPointCloudShallow(const sensor_msgs::msg::PointCloud2 &cloud,
                                                         std::shared_ptr<PointCloud2PtrWrapper> pointer = nullptr);

    // Returns a shallow copy of a point cloud from ROS2 PointCloud2
    // Note: It will add a reference to the cloud pointer, in order to keep it alive
    slam::PointCloudPtr ROSCloud2ToSlamPointCloudShallow(sensor_msgs::msg::PointCloud2::SharedPtr &cloud);

    // Returns a shallow copy of a point cloud from ROS2 PointCloud2
    // Note: It will add a reference to the cloud pointer, in order to keep it alive
    slam::PointCloudPtr ROSCloud2ToSlamPointCloudShallow(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud);

    // Returns a deep copy of a point cloud from ROS2 PointCloud2
    // Note: The data layout will be the same as the input cloud
    slam::PointCloudPtr ROSCloud2ToSlamPointCloudDeep(const sensor_msgs::msg::PointCloud2 &cloud);

    // Converts a slam PointCloud to ROS2 PointCloud2 message
    // This is used when publishing point clouds (ROS2 does not support auto-serialization of custom types)
    sensor_msgs::msg::PointCloud2 SlamPointCloudToROSCloud2(const slam::PointCloud &slam_cloud,
                                                             const std::string &frame_id = "base_link");

    // Converts a vector of WPoint3D to ROS2 PointCloud2 message (helper for ICP registration results)
    sensor_msgs::msg::PointCloud2 WPointsToROSCloud2(const std::vector<slam::WPoint3D> &points,
                                                      const std::string &frame_id = "base_link");

} // namespace slam

#endif // ROS2CORE_PC2_CONVERSION_H
