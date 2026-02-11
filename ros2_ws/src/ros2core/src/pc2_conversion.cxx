#include "ROS2Core/pc2_conversion.h"

#include <cstring>

namespace slam {

    /* ----------------------------------------------------------------------------------------------------------- */
    PROPERTY_TYPE ROSPointFieldDTypeToSlamDType(int ros_datatype) {
        switch (ros_datatype) {
            case sensor_msgs::msg::PointField::INT8:
                return slam::INT8;
            case sensor_msgs::msg::PointField::UINT8:
                return slam::UINT8;
            case sensor_msgs::msg::PointField::INT16:
                return slam::INT16;
            case sensor_msgs::msg::PointField::UINT16:
                return slam::UINT16;
            case sensor_msgs::msg::PointField::INT32:
                return slam::INT32;
            case sensor_msgs::msg::PointField::UINT32:
                return slam::UINT32;
            case sensor_msgs::msg::PointField::FLOAT32:
                return slam::FLOAT32;
            case sensor_msgs::msg::PointField::FLOAT64:
                return slam::FLOAT64;
            default:
                throw std::runtime_error("Incompatible ROS PointField datatype");
        }
    }

    /* ----------------------------------------------------------------------------------------------------------- */
    slam::ItemSchema::Builder SchemaBuilderFromCloud2(const sensor_msgs::msg::PointCloud2 &cloud) {
        slam::ItemSchema::Builder builder(cloud.point_step);
        builder.AddElement("properties", 0);

        std::vector<sensor_msgs::msg::PointField> fields(cloud.fields);
        std::sort(fields.begin(), fields.end(),
                  [](const sensor_msgs::msg::PointField &lhs, const sensor_msgs::msg::PointField &rhs) {
                      return lhs.offset < rhs.offset;
                  });

        int expected_offset = 0;
        int padding_idx = 0;

        bool x_property = false;
        int offset_of_x = 0;
        int dtype_x = -1;

        for (auto &field: fields) {
            if (field.name == "x") {
                x_property = true;
                offset_of_x = field.offset;
                dtype_x = field.datatype;
            }
            auto data_size = slam::PropertySize(slam::ROSPointFieldDTypeToSlamDType(field.datatype));
            if (field.offset != expected_offset) {
                builder.AddScalarProperty<char>("properties", "padding_" + std::to_string(padding_idx++),
                                               expected_offset, field.offset - expected_offset);
            }

            builder.AddProperty("properties", std::string(field.name),
                               slam::ROSPointFieldDTypeToSlamDType(field.datatype),
                               field.offset, 1);
            expected_offset = field.offset + data_size;
        }
        if (expected_offset != cloud.point_step)
            builder.AddScalarProperty<char>("properties", "padding_" + std::to_string(padding_idx++),
                                           expected_offset, cloud.point_step - expected_offset);

        if (x_property) {
            auto dtype = slam::ROSPointFieldDTypeToSlamDType(dtype_x);
            auto dtype_size = slam::PropertySize(dtype);
            builder.AddElement("vertex", offset_of_x)
                   .AddProperty("vertex", "x", dtype, 0, 1)
                   .AddProperty("vertex", "y", dtype, dtype_size, 1)
                   .AddProperty("vertex", "z", dtype, 2 * dtype_size, 1);
        }

        return builder;
    }

    /* ----------------------------------------------------------------------------------------------------------- */
    slam::PointCloudPtr ROSCloud2ToSlamPointCloudShallow(const sensor_msgs::msg::PointCloud2 &cloud,
                                                         std::shared_ptr<PointCloud2PtrWrapper> pointer) {
        auto cloud2_builder = slam::SchemaBuilderFromCloud2(cloud);
        auto collection2 = slam::BufferCollection::Factory(std::make_unique<slam::BufferWrapper>(
                cloud2_builder.Build(),
                (char *) (&cloud.data[0]),
                cloud.width * cloud.height,
                cloud2_builder.GetItemSize(),
                std::dynamic_pointer_cast<BufferWrapper::SmartDataPtrWrapper>(pointer)));
        return std::make_shared<slam::PointCloud>(std::move(collection2), "vertex");
    }

    /* ----------------------------------------------------------------------------------------------------------- */
    slam::PointCloudPtr ROSCloud2ToSlamPointCloudShallow(sensor_msgs::msg::PointCloud2::SharedPtr &cloud) {
        return ROSCloud2ToSlamPointCloudShallow(*cloud, std::make_shared<PointCloud2PtrWrapper>(cloud));
    }

    /* ----------------------------------------------------------------------------------------------------------- */
    slam::PointCloudPtr ROSCloud2ToSlamPointCloudDeep(const sensor_msgs::msg::PointCloud2 &cloud) {
        auto cloud2_builder = slam::SchemaBuilderFromCloud2(cloud);
        auto vector_buffer_ptr = std::make_unique<slam::VectorBuffer>(
                cloud2_builder.Build(), cloud2_builder.GetItemSize());
        auto num_items = cloud.width * cloud.height;
        vector_buffer_ptr->Reserve(num_items);
        vector_buffer_ptr->InsertItems(num_items, reinterpret_cast<const char *>(&cloud.data[0]));
        auto collection2 = slam::BufferCollection::Factory(std::move(vector_buffer_ptr));
        return std::make_shared<slam::PointCloud>(std::move(collection2), "vertex");
    }

    /* ----------------------------------------------------------------------------------------------------------- */
    slam::PointCloudPtr ROSCloud2ToSlamPointCloudShallow(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud) {
        return ROSCloud2ToSlamPointCloudShallow(*cloud, std::make_shared<PointCloud2PtrWrapper>(cloud));
    }

    /* ----------------------------------------------------------------------------------------------------------- */
    sensor_msgs::msg::PointCloud2 SlamPointCloudToROSCloud2(const slam::PointCloud &slam_cloud,
                                                             const std::string &frame_id) {
        sensor_msgs::msg::PointCloud2 ros_cloud;
        
        // Set frame ID and timestamp
        ros_cloud.header.frame_id = frame_id;
        ros_cloud.header.stamp.sec = 0;
        ros_cloud.header.stamp.nanosec = 0;

        // Get point cloud collection and schema
        const auto &collection = slam_cloud.GetCollection();
        const auto &item_info = collection.GetItemInfo(0);
        const auto &schema = item_info.item_schema;
        size_t num_points = collection.NumItemsPerBuffer();

        // Set dimensions
        ros_cloud.width = num_points;
        ros_cloud.height = 1;
        ros_cloud.is_dense = true;
        ros_cloud.point_step = item_info.item_size;
        ros_cloud.row_step = ros_cloud.point_step * ros_cloud.width;

        // Build fields array from schema
        const auto &element_names = schema.GetElementNames();
        for (const auto &element_name : element_names) {
            const auto &element_info = schema.GetElementInfo(element_name);
            for (const auto &property : element_info.properties) {
                sensor_msgs::msg::PointField field;
                field.name = property.property_name;
                field.offset = property.offset_in_elem;
                field.count = property.dimension;

                // Map slam PROPERTY_TYPE to ROS PointField datatype
                switch (property.type) {
                    case slam::PROPERTY_TYPE::INT8:
                        field.datatype = sensor_msgs::msg::PointField::INT8;
                        break;
                    case slam::PROPERTY_TYPE::UINT8:
                        field.datatype = sensor_msgs::msg::PointField::UINT8;
                        break;
                    case slam::PROPERTY_TYPE::INT16:
                        field.datatype = sensor_msgs::msg::PointField::INT16;
                        break;
                    case slam::PROPERTY_TYPE::UINT16:
                        field.datatype = sensor_msgs::msg::PointField::UINT16;
                        break;
                    case slam::PROPERTY_TYPE::INT32:
                        field.datatype = sensor_msgs::msg::PointField::INT32;
                        break;
                    case slam::PROPERTY_TYPE::UINT32:
                        field.datatype = sensor_msgs::msg::PointField::UINT32;
                        break;
                    case slam::PROPERTY_TYPE::FLOAT32:
                        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                        break;
                    case slam::PROPERTY_TYPE::FLOAT64:
                        field.datatype = sensor_msgs::msg::PointField::FLOAT64;
                        break;
                    default:
                        continue;
                }
                ros_cloud.fields.push_back(field);
            }
        }

        // Copy point cloud data
        // Use a View of the first element to access the underlying data
        if (!element_names.empty()) {
            const auto view = collection.element<char>(element_names[0]);
            size_t total_data_size = view.size();
            
            ros_cloud.data.resize(total_data_size);
            std::memcpy(ros_cloud.data.data(), view.item_buffer.view_data_ptr, total_data_size);
        }

        return ros_cloud;
    }

    /* ----------------------------------------------------------------------------------------------------------- */
    sensor_msgs::msg::PointCloud2 WPointsToROSCloud2(const std::vector<slam::WPoint3D> &points,
                                                      const std::string &frame_id) {
        sensor_msgs::msg::PointCloud2 cloud;
        
        cloud.header.frame_id = frame_id;
        cloud.height = 1;
        cloud.width = points.size();
        cloud.is_bigendian = false;
        cloud.is_dense = true;
        
        // Define point field layout (x, y, z, intensity)
        sensor_msgs::msg::PointField field_x;
        field_x.name = "x";
        field_x.offset = 0;
        field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_x.count = 1;
        cloud.fields.push_back(field_x);
        
        sensor_msgs::msg::PointField field_y;
        field_y.name = "y";
        field_y.offset = 4;
        field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_y.count = 1;
        cloud.fields.push_back(field_y);
        
        sensor_msgs::msg::PointField field_z;
        field_z.name = "z";
        field_z.offset = 8;
        field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_z.count = 1;
        cloud.fields.push_back(field_z);
        
        sensor_msgs::msg::PointField field_intensity;
        field_intensity.name = "intensity";
        field_intensity.offset = 12;
        field_intensity.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_intensity.count = 1;
        cloud.fields.push_back(field_intensity);
        
        cloud.point_step = 16;  // 4 * float32
        cloud.row_step = cloud.point_step * cloud.width;
        
        // Allocate and fill data
        cloud.data.resize(cloud.row_step);
        
        for (size_t i = 0; i < points.size(); ++i) {
            float *ptr = reinterpret_cast<float *>(cloud.data.data() + i * cloud.point_step);
            // Use world_point which is the Eigen::Vector3d
            ptr[0] = static_cast<float>(points[i].world_point.x());
            ptr[1] = static_cast<float>(points[i].world_point.y());
            ptr[2] = static_cast<float>(points[i].world_point.z());
            // Use timestamp as intensity (or could use raw_point values)
            ptr[3] = static_cast<float>(points[i].raw_point.timestamp);
        }
        
        return cloud;
    }

    /* ----------------------------------------------------------------------------------------------------------- */

} // namespace slam
