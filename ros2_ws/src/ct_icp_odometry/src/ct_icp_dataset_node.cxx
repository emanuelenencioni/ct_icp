#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Geometry>

#include <SlamCore/timer.h>
#include <ROS2Core/pc2_conversion.h>
#include <ROS2Core/nav_msgs_conversion.h>
#include <ct_icp/dataset.h>

/* ------------------------------------------------------------------------------------------------------------------ */
class CtIcpDatasetNode : public rclcpp::Node {
public:
    CtIcpDatasetNode() : Node("ct_icp_dataset_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing CT-ICP Dataset Node");

        // Load parameters
        auto dataset_param = this->declare_parameter<std::string>("dataset", "kitti");
        auto root_path = this->declare_parameter<std::string>("root_path", "");
        auto sequence = this->declare_parameter<std::string>("sequence", "");
        frequency_hz_ = this->declare_parameter<double>("frequency_hz", 10.0);

        // Create publishers
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ct_icp/pointcloud", 1);
        gt_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/ct_icp/gt_pose/odom", 1);

        // Initialize TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());

        // Initialize dataset
        InitializeDataset(dataset_param, root_path, sequence);

        // Log parameters
        double num_ms_per_frame = 1000.0 / frequency_hz_;
        RCLCPP_INFO_STREAM(this->get_logger(),
                "Sending frames at a frequency of " << frequency_hz_
                << " (Each frame is " << num_ms_per_frame << " ms)");

        // Start publishing frames in a separate thread
        publishing_thread_ = std::thread([this]() { PublishingLoop(); });
    }

    ~CtIcpDatasetNode() {
        if (publishing_thread_.joinable()) {
            publishing_thread_.join();
        }
    }

private:
    /* ---- Configuration ---- */
    double frequency_hz_ = 10.0;
    std::shared_ptr<ct_icp::ADatasetSequence> sequence_;

    /* ---- Publishers ---- */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_odom_pub_;

    /* ---- Transform Broadcaster ---- */
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /* ---- Thread ---- */
    std::thread publishing_thread_;

    const std::string world_frame_id_ = "odom";
    const std::string sensor_frame_id_ = "base_link";
    const std::string gt_frame_id_ = "gt";

    void InitializeDataset(const std::string &dataset_param, const std::string &root_path,
                          const std::string &sequence_name) {
        try {
            ct_icp::DatasetOptions options;
            options.dataset = ct_icp::DATASETFromString(dataset_param);
            options.root_path = root_path;

            auto cleaned_sequence_name = sequence_name;
            if (cleaned_sequence_name.size() >= dataset_param.size() + 1) {
                if (cleaned_sequence_name.substr(0, dataset_param.size() + 1) == dataset_param + "_") {
                    cleaned_sequence_name = cleaned_sequence_name.substr(dataset_param.size() + 1,
                                                                          cleaned_sequence_name.size() -
                                                                          (dataset_param.size() + 1));
                }
            }

            auto dataset = ct_icp::Dataset::LoadDataset(options);
            if (!dataset.HasSequence(cleaned_sequence_name)) {
                RCLCPP_ERROR_STREAM(this->get_logger(),
                        "[CT-ICP] The dataset does not contain the sequence '" << cleaned_sequence_name << "'.");
                RCLCPP_ERROR(this->get_logger(), "[CT-ICP] Available sequences are:");
                for (auto &seq_info: dataset.AllSequenceInfo()) {
                    RCLCPP_ERROR_STREAM(this->get_logger(), seq_info.sequence_name);
                }
                rclcpp::shutdown();
                return;
            }

            RCLCPP_INFO_STREAM(this->get_logger(),
                    "[CT-ICP] Loading the dataset " << ct_icp::DATASETEnumToString(options.dataset)
                    << " located at " << options.root_path
                    << ", Sequence: " << cleaned_sequence_name);

            sequence_ = dataset.GetSequence(cleaned_sequence_name);

        } catch (const std::exception &e) {
            RCLCPP_ERROR_STREAM(this->get_logger(),
                    "[CT-ICP] Could not load the Dataset.\nCaught exception: " << e.what());
            rclcpp::shutdown();
        }
    }

    void PublishingLoop() {
        if (!sequence_) {
            RCLCPP_ERROR(this->get_logger(), "Sequence is null, cannot start publishing");
            return;
        }

        double num_ms_per_frame = 1000.0 / frequency_hz_;
        size_t fid = 0;
        auto num_frames = sequence_->NumFrames();

        while (sequence_->HasNext() && rclcpp::ok()) {
            slam::Timer timer;

            try {
                /* ---- CONVERT CT-ICP FRAME TO POINTCLOUD2 ---- */
                {
                    slam::Timer::Ticker ticker(timer, "conversion");
                    auto next_frame = sequence_->NextFrame();

                    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing frame [" << fid++ << "/" << num_frames << "]");

                    // Convert to PointCloud2
                    auto stamp = slam::SecondsToROSTime(next_frame.begin_pose->dest_timestamp);
                    auto pc_msg = slam::SlamPointCloudToROSCloud2(
                            *next_frame.pointcloud,
                            sensor_frame_id_);
                    pc_msg.header.stamp = stamp;

                    /* ---- PUBLISH POINTCLOUD ---- */
                    pointcloud_pub_->publish(pc_msg);

                    /* ---- PUBLISH GROUND TRUTH IF AVAILABLE ---- */
                    if (next_frame.HasGroundTruth()) {
                        auto odom_msg = slam::SE3ToROSOdometry(
                                next_frame.begin_pose->pose,
                                world_frame_id_,
                                gt_frame_id_);

                        gt_odom_pub_->publish(odom_msg);

                        // Publish transform
                        geometry_msgs::msg::TransformStamped tf_gt;
                        tf_gt.header.stamp = stamp;
                        tf_gt.header.frame_id = world_frame_id_;
                        tf_gt.child_frame_id = gt_frame_id_;
                        tf_gt.transform.translation.x = next_frame.begin_pose->pose.tr.x();
                        tf_gt.transform.translation.y = next_frame.begin_pose->pose.tr.y();
                        tf_gt.transform.translation.z = next_frame.begin_pose->pose.tr.z();
                        tf_gt.transform.rotation.x = next_frame.begin_pose->pose.quat.x();
                        tf_gt.transform.rotation.y = next_frame.begin_pose->pose.quat.y();
                        tf_gt.transform.rotation.z = next_frame.begin_pose->pose.quat.z();
                        tf_gt.transform.rotation.w = next_frame.begin_pose->pose.quat.w();

                        tf_broadcaster_->sendTransform(tf_gt);
                    }

                    /* ---- SLEEP TO MAINTAIN FREQUENCY ---- */
                    double lag = std::max(num_ms_per_frame - timer.AverageDuration("conversion",
                                                                                     slam::Timer::MILLISECONDS), 0.0);
                    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(lag));
                }

            } catch (const std::exception &e) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Caught Exception: " << e.what());
                rclcpp::shutdown();
                break;
            }
        }
    }
};

/* ------------------------------------------------------------------------------------------------------------------ */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CtIcpDatasetNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
