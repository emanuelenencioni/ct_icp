#include <memory>
#include <mutex>
#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <SlamCore/timer.h>
#include <SlamCore/config_utils.h>

#include <ct_icp/odometry.h>
#include <ct_icp/config.h>

#include <ROS2Core/pc2_conversion.h>
#include <ROS2Core/nav_msgs_conversion.h>

#include "utils.h"

/* ------------------------------------------------------------------------------------------------------------------ */
// Config read from disk
struct Config {
    ct_icp::OdometryOptions odometry_options = ct_icp::OdometryOptions::DefaultDrivingProfile();
    bool output_state_on_failure = true;
    std::string failure_output_dir = "/tmp";
    ct_icp::TIME_UNIT unit = ct_icp::SECONDS;
    bool check_timestamp_consistency = true;
    double expected_frame_time_sec = 0.1;
};

/* ------------------------------------------------------------------------------------------------------------------ */
// ROS2 Odometry Node
class CtIcpOdometryNode : public rclcpp::Node {
public:
    CtIcpOdometryNode() : Node("ct_icp_odometry") {
        RCLCPP_INFO(this->get_logger(), "Initializing CT-ICP Odometry Node");

        // Load parameters
        auto config_path = this->declare_parameter<std::string>("config", "");
        debug_print_ = this->declare_parameter<bool>("debug_print", false);

        // Load configuration
        LoadConfig(config_path);

        // Initialize odometry algorithm
        odometry_ptr_ = std::make_unique<ct_icp::Odometry>(config_.odometry_options);

        // Create publishers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/ct_icp/pose/odom", 5);
        key_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ct_icp/key_points", 1);
        world_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ct_icp/world_points", 1);

        // Initialize transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());

        // Create point cloud subscriber
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/ct_icp/pointcloud",
                200,
                std::bind(&CtIcpOdometryNode::RegisterNewFrameCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "CT-ICP Odometry Node initialized successfully");
    }

private:
    /* ---- Configuration and State ---- */
    Config config_;
    std::unique_ptr<ct_icp::Odometry> odometry_ptr_;
    bool debug_print_ = false;

    /* ---- Frame tracking ---- */
    slam::frame_id_t frame_id_ = 0;
    std::atomic<double> previous_timestamp_;
    std::atomic<bool> is_initialized_ = false;
    std::mutex registration_mutex_;
    slam::Timer avg_timer_;

    /* ---- Publishers ---- */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr key_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr world_points_pub_;

    /* ---- Subscriber ---- */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;

    /* ---- Transform Broadcaster ---- */
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /* ---- Methods ---- */
    void LoadConfig(const std::string &config_path) {
        if (config_path.empty()) {
            RCLCPP_WARN(this->get_logger(), "Config path is empty, using default driving profile");
            config_.odometry_options = ct_icp::OdometryOptions::DefaultDrivingProfile();
        } else {
            try {
                RCLCPP_INFO_STREAM(this->get_logger(), "Loading config from: " << config_path);
                auto node = slam::config::RootNode(config_path);

                if (node["odometry_options"]) {
                    auto odometry_node = node["odometry_options"];
                    config_.odometry_options = ct_icp::yaml_to_odometry_options(odometry_node);
                }

                FIND_OPTION(node, config_, failure_output_dir, std::string)
                FIND_OPTION(node, config_, output_state_on_failure, bool)
                FIND_OPTION(node, config_, check_timestamp_consistency, bool)
                config_.unit = ct_icp::TimeUnitFromNode(node, "unit");

                RCLCPP_INFO(this->get_logger(), "Config loaded successfully");
            } catch (const std::exception &e) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Error loading config: " << e.what());
                rclcpp::shutdown();
                throw;
            }
        }
    }

    void RegisterNewFrameCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
        if (debug_print_) {
            RCLCPP_INFO(this->get_logger(), "Received Point Cloud Message!");
        }

        std::lock_guard<std::mutex> guard{registration_mutex_};

        // Convert ROS2 PointCloud2 to SlamCore PointCloud
        auto stamp = pc_msg->header.stamp;
        auto stamp_sec = slam::ROSTimeToSeconds(stamp);
        auto pointcloud = slam::ROSCloud2ToSlamPointCloudShallow(*pc_msg);
        pointcloud->RegisterFieldsFromSchema();

        /* ---- CHECK THAT THE TIMESTAMPS FIELD EXIST ---- */
        if (!pointcloud->HasTimestamps()) {
            if (config_.odometry_options.ct_icp_options.parametrization == ct_icp::CONTINUOUS_TIME) {
                RCLCPP_ERROR(this->get_logger(),
                        "Point cloud does not have timestamps, incompatible with CONTINUOUS_TIME parametrization");
                rclcpp::shutdown();
                return;
            }

            // Add timestamps which will all be set to the message timestamp
            auto copy = pointcloud->DeepCopyPtr();
            {
                auto field = copy->AddElementField<double, slam::FLOAT64>("new_timestamps");
                copy->SetTimestampsField(std::move(field));
            }
            auto timestamps = copy->Timestamps<double>();
            for (auto &t: timestamps)
                t = stamp_sec;
            pointcloud = copy;
        } else {
            if (debug_print_) {
                RCLCPP_INFO_STREAM(this->get_logger(),
                        "Processing Frame at timestamp " << stamp.sec << "(sec) " << stamp.nanosec
                        << " (nsec). Containing " << pointcloud->size() << " points.");
            }

            auto timestamps = pointcloud->TimestampsProxy<double>();
            auto minmax = std::minmax_element(timestamps.begin(), timestamps.end());
            double min_t = *minmax.first;
            double max_t = *minmax.second;

            double dt = max_t - min_t;
            double expected_dt;
            switch (config_.unit) {
                case ct_icp::SECONDS:
                    expected_dt = config_.expected_frame_time_sec;
                    break;
                case ct_icp::MILLI_SECONDS:
                    expected_dt = config_.expected_frame_time_sec * 1.e3;
                    break;
                case ct_icp::NANO_SECONDS:
                    expected_dt = config_.expected_frame_time_sec * 1.e9;
                    break;
            }

            if (!is_initialized_) {
                is_initialized_ = true;
                if (debug_print_) {
                    RCLCPP_INFO_STREAM(this->get_logger(), "Min_t=" << min_t << ", Max_t=" << max_t);
                }
            } else {
                bool invalid_timestamps = false;
                double r_dt = dt / expected_dt;

                if (debug_print_) {
                    RCLCPP_INFO_STREAM(this->get_logger(),
                            "Min_t=" << min_t << ", Max_t=" << max_t
                            << ", dt=" << dt << " r_dt=" << r_dt);
                }

                if (r_dt > 1.05 || r_dt < 0.95) {
                    invalid_timestamps = true;
                    if (debug_print_) {
                        RCLCPP_INFO(this->get_logger(),
                                "Found Inconsistent Timestamp for the frame");
                    }

                    std::vector<size_t> ids;
                    ids.reserve(pointcloud->size());
                    auto timestamps = pointcloud->TimestampsProxy<double>();

                    double prev_t = previous_timestamp_;
                    double next_t = prev_t + expected_dt;

                    for (auto idx(0); idx < pointcloud->size(); idx++) {
                        double timestamp = timestamps[idx];
                        if (prev_t <= timestamp && timestamp <= next_t)
                            ids.push_back(idx);
                    }
                    {
                        if (debug_print_) {
                            RCLCPP_WARN(this->get_logger(), "Skipping the frame");
                        }
                        return;
                    }
                } else if (std::abs(previous_timestamp_ - min_t) > expected_dt) {
                    if (debug_print_) {
                        RCLCPP_WARN(this->get_logger(),
                                "Found Inconsistent Timestamp for the frame");
                        RCLCPP_WARN(this->get_logger(), "Will continue the acquisition");
                    }
                }
            }

            previous_timestamp_ = max_t;
        }

        /* ---- REGISTER NEW FRAME ---- */
        slam::Timer timer;
        ct_icp::Odometry::RegistrationSummary summary;

        {
            slam::Timer::Ticker avg_ticker(avg_timer_, "registration");
            {
                slam::Timer::Ticker ticker(timer, "registration");
                summary = odometry_ptr_->RegisterFrame(*pointcloud, frame_id_++);
            }
        }

        if (debug_print_) {
            RCLCPP_INFO_STREAM(this->get_logger(),
                    "Registration took: " << timer.AverageDuration("registration",
                                                                   slam::Timer::MILLISECONDS) << "(ms)");
            RCLCPP_INFO_STREAM(this->get_logger(),
                    "Average Registration time: " << avg_timer_.AverageDuration("registration",
                                                                                slam::Timer::MILLISECONDS) << "(ms)");
        }

        if (summary.success) {
            if (debug_print_)
                RCLCPP_INFO(this->get_logger(), "Registration is a success.");
        } else {
            if (debug_print_)
                RCLCPP_INFO(this->get_logger(), "Registration is a failure");

            if (config_.output_state_on_failure) {
                if (debug_print_)
                    RCLCPP_INFO(this->get_logger(), "Persisting last state");

                namespace fs = std::filesystem;
                fs::path output_dir_path(config_.failure_output_dir);
                if (!exists(output_dir_path))
                    create_directories(output_dir_path);

                {
                    auto initial_frame_path = output_dir_path / "initial_frame.ply";
                    if (debug_print_)
                        RCLCPP_INFO_STREAM(this->get_logger(), "Saving Initial Frame to " << initial_frame_path);
                    std::vector<slam::Pose> initial_frames{summary.initial_frame.begin_pose,
                                                           summary.initial_frame.end_pose};
                    slam::SavePosesAsPLY(initial_frame_path, initial_frames);
                }

                {
                    auto map_path = output_dir_path / "map.ply";
                    if (debug_print_)
                        RCLCPP_INFO_STREAM(this->get_logger(), "Saving Map to " << map_path);
                    auto pc = odometry_ptr_->GetMapPointCloud();
                    pc->RegisterFieldsFromSchema();
                    auto mapper = slam::PLYSchemaMapper::BuildDefaultFromBufferCollection(pc->GetCollection());
                    slam::WritePLY(map_path, *pc, mapper);
                }

                {
                    auto frame_path = output_dir_path / "frame.ply";
                    if (debug_print_)
                        RCLCPP_INFO_STREAM(this->get_logger(), "Saving frame to " << frame_path);
                    auto mapper = slam::PLYSchemaMapper::BuildDefaultFromBufferCollection(
                            pointcloud->GetCollection());
                    slam::WritePLY(frame_path, *pointcloud, mapper);
                }
            }

            rclcpp::shutdown();
        }

        /* ---- PUBLISH RESULTS ---- */
        odom_pub_->publish(ct_icp::SlamPoseToROSOdometry(summary.frame.end_pose.pose, stamp));

        if (!summary.corrected_points.empty()) {
            auto msg = slam::WPointsToROSCloud2(summary.corrected_points,
                                                ct_icp::main_frame_id);
            msg.header.stamp = stamp;
            world_points_pub_->publish(msg);
        }

        if (!summary.keypoints.empty()) {
            auto msg = slam::WPointsToROSCloud2(summary.keypoints,
                                                ct_icp::main_frame_id);
            msg.header.stamp = stamp;
            key_points_pub_->publish(msg);
        }

        tf_broadcaster_->sendTransform(
                ct_icp::TransformFromPose(summary.frame.begin_pose.pose, stamp));
    }
};

/* ------------------------------------------------------------------------------------------------------------------ */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CtIcpOdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
