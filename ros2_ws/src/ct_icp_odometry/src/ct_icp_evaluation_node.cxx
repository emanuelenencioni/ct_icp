#include <mutex>
#include <thread>
#include <chrono>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <SlamCore/timer.h>
#include <SlamCore/eval.h>

#include <ct_icp/dataset.h>

#include <ROS2Core/nav_msgs_conversion.h>

/* ------------------------------------------------------------------------------------------------------------------ */
class CtIcpEvaluationNode : public rclcpp::Node {
public:
    CtIcpEvaluationNode() : Node("ct_icp_evaluation") {
        RCLCPP_INFO(this->get_logger(), "Initializing CT-ICP Evaluation Node");

        // Load parameters
        auto odom_topic = this->declare_parameter<std::string>("odom_topic", "/odom");
        auto gt_type_str = this->declare_parameter<std::string>("gt_type", "none");

        options_.odometry_topic = odom_topic;
        options_.gt_type = GTTypeFromString(gt_type_str);

        if (options_.gt_type == GT_TYPE::NONE) {
            RCLCPP_WARN(this->get_logger(),
                    "No ground truth setup in the parameters. The node will only record odometry messages!");
        }

        RCLCPP_INFO_STREAM(this->get_logger(),
                "Listening to odometry topic `" << options_.odometry_topic
                << "`, gt_type=" << static_cast<int>(options_.gt_type));

        // Create subscriber for odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                options_.odometry_topic,
                10,
                [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                    HandleOdometryMessage(msg, true);
                });

        // Load ground truth if needed
        if (options_.gt_type != GT_TYPE::NONE) {
            LoadGroundTruth();
        }

        // Start evaluation loop thread
        signal_end_ = false;
        evaluation_thread_ = std::thread([this]() { RunEvaluationLoop(); });

        RCLCPP_INFO(this->get_logger(), "CT-ICP Evaluation Node initialized successfully");
    }

    ~CtIcpEvaluationNode() {
        signal_end_ = true;
        if (evaluation_thread_.joinable()) {
            evaluation_thread_.join();
        }
    }

private:
    /* ---- Ground Truth Type ---- */
    enum GT_TYPE {
        NONE = 0,
        FROM_TOPIC = 1,
        FROM_PLY_FILE = 2,
        FROM_KITTI_FILE = 3,
        FROM_HILTI_2021_FILE = 4,
        FROM_HILTI_2022_FILE = 5,
        FROM_NCLT_FILE = 6
    };

    struct Options {
        GT_TYPE gt_type = GT_TYPE::NONE;
        std::string odometry_topic = "/odom";
        std::string gt_topic = "/gt_odom";
        std::string gt_file_path;
    };

    /* ---- Configuration ---- */
    Options options_;

    /* ---- State ---- */
    std::mutex trajectory_guard_;
    std::vector<slam::Pose> gt_trajectory_;
    std::vector<slam::Pose> odometry_poses_;
    std::atomic<bool> trajectory_changed_ = false;
    std::atomic<bool> signal_end_ = false;

    /* ---- Subscribers ---- */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;

    /* ---- Thread ---- */
    std::thread evaluation_thread_;

    static GT_TYPE GTTypeFromString(const std::string &gt_type) {
        std::string lc_gt_type = gt_type;
        std::transform(lc_gt_type.begin(), lc_gt_type.end(),
                      lc_gt_type.begin(), [](unsigned char c) { return std::tolower(c); });

        if (lc_gt_type == "none")
            return GT_TYPE::NONE;
        else if (lc_gt_type == "from_topic")
            return GT_TYPE::FROM_TOPIC;
        else if (lc_gt_type == "from_kitti_file")
            return GT_TYPE::FROM_KITTI_FILE;
        else if (lc_gt_type == "from_hilti_2021_file")
            return GT_TYPE::FROM_HILTI_2021_FILE;
        else if (lc_gt_type == "from_hilti_2022_file")
            return GT_TYPE::FROM_HILTI_2022_FILE;
        else if (lc_gt_type == "from_nclt_file")
            return GT_TYPE::FROM_NCLT_FILE;
        else if (lc_gt_type == "from_ply_file")
            return GT_TYPE::FROM_PLY_FILE;
        else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("ct_icp_evaluation"),
                    "The type of ground truth is not recognised: `" << gt_type << "`, returning NONE");
            return GT_TYPE::NONE;
        }
    }

    void HandleOdometryMessage(const nav_msgs::msg::Odometry::SharedPtr message, bool is_odometry) {
        if (message) {
            std::lock_guard<std::mutex> lock{trajectory_guard_};
            if (is_odometry) {
                odometry_poses_.push_back(slam::ROSOdometryToPose(*message));
            } else {
                gt_trajectory_.push_back(slam::ROSOdometryToPose(*message));
            }
            trajectory_changed_ = true;
        }
    }

    void LoadGroundTruth() {
        if (options_.gt_type == GT_TYPE::FROM_TOPIC) {
            auto gt_topic = this->declare_parameter<std::string>("gt_topic", "/gt_odom");
            options_.gt_topic = gt_topic;

            gt_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    options_.gt_topic,
                    10,
                    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                        HandleOdometryMessage(msg, false);
                    });

            RCLCPP_INFO_STREAM(this->get_logger(),
                    "Setting the ground truth from topic " << options_.gt_topic);
        } else {
            auto file_path = this->declare_parameter<std::string>("gt_file_path", "");
            options_.gt_file_path = file_path;

            try {
                if (options_.gt_type == GT_TYPE::FROM_PLY_FILE) {
                    gt_trajectory_ = slam::ReadPosesFromPLY(file_path);
                } else if (options_.gt_type == GT_TYPE::FROM_KITTI_FILE) {
                    gt_trajectory_ = slam::LoadPosesKITTIFormat(file_path);
                } else if (options_.gt_type == GT_TYPE::FROM_HILTI_2021_FILE) {
                    gt_trajectory_ = ct_icp::ReadHILTIPosesInLidarFrame(file_path, ct_icp::HILTI_2021);
                } else if (options_.gt_type == GT_TYPE::FROM_HILTI_2022_FILE) {
                    gt_trajectory_ = ct_icp::ReadHILTIPosesInLidarFrame(file_path, ct_icp::HILTI_2022);
                } else if (options_.gt_type == GT_TYPE::FROM_NCLT_FILE) {
                    gt_trajectory_ = ct_icp::ReadNCLTPoses(file_path);
                }

                RCLCPP_INFO_STREAM(this->get_logger(),
                        "Setting the ground truth from file " << file_path);
            } catch (const std::exception &e) {
                RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Could not load the ground truth poses from the file " << file_path
                        << ". Exception: " << e.what());
                rclcpp::shutdown();
            }
        }
    }

    void RunEvaluationLoop() {
        std::vector<slam::Pose> gt_poses, odom_poses, sampled_gt_poses, sampled_odom_poses;

        while (!signal_end_ && rclcpp::ok()) {
            // Compute the metrics for the trajectory
            if (trajectory_changed_ && !gt_trajectory_.empty() && !odometry_poses_.empty()) {
                {
                    // Make a copy of the two trajectories
                    std::lock_guard<std::mutex> lock{trajectory_guard_};
                    gt_poses = gt_trajectory_;
                    odom_poses = odometry_poses_;
                    trajectory_changed_ = false;
                }

                // Compute trajectory metrics
                auto max_t = odom_poses.back().dest_timestamp;
                double min_t = odom_poses.front().dest_timestamp;

                sampled_gt_poses.clear();
                for (auto &gt_pose: gt_poses) {
                    if (gt_pose.dest_timestamp > max_t)
                        break;
                    if (gt_pose.dest_timestamp < min_t)
                        continue;
                    sampled_gt_poses.push_back(gt_pose);
                }

                auto odom_trajectory = slam::LinearContinuousTrajectory::Create(std::move(odom_poses), false);
                sampled_odom_poses.clear();
                sampled_odom_poses.reserve(sampled_gt_poses.size());

                for (auto &gt_pose: sampled_gt_poses) {
                    sampled_odom_poses.push_back(odom_trajectory.InterpolatePose(gt_pose.dest_timestamp));
                }

                // Compute metrics
                if (sampled_gt_poses.size() > 5) {
                    auto kitti_metrics = slam::kitti::EvaluatePoses(sampled_gt_poses, sampled_odom_poses);
                    auto slam_metrics = slam::ComputeTrajectoryMetrics(sampled_gt_poses, sampled_odom_poses);

                    double global_ate = (sampled_gt_poses.back().pose.tr - sampled_odom_poses.back().pose.tr).norm();

                    RCLCPP_INFO_STREAM(this->get_logger(),
                            "Metrics: GLOBAL_ATE=" << global_ate << "(m) /  MEAN ATE="
                            << slam_metrics.mean_ate << "(m) / MAX_ATE="
                            << slam_metrics.max_ate << "(m) / KITTI_MEAN_RPE="
                            << kitti_metrics.mean_rpe << "%");
                }
            }

            std::this_thread::sleep_for(std::chrono::duration<double>(3.0));
        }
    }
};

/* ------------------------------------------------------------------------------------------------------------------ */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CtIcpEvaluationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
