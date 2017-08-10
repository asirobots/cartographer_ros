//
// Created by brannon on 8/7/17.
//

#ifndef CARTOGRAPHER_ROS_ASI_NODE_H
#define CARTOGRAPHER_ROS_ASI_NODE_H

#include "node.h"
#include "localization_msgs/msg/pose2_d_with_covariance_relative_stamped.hpp"
#include "localization_msgs/msg/body_velocity_with_covariance_lean_stamped.hpp"
#include "localization_msgs/msg/body_accel_with_covariance_lean_stamped.hpp"


namespace cartographer_ros {

  class AsiNode : public Node {
  public:
    AsiNode(const NodeOptions &node_options, tf2_ros::Buffer *tf_buffer);

  protected:
    cartographer_ros_msgs::msg::SensorTopics DefaultSensorTopics() override;

    void PublishOtherOdometry(const std_msgs::msg::Header::_stamp_type &timestamp,
                                      const MapBuilderBridge::TrajectoryState &trajectory_state,
                                      const cartographer::transform::Rigid3d &tracking_to_local) override;

    void LaunchSubscribers(const TrajectoryOptions &options,
                                   const cartographer_ros_msgs::msg::SensorTopics &topics,
                                   int trajectory_id) override;

    std::unordered_set<string> ComputeExpectedTopics(
        const TrajectoryOptions &options,
        const cartographer_ros_msgs::msg::SensorTopics &topics) override;

    std::shared_ptr<rclcpp::Publisher<localization_msgs::msg::Pose2DWithCovarianceRelativeStamped>> lean_pose_publisher_ = nullptr;
    std::shared_ptr<rclcpp::Publisher<localization_msgs::msg::BodyVelocityWithCovarianceLeanStamped>> velocity_publisher_ = nullptr;
    std::shared_ptr<rclcpp::Publisher<localization_msgs::msg::BodyAccelWithCovarianceLeanStamped>> acceleration_publisher_ = nullptr;
    rclcpp::SubscriptionBase::SharedPtr asi_clock_subscriber_ = nullptr;
    rclcpp::SubscriptionBase::SharedPtr lean_odometry_subscriber_ = nullptr;
    rclcpp::SubscriptionBase::SharedPtr lean_twist_subscriber_ = nullptr;
    rclcpp::SubscriptionBase::SharedPtr lean_imu_subscriber_ = nullptr;

    cartographer::transform::Rigid3d last_twist_odometry_;
    double last_twist_odometry_time_ = std::numeric_limits<double>::infinity();
    cartographer::mapping::TrajectoryBuilder::PoseEstimate last_pose_estimate_;
  };

}

#endif //CARTOGRAPHER_ROS_ASI_NODE_H
