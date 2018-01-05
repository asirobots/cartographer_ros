//
// Created by brannon on 8/7/17.
//

#ifndef CARTOGRAPHER_ROS_ASI_NODE_H
#define CARTOGRAPHER_ROS_ASI_NODE_H

#include "node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace cartographer_ros {

  class AsiNode : public Node {
  public:
    AsiNode(const NodeOptions &node_options, tf2_ros::Buffer *tf_buffer);

  protected:
    cartographer_ros_msgs::msg::SensorTopics DefaultSensorTopics() override;

    void PublishOtherOdometry(const std_msgs::msg::Header::_stamp_type &timestamp,
                              const MapBuilderBridge::TrajectoryState &trajectory_state,
                              const cartographer::transform::Rigid3d &tracking_to_local,
                              const cartographer::transform::Rigid3d &tracking_to_map) override;

    void LaunchSubscribers(const TrajectoryOptions &options,
                                   const cartographer_ros_msgs::msg::SensorTopics &topics,
                                   int trajectory_id) override;

    std::unordered_set<string> ComputeExpectedTopics(
        const TrajectoryOptions &options,
        const cartographer_ros_msgs::msg::SensorTopics &topics) override;

    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose3d_publisher_ = nullptr;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> pose3d_cov_publisher_ = nullptr;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers_;

    tf2_ros::Buffer *tf_buffer_;

    cartographer::transform::Rigid3d last_twist_odometry_;
    double last_twist_odometry_time_ = std::numeric_limits<double>::infinity();
    cartographer::mapping::TrajectoryBuilder::PoseEstimate last_pose_estimate_;
  };

}

#endif //CARTOGRAPHER_ROS_ASI_NODE_H
