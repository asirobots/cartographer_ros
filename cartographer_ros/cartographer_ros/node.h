/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_NODE_H_
#define CARTOGRAPHER_ROS_NODE_H_

#include <memory>
#include <vector>

#include "cartographer/common/mutex.h"
#include "cartographer_ros/map_builder_bridge.h"
#include "cartographer_ros/node_options.h"

#include "cartographer_ros_msgs/srv/finish_trajectory.hpp"
#include "cartographer_ros_msgs/srv/start_trajectory.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include "cartographer_ros_msgs/srv/write_assets.hpp"

#include "cartographer_ros_msgs/msg/submap_entry.hpp"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/msg/trajectory_submap_list.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include <rclcpp/rclcpp.hpp>
#include "localization_msgs/msg/pose_with_covariance_lean_relative_stamped.hpp"
#include "localization_msgs/msg/pose2_d_with_covariance_relative_stamped.hpp"


namespace cartographer_ros {

// Default topic names; expected to be remapped as needed.
constexpr char kLaserScanTopic[] = "scan";
constexpr char kMultiEchoLaserScanTopic[] = "echoes";
constexpr char kPointCloud2Topic[] = "points2";
constexpr char kImuTopic[] = "imu";
constexpr char kOdometryTopic[] = "odom";
constexpr char kFinishTrajectoryServiceName[] = "finish_trajectory";
constexpr char kOccupancyGridTopic[] = "map";
constexpr char kScanMatchedPointCloudTopic[] = "scan_matched_points2";
constexpr char kSubmapListTopic[] = "submap_list";
constexpr char kSubmapQueryServiceName[] = "submap_query";
constexpr char kStartTrajectoryServiceName[] = "start_trajectory";
constexpr char kWriteAssetsServiceName[] = "write_assets";

constexpr char lean_pose_topic[] = "Vehicle_Pose";
constexpr char asi_clock_topic[] = "asi_clock";

    // Wires up ROS topics to SLAM.
class Node {
 public:
  Node(const NodeOptions& node_options, tf2_ros::Buffer* tf_buffer);
  ~Node();

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  // Finishes all yet active trajectories.
  void FinishAllTrajectories();

  void StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options);
  rclcpp::node::Node::SharedPtr node_handle();
  MapBuilderBridge* map_builder_bridge();

 private:
  bool HandleSubmapQuery(const std::shared_ptr<::rmw_request_id_t> request_id,
                         cartographer_ros_msgs::srv::SubmapQuery::Request::SharedPtr request,
      cartographer_ros_msgs::srv::SubmapQuery::Response::SharedPtr response);
  bool HandleStartTrajectory(const std::shared_ptr<::rmw_request_id_t> request_id,
      cartographer_ros_msgs::srv::StartTrajectory::Request::SharedPtr request,
      cartographer_ros_msgs::srv::StartTrajectory::Response::SharedPtr response);
  bool HandleFinishTrajectory(const std::shared_ptr<::rmw_request_id_t> request_id,
      cartographer_ros_msgs::srv::FinishTrajectory::Request::SharedPtr request,
      cartographer_ros_msgs::srv::FinishTrajectory::Response::SharedPtr response);
  bool HandleWriteAssets(const std::shared_ptr<::rmw_request_id_t> request_id,
      cartographer_ros_msgs::srv::WriteAssets::Request::SharedPtr request,
      cartographer_ros_msgs::srv::WriteAssets::Response::SharedPtr response);
  int AddTrajectory(const TrajectoryOptions& options,
                    const cartographer_ros_msgs::msg::SensorTopics& topics);
  void LaunchSubscribers(const TrajectoryOptions& options,
                         const cartographer_ros_msgs::msg::SensorTopics& topics,
                         int trajectory_id);
  void PublishSubmapList();
  void PublishTrajectoryStates();
  void SpinOccupancyGridThreadForever();
  bool ValidateTrajectoryOptions(const TrajectoryOptions& options);
  bool ValidateTopicName(const ::cartographer_ros_msgs::msg::SensorTopics& topics,
                         const TrajectoryOptions& options);

  const NodeOptions node_options_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  cartographer::common::Mutex mutex_;
  MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);

  ::rclcpp::node::Node::SharedPtr node_handle_;
  ::rclcpp::publisher::Publisher<::cartographer_ros_msgs::msg::SubmapList>::SharedPtr submap_list_publisher_;
    // These ros::ServiceServers need to live for the lifetime of the node.
  std::vector<rclcpp::service::ServiceBase::SharedPtr> service_servers_;
  ::rclcpp::publisher::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_matched_point_cloud_publisher_;
  cartographer::common::Time last_scan_matched_point_cloud_time_ =
      cartographer::common::Time::min();

  // These are keyed with 'trajectory_id'.
  std::unordered_map<int, rclcpp::SubscriptionBase::SharedPtr> laser_scan_subscribers_;
  std::unordered_map<int, rclcpp::SubscriptionBase::SharedPtr> multi_echo_laser_scan_subscribers_;
  std::unordered_map<int, rclcpp::SubscriptionBase::SharedPtr> odom_subscribers_;
  std::unordered_map<int, rclcpp::SubscriptionBase::SharedPtr> imu_subscribers_;
  std::unordered_map<int, std::vector<rclcpp::SubscriptionBase::SharedPtr>> point_cloud_subscribers_;
  std::unordered_map<int, bool> is_active_trajectory_ GUARDED_BY(mutex_);
  ::rclcpp::publisher::Publisher<::nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
  std::thread occupancy_grid_thread_;
  bool terminating_ = false GUARDED_BY(mutex_);

  // We have to keep the timer handles of ::ros::WallTimers around, otherwise
  // they do not fire.
  std::vector<::rclcpp::TimerBase::SharedPtr> wall_timers_;
  std::shared_ptr<rclcpp::Publisher<localization_msgs::msg::Pose2DWithCovarianceRelativeStamped>> lean_pose_publisher;
  rclcpp::SubscriptionBase::SharedPtr asi_clock_subscriber_;
  std::unordered_map<int, rclcpp::SubscriptionBase::SharedPtr> asi_pose_subscribers_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_NODE_H_
