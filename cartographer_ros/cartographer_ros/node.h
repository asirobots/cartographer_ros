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

#include <map>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cartographer/common/mutex.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer_ros/map_builder_bridge.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/trajectory_options.h"

#include "cartographer_ros_msgs/srv/finish_trajectory.hpp"
#include "cartographer_ros_msgs/srv/sensor_topics.hpp"
#include "cartographer_ros_msgs/srv/start_trajectory.hpp"
#include "cartographer_ros_msgs/msg/submap_entry.hpp"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include "cartographer_ros_msgs/msg/write_trajectory_options.hpp"
#include "cartographer_ros_msgs/msg/write_state.hpp"

#include "tf2_ros/transform_broadcaster.h"

namespace cartographer_ros {


// Wires up ROS topics to SLAM.
class Node {
 public:
  Node(const NodeOptions& node_options, tf2_ros::Buffer* tf_buffer);
  ~Node();

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  // Finishes all yet active trajectories.
  void FinishAllTrajectories();
  // Finishes a single given trajectory.
  void FinishTrajectory(int trajectory_id);

  // Starts the first trajectory with the default topics.
  void StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options);

  // Compute the default topics for the given 'options'.
  std::unordered_set<string> ComputeDefaultTopics(
      const TrajectoryOptions& options);

  // Adds a trajectory for offline processing, i.e. not listening to topics.
  int AddOfflineTrajectory(
      const std::unordered_set<string>& expected_sensor_ids,
      const TrajectoryOptions& options);

  // The following functions handle adding sensor data to a trajectory.
  void HandleOdometryMessage(int trajectory_id, const string& sensor_id,
                             const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
  void HandleImuMessage(int trajectory_id, const string& sensor_id,
                        const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
  void HandleLaserScanMessage(int trajectory_id, const string& sensor_id,
                              const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
  void HandleMultiEchoLaserScanMessage(
      int trajectory_id, const string& sensor_id,
      const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr& msg);
  void HandlePointCloud2Message(int trajectory_id, const string& sensor_id,
                                const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  // Serializes the complete Node state.
  void SerializeState(const string& filename);

  // Loads a persisted state to use as a map.
  void LoadMap(const std::string& map_filename);

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
  bool HandleWriteState(const std::shared_ptr<::rmw_request_id_t> request_id,
      cartographer_ros_msgs::srv::WriteState::Request::SharedPtr request,
      cartographer_ros_msgs::srv::WriteState::Response::SharedPtr response);
  int AddTrajectory(const TrajectoryOptions& options,
                    const cartographer_ros_msgs::msg::SensorTopics& topics);
  void LaunchSubscribers(const TrajectoryOptions& options,
                         const cartographer_ros_msgs::msg::SensorTopics& topics,
                         int trajectory_id);
  void PublishSubmapList();
  void AddExtrapolator(int trajectory_id, const TrajectoryOptions& options);
  void PublishTrajectoryStates();
  void PublishTrajectoryNodeList();
  void PublishConstraintList();
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
  ::ros::Publisher trajectory_node_list_publisher_;
  ::ros::Publisher constraint_list_publisher_;
// These ros::ServiceServers need to live for the lifetime of the node.
  std::vector<rclcpp::service::ServiceBase::SharedPtr> service_servers_;
  ::rclcpp::publisher::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_matched_point_cloud_publisher_;
  cartographer::common::Time last_scan_matched_point_cloud_time_ =
      cartographer::common::Time::min();

  // These are keyed with 'trajectory_id'.
  std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_;
  std::unordered_map<int, std::vector<rclcpp::SubscriptionBase::SharedPtr>> subscribers_;
  std::unordered_set<std::string> subscribed_topics_;
  std::unordered_map<int, bool> is_active_trajectory_ GUARDED_BY(mutex_);

  // We have to keep the timer handles of ::ros::WallTimers around, otherwise
  // they do not fire.
  std::vector<::ros::WallTimer> wall_timers_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_NODE_H_
