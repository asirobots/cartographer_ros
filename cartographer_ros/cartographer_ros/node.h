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

#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/mutex.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer_ros/map_builder_bridge.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/trajectory_options.h"

#include "cartographer_ros_msgs/srv/finish_trajectory.hpp"
#include "cartographer_ros_msgs/srv/start_trajectory.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include "cartographer_ros_msgs/srv/write_state.hpp"
#include "cartographer_ros_msgs/msg/sensor_topics.hpp"
#include "cartographer_ros_msgs/msg/submap_entry.hpp"
#include "cartographer_ros_msgs/msg/submap_list.hpp"

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

  // Runs final optimization. All trajectories have to be finished when calling.
  void RunFinalOptimization();

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
                             nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void HandleImuMessage(int trajectory_id, const string& sensor_id,
                        sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void HandleLaserScanMessage(int trajectory_id, const string& sensor_id,
                              sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void HandleMultiEchoLaserScanMessage(
      int trajectory_id, const string& sensor_id,
      sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr msg);
  void HandlePointCloud2Message(int trajectory_id, const string& sensor_id,
                                sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  // Serializes the complete Node state.
  void SerializeState(const string& filename);

  // Loads a persisted state to use as a map.
  void LoadMap(const std::string& map_filename);

  rclcpp::Node::SharedPtr node_handle();

 protected:
  virtual cartographer_ros_msgs::msg::SensorTopics DefaultSensorTopics();
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
  // Returns the set of topic names we want to subscribe to.
  virtual std::unordered_set<string> ComputeExpectedTopics(
      const TrajectoryOptions& options,
      const cartographer_ros_msgs::msg::SensorTopics& topics);

  int AddTrajectory(const TrajectoryOptions& options,
                    const cartographer_ros_msgs::msg::SensorTopics& topics);
  virtual void LaunchSubscribers(const TrajectoryOptions& options,
                         const cartographer_ros_msgs::msg::SensorTopics& topics,
                         int trajectory_id);
  void PublishSubmapList();
  void AddExtrapolator(int trajectory_id, const TrajectoryOptions& options);
  void AddSensorSamplers(int trajectory_id, const TrajectoryOptions& options);
  void PublishTrajectoryStates();
  void PublishTrajectoryNodeList();
  void PublishConstraintList();
  void SpinOccupancyGridThreadForever();
  bool ValidateTrajectoryOptions(const TrajectoryOptions& options);
  bool ValidateTopicNames(const ::cartographer_ros_msgs::msg::SensorTopics& topics,
                         const TrajectoryOptions& options);

  virtual void PublishOtherOdometry(const std_msgs::msg::Header::_stamp_type &timestamp,
                                    const MapBuilderBridge::TrajectoryState &trajectory_state,
                                    const cartographer::transform::Rigid3d &tracking_to_local,
                                    const cartographer::transform::Rigid3d &tracking_to_map);


  const NodeOptions node_options_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  cartographer::common::Mutex mutex_;
  MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);

  ::rclcpp::Node::SharedPtr node_handle_;
  ::rclcpp::Publisher<::cartographer_ros_msgs::msg::SubmapList>::SharedPtr submap_list_publisher_;
  ::rclcpp::Publisher<::visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_node_list_publisher_;
  ::rclcpp::Publisher<::visualization_msgs::msg::MarkerArray>::SharedPtr constraint_list_publisher_;
  rclcpp::Clock system_clock_;

  // These ros::ServiceServers need to live for the lifetime of the node.
  std::vector<rclcpp::ServiceBase::SharedPtr> service_servers_;
  ::rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_matched_point_cloud_publisher_;
  cartographer::common::Time last_scan_matched_point_cloud_time_ =
      cartographer::common::Time::min();

  struct TrajectorySensorSamplers {
    TrajectorySensorSamplers(double rangefinder_sampling_ratio,
                             double odometry_sampling_ratio,
                             double imu_sampling_ratio)
        : rangefinder_sampler(rangefinder_sampling_ratio),
          odometry_sampler(odometry_sampling_ratio),
          imu_sampler(imu_sampling_ratio) {}

    ::cartographer::common::FixedRatioSampler rangefinder_sampler;
    ::cartographer::common::FixedRatioSampler odometry_sampler;
    ::cartographer::common::FixedRatioSampler imu_sampler;
  };

  // These are keyed with 'trajectory_id'.
  std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_;
  std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_;
  std::unordered_map<int, std::vector<rclcpp::SubscriptionBase::SharedPtr>> subscribers_;
  std::unordered_set<std::string> subscribed_topics_;
  std::unordered_map<int, bool> is_active_trajectory_ GUARDED_BY(mutex_);

  // We have to keep the timer handles of ::ros::WallTimers around, otherwise
  // they do not fire.
  std::vector<::rclcpp::TimerBase::SharedPtr> wall_timers_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_NODE_H_
