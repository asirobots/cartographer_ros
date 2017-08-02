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

#include "cartographer_ros/node.h"

#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "glog/logging.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "asiframework_msgs/msg/asi_time.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "visualization_msgs/MarkerArray.h"

namespace cartographer_ros {

namespace {

cartographer_ros_msgs::msg::SensorTopics DefaultSensorTopics() {
  cartographer_ros_msgs::msg::SensorTopics topics;
  topics.laser_scan_topic = kLaserScanTopic;
  topics.multi_echo_laser_scan_topic = kMultiEchoLaserScanTopic;
  topics.point_cloud2_topic = kPointCloud2Topic;
  topics.imu_topic = kImuTopic;
  topics.odometry_topic = kOdometryTopic;
  return topics;
}

// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
template <typename MessageType>
::ros::Subscriber SubscribeWithHandler(
    void (Node::*handler)(int, const string&,
                          const typename MessageType::ConstPtr&),
    const int trajectory_id, const string& topic,
    ::ros::NodeHandle* const node_handle, Node* const node) {
  return node_handle->subscribe<MessageType>(
      topic, kInfiniteSubscriberQueueSize,
      boost::function<void(const typename MessageType::ConstPtr&)>(
          [node, handler, trajectory_id,
           topic](const typename MessageType::ConstPtr& msg) {
            (node->*handler)(trajectory_id, topic, msg);
          }));
}

}  // namespace

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

Node::Node(const NodeOptions& node_options, tf2_ros::Buffer* const tf_buffer)
    : node_options_(node_options),
      map_builder_bridge_(node_options_, tf_buffer) {
  carto::common::MutexLocker lock(&mutex_);
  node_handle_ = rclcpp::Node::make_shared("cartographer_node");
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_handle_);

  auto submap_list_qos = rmw_qos_profile_default;
  submap_list_qos.depth = kLatestOnlyPublisherQueueSize;
  submap_list_publisher_ = node_handle_->create_publisher<::cartographer_ros_msgs::msg::SubmapList>(
      kSubmapListTopic, submap_list_qos);
  trajectory_node_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
  constraint_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kConstraintListTopic, kLatestOnlyPublisherQueueSize);
  service_servers_.push_back(node_handle_->create_service<::cartographer_ros_msgs::srv::SubmapQuery>(
      kSubmapQueryServiceName, std::bind(&Node::HandleSubmapQuery, this, _1, _2, _3)));
  service_servers_.push_back(node_handle_->create_service<::cartographer_ros_msgs::srv::StartTrajectory>(
      kStartTrajectoryServiceName, std::bind(&Node::HandleStartTrajectory, this, _1, _2, _3)));
  service_servers_.push_back(node_handle_->create_service<::cartographer_ros_msgs::srv::FinishTrajectory>(
      kFinishTrajectoryServiceName, std::bind(&Node::HandleFinishTrajectory, this, _1, _2, _3)));
  service_servers_.push_back(node_handle_.advertiseService(
      kWriteStateServiceName, &Node::HandleWriteState, this));

  scan_matched_point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);

    lean_pose_publisher = node_handle_->create_publisher<localization_msgs::msg::Pose2DWithCovarianceRelativeStamped>(
            "cartographer_pose", rmw_qos_profile_default);


  wall_timers_.push_back(node_handle_->create_wall_timer(std::chrono::milliseconds((int)(node_options_.submap_publish_period_sec*1000)), std::bind(&Node::PublishSubmapList, this)));
  wall_timers_.push_back(node_handle_->create_wall_timer(std::chrono::milliseconds((int)(node_options_.pose_publish_period_sec*1000)), std::bind(&Node::PublishTrajectoryStates, this)));

  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.submap_publish_period_sec),
      &Node::PublishSubmapList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.pose_publish_period_sec),
      &Node::PublishTrajectoryStates, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishTrajectoryNodeList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kConstraintPublishPeriodSec),
      &Node::PublishConstraintList, this));
}

Node::~Node() {}

::ros::NodeHandle* Node::node_handle() { return &node_handle_; }

bool Node::HandleSubmapQuery(
    ::cartographer_ros_msgs::msg::SubmapQuery::Request& request,
    ::cartographer_ros_msgs::msg::SubmapQuery::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  return map_builder_bridge_.HandleSubmapQuery(request, response);
}

::rclcpp::node::Node::SharedPtr Node::node_handle() { return node_handle_; }

MapBuilderBridge* Node::map_builder_bridge() { return &map_builder_bridge_; }


void Node::PublishSubmapList() {
  carto::common::MutexLocker lock(&mutex_);
  submap_list_publisher_->publish(map_builder_bridge_.GetSubmapList());
}

void Node::AddExtrapolator(const int trajectory_id,
                           const TrajectoryOptions& options) {
  constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
  CHECK(extrapolators_.count(trajectory_id) == 0);
  const double gravity_time_constant =
      node_options_.map_builder_options.use_trajectory_builder_3d()
          ? options.trajectory_builder_options.trajectory_builder_3d_options()
                .imu_gravity_time_constant()
          : options.trajectory_builder_options.trajectory_builder_2d_options()
                .imu_gravity_time_constant();
  extrapolators_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
          gravity_time_constant));
}

void Node::PublishTrajectoryStates(const ::ros::WallTimerEvent& timer_event) {
  carto::common::MutexLocker lock(&mutex_);
  for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
    const auto& trajectory_state = entry.second;

    auto& extrapolator = extrapolators_.at(entry.first);
    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    if (trajectory_state.pose_estimate.time != extrapolator.GetLastPoseTime()) {
      scan_matched_point_cloud_publisher_.publish(ToPointCloud2Message(
          carto::common::ToUniversal(trajectory_state.pose_estimate.time),
          node_options_.map_frame,
          carto::sensor::TransformPointCloud(
              trajectory_state.pose_estimate.point_cloud,
              trajectory_state.local_to_map.cast<float>())));
      extrapolator.AddPose(trajectory_state.pose_estimate.time,
                           trajectory_state.pose_estimate.pose);
    }

    geometry_msgs::TransformStamped stamped_transform;
    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance. If we already know a newer pose, we use its
    // time instead. Since tf knows how to interpolate, providing newer
    // information is better.
    const ::cartographer::common::Time now =
        std::max(FromRos(ros::Time::now()), extrapolator.GetLastPoseTime());
    stamped_transform.header.stamp = ToRos(now);
    const Rigid3d tracking_to_local = extrapolator.ExtrapolatePose(now);
    const Rigid3d tracking_to_map =
        trajectory_state.local_to_map * tracking_to_local;

    if (trajectory_state.published_to_tracking != nullptr) {
      if (trajectory_state.trajectory_options.provide_odom_frame) {
        std::vector<geometry_msgs::msg::TransformStamped> stamped_transforms;

        stamped_transform.header.frame_id = node_options_.map_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.odom_frame;
        stamped_transform.transform =
            ToGeometryMsgTransform(trajectory_state.local_to_map);
        stamped_transforms.push_back(stamped_transform);

        stamped_transform.header.frame_id =
            trajectory_state.trajectory_options.odom_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_local * (*trajectory_state.published_to_tracking));
        stamped_transforms.push_back(stamped_transform);

        tf_broadcaster_->sendTransform(stamped_transforms);
      } else {
        stamped_transform.header.frame_id = node_options_.map_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_map * (*trajectory_state.published_to_tracking));
        tf_broadcaster_->sendTransform(stamped_transform);
      }
      localization_msgs::msg::Pose2DWithCovarianceRelativeStamped poseMsg;
      poseMsg.header.stamp = stamped_transform.header.stamp;
      poseMsg.header.frame_id = node_options_.map_frame;
      poseMsg.child_frame_id = trajectory_state.trajectory_options.odom_frame;

      auto &orientation = tracking_to_local.rotation();
      auto &position = tracking_to_local.translation();
      poseMsg.pose2d.x = position.x();
      poseMsg.pose2d.y = position.y();
      // float yaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)); for q0 = w, q1 = x
      auto q0 = orientation.w();
      auto q1 = orientation.x();
      auto q2 = orientation.y();
      auto q3 = orientation.z();
      poseMsg.pose2d.theta = std::atan2(2.0 * (q3 * q0 + q1 * q2), -1.0 + 2.0 * (q0 * q0 + q1 * q1));

      poseMsg.position_covariance = {0.05, 0.0, 0.05};
      poseMsg.yaw_covariance = 0.15;

      lean_pose_publisher->publish(poseMsg);

    }
  }
}

void Node::PublishTrajectoryNodeList() {
  carto::common::MutexLocker lock(&mutex_);
  if (trajectory_node_list_publisher_.getNumSubscribers() > 0) {
    trajectory_node_list_publisher_.publish(
        map_builder_bridge_.GetTrajectoryNodeList());
  }
}

void Node::PublishConstraintList() {
  carto::common::MutexLocker lock(&mutex_);
  if (constraint_list_publisher_.getNumSubscribers() > 0) {
    constraint_list_publisher_.publish(map_builder_bridge_.GetConstraintList());
  }
}

std::unordered_set<string> Node::ComputeExpectedTopics(
    const TrajectoryOptions& options,
    const cartographer_ros_msgs::msg::SensorTopics& topics) {
  std::unordered_set<string> expected_topics;
  // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
  for (const string& topic : ComputeRepeatedTopicNames(
           topics.laser_scan_topic, options.num_laser_scans)) {
    expected_topics.insert(topic);
  }
  for (const string& topic :
       ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                 options.num_multi_echo_laser_scans)) {
    expected_topics.insert(topic);
  }
  for (const string& topic : ComputeRepeatedTopicNames(
           topics.point_cloud2_topic, options.num_point_clouds)) {
    expected_topics.insert(topic);
  }
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    expected_topics.insert(topics.imu_topic);
  }
  // Odometry is optional.
  if (options.use_odometry) {
    expected_topics.insert(topics.odometry_topic);
  }
  return expected_topics;
}

int Node::AddTrajectory(const TrajectoryOptions& options,
                        const cartographer_ros_msgs::msg::SensorTopics& topics) {
  const std::unordered_set<string> expected_sensor_ids =
      ComputeExpectedTopics(options, topics);
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  LaunchSubscribers(options, topics, trajectory_id);
  is_active_trajectory_[trajectory_id] = true;
  subscribed_topics_.insert(expected_sensor_ids.begin(),
                            expected_sensor_ids.end());
  return trajectory_id;
}

void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const cartographer_ros_msgs::msg::SensorTopics& topics,
                             const int trajectory_id) {
  for (const string& topic : ComputeRepeatedTopicNames(
           topics.laser_scan_topic, options.num_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        SubscribeWithHandler<sensor_msgs::LaserScan>(
            &Node::HandleLaserScanMessage, trajectory_id, topic, &node_handle_,
            this));
  }
  for (const string& topic :
       ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                 options.num_multi_echo_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        SubscribeWithHandler<sensor_msgs::MultiEchoLaserScan>(
            &Node::HandleMultiEchoLaserScanMessage, trajectory_id, topic,
            &node_handle_, this));
  }
  for (const string& topic : ComputeRepeatedTopicNames(
           topics.point_cloud2_topic, options.num_point_clouds)) {
    subscribers_[trajectory_id].push_back(
        SubscribeWithHandler<sensor_msgs::PointCloud2>(
            &Node::HandlePointCloud2Message, trajectory_id, topic,
            &node_handle_, this));
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    string topic = topics.imu_topic;
    subscribers_[trajectory_id].push_back(
        SubscribeWithHandler<sensor_msgs::Imu>(&Node::HandleImuMessage,
                                               trajectory_id, topic,
                                               &node_handle_, this));
  }

  if (options.use_odometry) {
    string topic = topics.odometry_topic;
    subscribers_[trajectory_id].push_back(
        SubscribeWithHandler<nav_msgs::Odometry>(&Node::HandleOdometryMessage,
                                                 trajectory_id, topic,
                                                 &node_handle_, this));
  }
}

bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_2d_options();
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_3d_options();
  }
  return false;
}

bool Node::ValidateTopicNames(
    const ::cartographer_ros_msgs::msg::SensorTopics& topics,
    const TrajectoryOptions& options) {
  for (const std::string& topic : ComputeExpectedTopics(options, topics)) {
    if (subscribed_topics_.count(topic) > 0) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

bool Node::HandleStartTrajectory(
    ::cartographer_ros_msgs::msg::StartTrajectory::Request& request,
    ::cartographer_ros_msgs::msg::StartTrajectory::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  TrajectoryOptions options;
  if (!FromRosMessage(request.options, &options) ||
      !ValidateTrajectoryOptions(options)) {
    LOG(ERROR) << "Invalid trajectory options.";
    return false;
  }
  if (!ValidateTopicNames(request.topics, options)) {
    LOG(ERROR) << "Invalid topics.";
    return false;
  }
  response.trajectory_id = AddTrajectory(options, request.topics);
  return true;
}

void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  CHECK(ValidateTrajectoryOptions(options));
  AddTrajectory(options, DefaultSensorTopics());
}

std::unordered_set<string> Node::ComputeDefaultTopics(
    const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  return ComputeExpectedTopics(options, DefaultSensorTopics());
}

int Node::AddOfflineTrajectory(
    const std::unordered_set<string>& expected_sensor_ids,
    const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  is_active_trajectory_[trajectory_id] = true;
  return trajectory_id;
}

bool Node::HandleFinishTrajectory(
    ::cartographer_ros_msgs::msg::FinishTrajectory::Request& request,
    ::cartographer_ros_msgs::msg::FinishTrajectory::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  const int trajectory_id = request.trajectory_id;
  if (is_active_trajectory_.count(trajectory_id) == 0) {
    LOG(INFO) << "Trajectory_id " << trajectory_id << " is not created yet.";
    return false;
  }
  if (!is_active_trajectory_[trajectory_id]) {
    LOG(INFO) << "Trajectory_id " << trajectory_id
              << " has already been finished.";
    return false;
  }

  // Shutdown the subscribers of this trajectory.
  for (auto& entry : subscribers_[trajectory_id]) {
    entry.shutdown();
    subscribed_topics_.erase(entry.getTopic());
    LOG(INFO) << "Shutdown the subscriber of [" << entry.getTopic() << "]";
  }
  CHECK_EQ(subscribers_.erase(trajectory_id), 1);
  map_builder_bridge_.FinishTrajectory(trajectory_id);
  is_active_trajectory_[trajectory_id] = false;
  return true;
}

bool Node::HandleWriteState(
    ::cartographer_ros_msgs::msg::WriteState::Request& request,
    ::cartographer_ros_msgs::msg::WriteState::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.SerializeState(request.filename);
  return true;
}

void Node::FinishAllTrajectories() {
  carto::common::MutexLocker lock(&mutex_);
  for (const auto& entry : is_active_trajectory_) {
    const int trajectory_id = entry.first;
    if (entry.second) {
      map_builder_bridge_.FinishTrajectory(trajectory_id);
    }
  }
}

void Node::FinishTrajectory(const int trajectory_id) {
  carto::common::MutexLocker lock(&mutex_);
  CHECK(is_active_trajectory_.at(trajectory_id));
  map_builder_bridge_.FinishTrajectory(trajectory_id);
  is_active_trajectory_[trajectory_id] = false;
}

void Node::HandleOdometryMessage(const int trajectory_id,
                                 const string& sensor_id,
                                 const nav_msgs::Odometry::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleOdometryMessage(sensor_id, msg);
}

void Node::HandleImuMessage(const int trajectory_id, const string& sensor_id,
                            const sensor_msgs::Imu::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
  if (imu_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
  }
  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const string& sensor_id,
                                  const sensor_msgs::LaserScan::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);
}

void Node::HandleMultiEchoLaserScanMessage(
    int trajectory_id, const string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}

void Node::HandlePointCloud2Message(
    const int trajectory_id, const string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandlePointCloud2Message(sensor_id, msg);
}

void Node::SerializeState(const string& filename) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.SerializeState(filename);
}

void Node::LoadMap(const std::string& map_filename) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.LoadMap(map_filename);
}

}  // namespace cartographer_ros
