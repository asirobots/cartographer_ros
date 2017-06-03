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
#include <memory>

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

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

namespace {

constexpr int kInfiniteSubscriberQueueSize = 0;
constexpr int kLatestOnlyPublisherQueueSize = 1;

// Try to convert 'msg' into 'options'. Returns false on failure.
bool FromRosMessage(const cartographer_ros_msgs::TrajectoryOptions& msg,
                    TrajectoryOptions* options) {
  options->tracking_frame = msg.tracking_frame;
  options->published_frame = msg.published_frame;
  options->odom_frame = msg.odom_frame;
  options->provide_odom_frame = msg.provide_odom_frame;
  options->use_odometry = msg.use_odometry;
  options->use_laser_scan = msg.use_laser_scan;
  options->use_multi_echo_laser_scan = msg.use_multi_echo_laser_scan;
  options->num_point_clouds = msg.num_point_clouds;
  if (!options->trajectory_builder_options.ParseFromString(
          msg.trajectory_builder_options_proto)) {
    LOG(ERROR) << "Failed to parse protobuf";
    return false;
  }
  return true;
}
  node_handle_ = rclcpp::Node::make_shared("cartographer_node");
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_handle_);

void ShutdownSubscriber(std::unordered_map<int, ::ros::Subscriber>& subscribers,
                        int trajectory_id) {
  if (subscribers.count(trajectory_id) == 0) {
    return;
  }
  subscribers[trajectory_id].shutdown();
  LOG(INFO) << "Shutdown the subscriber of ["
            << subscribers[trajectory_id].getTopic() << "]";
  CHECK_EQ(subscribers.erase(trajectory_id), 1);
}

bool IsTopicNameUnique(
    const string& topic,
    const std::unordered_map<int, ::ros::Subscriber>& subscribers) {
  for (auto& entry : subscribers) {
    if (entry.second.getTopic() == topic) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

}  // namespace

Node::Node(const NodeOptions& node_options, tf2_ros::Buffer* const tf_buffer)
    : node_options_(node_options),
      map_builder_bridge_(node_options_, tf_buffer) {
  carto::common::MutexLocker lock(&mutex_);
  submap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  service_servers_.push_back(node_handle_.advertiseService(
      kSubmapQueryServiceName, &Node::HandleSubmapQuery, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kStartTrajectoryServiceName, &Node::HandleStartTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kFinishTrajectoryServiceName, &Node::HandleFinishTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kWriteAssetsServiceName, &Node::HandleWriteAssets, this));
  submap_list_publisher_ = node_handle_->create_publisher<::cartographer_ros_msgs::msg::SubmapList>(kSubmapListTopic, rmw_qos_profile_default);
  if (options_.publish_asi_pose)
    lean_pose_publisher = node_handle_->create_publisher<localization_msgs::msg::Pose2DWithCovarianceRelativeStamped>("cartographer_pose", rmw_qos_profile_default);
  submap_query_server_ = node_handle_->create_service<::cartographer_ros_msgs::srv::SubmapQuery>(kSubmapQueryServiceName,
        [this] (
                const std::shared_ptr<::cartographer_ros_msgs::srv::SubmapQuery::Request> request,
                std::shared_ptr<::cartographer_ros_msgs::srv::SubmapQuery::Response> response)
        {
          carto::common::MutexLocker lock(&mutex_);
          map_builder_bridge_.HandleSubmapQuery(request, response);
        });

  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    occupancy_grid_publisher_ =
      node_handle_->create_publisher<::nav_msgs::msg::OccupancyGrid>(kOccupancyGridTopic, rmw_qos_profile_default);
    occupancy_grid_thread_ =
        std::thread(&Node::SpinOccupancyGridThreadForever, this);
  }

  scan_matched_point_cloud_publisher_ =
    node_handle_->create_publisher<sensor_msgs::msg::PointCloud2>(
                                                                  kScanMatchedPointCloudTopic, rmw_qos_profile_default);

  wall_timers_.push_back(node_handle_->create_wall_timer(std::chrono::milliseconds((int)(options_.submap_publish_period_sec*1000)), std::bind(&Node::PublishSubmapList, this)));
  wall_timers_.push_back(node_handle_->create_wall_timer(std::chrono::milliseconds((int)(options_.pose_publish_period_sec*1000)), std::bind(&Node::PublishTrajectoryStates, this)));
}

Node::~Node() {
  {
    carto::common::MutexLocker lock(&mutex_);
    terminating_ = true;
  }
  if (occupancy_grid_thread_.joinable()) {
    occupancy_grid_thread_.join();
  }
}

::ros::NodeHandle* Node::node_handle() { return &node_handle_; }
::rclcpp::node::Node::SharedPtr Node::node_handle() { return node_handle_; }

MapBuilderBridge* Node::map_builder_bridge() { return &map_builder_bridge_; }

void Node::PublishSubmapList() {
  carto::common::MutexLocker lock(&mutex_);
  submap_list_publisher_->publish(map_builder_bridge_.GetSubmapList());
}

void Node::PublishTrajectoryStates() {
  carto::common::MutexLocker lock(&mutex_);
  for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
    const auto& trajectory_state = entry.second;

    geometry_msgs::msg::TransformStamped stamped_transform;
    stamped_transform.header.stamp = ToRos(trajectory_state.pose_estimate.time);

    const auto& tracking_to_local = trajectory_state.pose_estimate.pose;
    const Rigid3d tracking_to_map =
        trajectory_state.local_to_map * tracking_to_local;

    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    if (trajectory_state.pose_estimate.time !=
        last_scan_matched_point_cloud_time_) {
      scan_matched_point_cloud_publisher_->publish(ToPointCloud2Message(
          carto::common::ToUniversal(trajectory_state.pose_estimate.time),
          trajectory_state.trajectory_options.tracking_frame,
          carto::sensor::TransformPointCloud(
              trajectory_state.pose_estimate.point_cloud,
              tracking_to_local.inverse().cast<float>())));
      last_scan_matched_point_cloud_time_ = trajectory_state.pose_estimate.time;
    } else {
      // If we do not publish a new point cloud, we still allow time of the
      // published poses to advance.
      auto lt = map_builder_bridge_.last_time;
      stamped_transform.header.stamp = lt.sec < 0 ? rclcpp::Time::now() : lt;
    }

    if (trajectory_state.published_to_tracking != nullptr) {
      if (trajectory_state.trajectory_options.provide_odom_frame) {
        std::vector<geometry_msgs::TransformStamped> stamped_transforms;

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

      if (options_.publish_asi_pose) {
        localization_msgs::msg::Pose2DWithCovarianceRelativeStamped poseMsg;
        poseMsg.header.stamp = stamped_transform.header.stamp;
        poseMsg.header.frame_id = options_.map_frame;
        poseMsg.child_frame_id = options_.published_frame;

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
}

void Node::SpinOccupancyGridThreadForever() {
  for (;;) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    {
      carto::common::MutexLocker lock(&mutex_);
      if (terminating_) {
        return;
      }
    }
    if (node_handle_->count_subscribers(kOccupancyGridTopic) == 0) {
      continue;
    }
    const auto occupancy_grid = map_builder_bridge_.BuildOccupancyGrid();
    if (occupancy_grid != nullptr) {
      occupancy_grid_publisher_->publish(*occupancy_grid);
    }
  }
}

int Node::AddTrajectory(const TrajectoryOptions& options,
                        const cartographer_ros_msgs::SensorTopics& topics) {
  std::unordered_set<string> expected_sensor_ids;

  if (options.use_laser_scan) {
    expected_sensor_ids.insert(topics.laser_scan_topic);
  }
  if (options.use_multi_echo_laser_scan) {
    expected_sensor_ids.insert(topics.multi_echo_laser_scan_topic);
  }
  if (options.num_point_clouds > 0) {
    for (int i = 0; i < options.num_point_clouds; ++i) {
      string topic = topics.point_cloud2_topic;
      if (options.num_point_clouds > 1) {
        topic += "_" + std::to_string(i + 1);
      }
      expected_sensor_ids.insert(topic);
    }
  }
  if (options.trajectory_builder_options.trajectory_builder_2d_options()
          .use_imu_data()) {
    expected_sensor_ids.insert(topics.imu_topic);
  }
  if (options.use_odometry) {
    expected_sensor_ids.insert(topics.odometry_topic);
  }
  return map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
}

void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const cartographer_ros_msgs::SensorTopics& topics,
                             const int trajectory_id) {
  if (options.use_laser_scan) {
    const string topic = topics.laser_scan_topic;
    laser_scan_subscribers_[trajectory_id] =
        node_handle_.subscribe<sensor_msgs::LaserScan>(
            topic, kInfiniteSubscriberQueueSize,
            boost::function<void(const sensor_msgs::LaserScan::ConstPtr&)>(
                [this, trajectory_id,
                 topic](const sensor_msgs::LaserScan::ConstPtr& msg) {
                  map_builder_bridge_.sensor_bridge(trajectory_id)
                      ->HandleLaserScanMessage(topic, msg);
                }));
  }

  if (options.use_multi_echo_laser_scan) {
    const string topic = topics.multi_echo_laser_scan_topic;
    multi_echo_laser_scan_subscribers_[trajectory_id] =
        node_handle_.subscribe<sensor_msgs::MultiEchoLaserScan>(
            topic, kInfiniteSubscriberQueueSize,
            boost::function<void(
                const sensor_msgs::MultiEchoLaserScan::ConstPtr&)>(
                [this, trajectory_id,
                 topic](const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
                  map_builder_bridge_.sensor_bridge(trajectory_id)
                      ->HandleMultiEchoLaserScanMessage(topic, msg);
                }));
  }

  std::vector<::ros::Subscriber> grouped_point_cloud_subscribers;
  if (options.num_point_clouds > 0) {
    for (int i = 0; i < options.num_point_clouds; ++i) {
      string topic = topics.point_cloud2_topic;
      if (options.num_point_clouds > 1) {
        topic += "_" + std::to_string(i + 1);
      }
      grouped_point_cloud_subscribers.push_back(node_handle_.subscribe(
          topic, kInfiniteSubscriberQueueSize,
          boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>(
              [this, trajectory_id,
               topic](const sensor_msgs::PointCloud2::ConstPtr& msg) {
                map_builder_bridge_.sensor_bridge(trajectory_id)
                    ->HandlePointCloud2Message(topic, msg);
              })));
    }
    point_cloud_subscribers_[trajectory_id] = grouped_point_cloud_subscribers;
  }

  if (options.trajectory_builder_options.trajectory_builder_2d_options()
          .use_imu_data()) {
    string topic = topics.imu_topic;
    imu_subscribers_[trajectory_id] = node_handle_.subscribe<sensor_msgs::Imu>(
        topic, kInfiniteSubscriberQueueSize,
        boost::function<void(const sensor_msgs::Imu::ConstPtr&)>(
            [this, trajectory_id,
             topic](const sensor_msgs::Imu::ConstPtr& msg) {
              map_builder_bridge_.sensor_bridge(trajectory_id)
                  ->HandleImuMessage(topic, msg);
            }));
  }

  if (options.use_odometry) {
    string topic = topics.odometry_topic;
    odom_subscribers_[trajectory_id] =
        node_handle_.subscribe<nav_msgs::Odometry>(
            topic, kInfiniteSubscriberQueueSize,
            boost::function<void(const nav_msgs::Odometry::ConstPtr&)>(
                [this, trajectory_id,
                 topic](const nav_msgs::Odometry::ConstPtr& msg) {
                  map_builder_bridge_.sensor_bridge(trajectory_id)
                      ->HandleOdometryMessage(topic, msg);
                }));
  }

  is_active_trajectory_[trajectory_id] = true;
}

bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d() &&
      options.trajectory_builder_options.has_trajectory_builder_2d_options()) {
    // Using point clouds is only supported in 3D.
    if (options.num_point_clouds == 0) {
      return true;
    }
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d() &&
      options.trajectory_builder_options.has_trajectory_builder_3d_options()) {
    if (options.num_point_clouds != 0) {
      return true;
    }
  }
  return false;
}

bool Node::ValidateTopicName(
    const ::cartographer_ros_msgs::SensorTopics& topics,
    const TrajectoryOptions& options) {
  if (!IsTopicNameUnique(topics.laser_scan_topic, laser_scan_subscribers_)) {
    return false;
  }
  if (!IsTopicNameUnique(topics.multi_echo_laser_scan_topic,
                         multi_echo_laser_scan_subscribers_)) {
    return false;
  }
  if (!IsTopicNameUnique(topics.imu_topic, imu_subscribers_)) {
    return false;
  }
  if (!IsTopicNameUnique(topics.odometry_topic, odom_subscribers_)) {
    return false;
  }
  for (auto& subscribers : point_cloud_subscribers_) {
    string topic = topics.point_cloud2_topic;
    int count = 0;
    for (auto& subscriber : subscribers.second) {
      if (options.num_point_clouds > 1) {
        topic += "_" + std::to_string(count + 1);
        ++count;
      }
      if (subscriber.getTopic() == topic) {
        LOG(ERROR) << "Topic name [" << topic << "] is already used";
        return false;
      }
    }
  }
  return true;
}

bool Node::HandleStartTrajectory(
    ::cartographer_ros_msgs::StartTrajectory::Request& request,
    ::cartographer_ros_msgs::StartTrajectory::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  TrajectoryOptions options;
  if (!FromRosMessage(request.options, &options) ||
      !Node::ValidateTrajectoryOptions(options)) {
    LOG(ERROR) << "Invalid trajectory options.";
    return false;
  }
  if (!Node::ValidateTopicName(request.topics, options)) {
    LOG(ERROR) << "Invalid topics.";
    return false;
  }

  std::unordered_set<string> expected_sensor_ids;
  const int trajectory_id = AddTrajectory(options, request.topics);
  LaunchSubscribers(options, request.topics, trajectory_id);

  is_active_trajectory_[trajectory_id] = true;
  return true;
}

void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  cartographer_ros_msgs::SensorTopics topics;
  topics.laser_scan_topic = kLaserScanTopic;
  topics.multi_echo_laser_scan_topic = kMultiEchoLaserScanTopic;
  topics.point_cloud2_topic = kPointCloud2Topic;
  topics.imu_topic = kImuTopic;
  topics.odometry_topic = kOdometryTopic;

  const int trajectory_id = AddTrajectory(options, topics);
  LaunchSubscribers(options, topics, trajectory_id);
}

bool Node::HandleFinishTrajectory(
    ::cartographer_ros_msgs::FinishTrajectory::Request& request,
    ::cartographer_ros_msgs::FinishTrajectory::Response& response) {
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

  ShutdownSubscriber(laser_scan_subscribers_, trajectory_id);
  ShutdownSubscriber(multi_echo_laser_scan_subscribers_, trajectory_id);
  ShutdownSubscriber(odom_subscribers_, trajectory_id);
  ShutdownSubscriber(imu_subscribers_, trajectory_id);

  if (point_cloud_subscribers_.count(trajectory_id) != 0) {
    for (auto& entry : point_cloud_subscribers_[trajectory_id]) {
      LOG(INFO) << "Shutdown the subscriber of [" << entry.getTopic() << "]";
      entry.shutdown();
    }
    CHECK_EQ(point_cloud_subscribers_.erase(trajectory_id), 1);
  }
  map_builder_bridge_.FinishTrajectory(trajectory_id);
  is_active_trajectory_[trajectory_id] = false;
  return true;
}

bool Node::HandleWriteAssets(
    ::cartographer_ros_msgs::WriteAssets::Request& request,
    ::cartographer_ros_msgs::WriteAssets::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.WriteAssets(request.stem);
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
}  // namespace cartographer_ros
