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

#ifndef CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H_
#define CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/msg/submap_entry.hpp"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace cartographer_ros {

class MapBuilderBridge {
 public:
  struct TrajectoryState {
    cartographer::mapping::TrajectoryBuilder::PoseEstimate pose_estimate;
    cartographer::transform::Rigid3d local_to_map;
    std::unique_ptr<cartographer::transform::Rigid3d> published_to_tracking;
    TrajectoryOptions trajectory_options;
  };

  MapBuilderBridge(const NodeOptions& node_options, tf2_ros::Buffer* tf_buffer);

  MapBuilderBridge(const MapBuilderBridge&) = delete;
  MapBuilderBridge& operator=(const MapBuilderBridge&) = delete;

  void LoadMap(const std::string& map_filename);
  int AddTrajectory(const std::unordered_set<string>& expected_sensor_ids,
                    const TrajectoryOptions& trajectory_options);
  void FinishTrajectory(int trajectory_id);
  void RunFinalOptimization();
  void SerializeState(const string& filename);

  void HandleSubmapQuery(::cartographer_ros_msgs::srv::SubmapQuery::Request::ConstSharedPtr request,
                         ::cartographer_ros_msgs::srv::SubmapQuery::Response::SharedPtr response);

  cartographer_ros_msgs::msg::SubmapList GetSubmapList();
  std::unordered_map<int, TrajectoryState> GetTrajectoryStates();
  visualization_msgs::msg::MarkerArray GetTrajectoryNodeList();
  visualization_msgs::msg::MarkerArray GetConstraintList();

  SensorBridge* sensor_bridge(int trajectory_id);

  builtin_interfaces::msg::Time last_time;

 private:
  const NodeOptions node_options_;
  cartographer::mapping::MapBuilder map_builder_;
  tf2_ros::Buffer* const tf_buffer_;

  // These are keyed with 'trajectory_id'.
  std::unordered_map<int, TrajectoryOptions> trajectory_options_;
  std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H_
