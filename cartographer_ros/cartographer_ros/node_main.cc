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

#include <string>
#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/make_unique.h"
#include "cartographer_ros/asi_node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "tf2_ros/transform_listener.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(map_filename_to_load, "", "If non-empty, filename of a map to load.");
DEFINE_string(map_filename_to_store, "", "If non-empty, filename of a map to save on exit (Ctrl+C or SIGINT).");

namespace cartographer_ros {

void Run() {
  tf2_ros::Buffer tf_buffer(tf2::Duration(std::chrono::minutes(2)), false); // original timeout was 11 days; that seems overkill (and memory kill)
  tf2_ros::TransformListener tf(tf_buffer); // makes its own node and thread inside
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  AsiNode node(node_options, &tf_buffer);
  if (!FLAGS_map_filename_to_load.empty()) {
    node.LoadMap(FLAGS_map_filename_to_load);
  }
  node.StartTrajectoryWithDefaultTopics(trajectory_options);

  rclcpp::spin(node.node_handle());

  node.FinishAllTrajectories();
  node.RunFinalOptimization(); // not sure how to wait for this; it seems async

  if (!FLAGS_map_filename_to_store.empty()) {
    node.SerializeState(FLAGS_map_filename_to_store);
  }
}

}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::rclcpp::init(argc, argv);
  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
}
