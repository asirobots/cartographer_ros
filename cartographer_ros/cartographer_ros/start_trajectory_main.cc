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
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/srv/start_trajectory.hpp"
#include "cartographer_ros_msgs/msg/trajectory_options.hpp"
#include "gflags/gflags.h"
#include "rclcpp/rclcpp.hpp"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

namespace cartographer_ros {
namespace {

TrajectoryOptions LoadOptions() {
  auto file_resolver = cartographer::common::make_unique<
      cartographer::common::ConfigurationFileResolver>(
      std::vector<string>{FLAGS_configuration_directory});
  const string code =
      file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
  auto lua_parameter_dictionary =
      cartographer::common::LuaParameterDictionary::NonReferenceCounted(
          code, std::move(file_resolver));
  return CreateTrajectoryOptions(lua_parameter_dictionary.get());
}

bool Run() {
  rclcpp::node::Node node_handle("start_trajectory_node");
  auto client = node_handle.create_client<cartographer_ros_msgs::srv::StartTrajectory>(
          kStartTrajectoryServiceName);
  auto srv = std::make_shared<cartographer_ros_msgs::srv::StartTrajectory::Request>();
  srv->options = *ToRosMessage(LoadOptions());
  srv->topics.laser_scan_topic = kLaserScanTopic;
  srv->topics.multi_echo_laser_scan_topic = kMultiEchoLaserScanTopic;
  srv->topics.point_cloud2_topic = kPointCloud2Topic;
  srv->topics.imu_topic = kImuTopic;
  srv->topics.odometry_topic = kOdometryTopic;

  if (!client->wait_for_service(std::chrono::seconds(3))) {
    LOG(ERROR) << "Error connecting trajectory service.";
    return false;
  }
  auto future = client->async_send_request(srv);
  auto status = future.wait_for(std::chrono::seconds(7));
  if (status != std::future_status::ready) {
    LOG(ERROR) << "Error starting trajectory.";
    return false;
  }
  LOG(INFO) << "Started trajectory " << future.get()->trajectory_id;
  return true;
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage(
      "\n\n"
      "Convenience tool around the start_trajectory service. This takes a Lua "
      "file that is accepted by the node as well and starts a new trajectory "
      "using its settings.\n");
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::rclcpp::init(argc, argv);

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  int exit_code = cartographer_ros::Run() ? 0 : 1;
  ::rclcpp::shutdown();
  return exit_code;
}
