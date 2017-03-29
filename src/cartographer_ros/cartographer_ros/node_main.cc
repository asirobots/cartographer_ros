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
#include "cartographer_ros/node.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"
#include <rclcpp/rclcpp.hpp>
#include "cartographer_ros_msgs/srv/finish_trajectory.hpp"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

namespace cartographer_ros {
namespace {

NodeOptions LoadOptions() {
  auto file_resolver = cartographer::common::make_unique<
      cartographer::common::ConfigurationFileResolver>(
      std::vector<string>{FLAGS_configuration_directory});
  const string code =
      file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  return CreateNodeOptions(&lua_parameter_dictionary);
}

void Run() {
  const auto options = LoadOptions();
  constexpr double kTfBufferCacheTimeInNs = 1e15; // 1 million seconds
  tf2_ros::Buffer tf_buffer{::tf2::Duration(kTfBufferCacheTimeInNs)};
  tf2_ros::TransformListener tf(tf_buffer);
  Node node(options, &tf_buffer);
  node.Initialize();

  int trajectory_id = -1;
  std::unordered_set<string> expected_sensor_ids;

  // For 2D SLAM, subscribe to exactly one horizontal laser.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber;
  if (options.use_laser_scan) {
    laser_scan_subscriber = node.node_handle()->create_subscription<sensor_msgs::msg::LaserScan>(
                                                                                                 kLaserScanTopic,
                                                                                                 [&](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
                                                                                                   node.map_builder_bridge()
                                                                                                   ->sensor_bridge(trajectory_id)
                                                                                                   ->HandleLaserScanMessage(kLaserScanTopic, msg);
                               }, rmw_qos_profile_default);
    expected_sensor_ids.insert(kLaserScanTopic);
  }

  rclcpp::Subscription<sensor_msgs::msg::MultiEchoLaserScan>::SharedPtr multi_echo_laser_scan_subscriber;
  if (options.use_multi_echo_laser_scan) {
    multi_echo_laser_scan_subscriber = node.node_handle()->create_subscription<sensor_msgs::msg::MultiEchoLaserScan>(
                                                                                                 kMultiEchoLaserScanTopic,
                                                                                                 [&](sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr msg) {
                                                                                                   node.map_builder_bridge()
                                                                                                   ->sensor_bridge(trajectory_id)
                                                                                                   ->HandleMultiEchoLaserScanMessage(kMultiEchoLaserScanTopic, msg);
                               }, rmw_qos_profile_default);
    expected_sensor_ids.insert(kMultiEchoLaserScanTopic);
  }

  // For 3D SLAM, subscribe to all point clouds topics.
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> point_cloud_subscribers;
  if (options.num_point_clouds > 0) {
    for (int i = 0; i < options.num_point_clouds; ++i) {
      string topic = kPointCloud2Topic;
      if (options.num_point_clouds > 1) {
        topic += "_" + std::to_string(i + 1);
      }
      point_cloud_subscribers.push_back(node.node_handle()->create_subscription<sensor_msgs::msg::PointCloud2>(
          topic,
	  [&, topic](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                node.map_builder_bridge()
                    ->sensor_bridge(trajectory_id)
                    ->HandlePointCloud2Message(topic, msg);
	  }, rmw_qos_profile_default));
      expected_sensor_ids.insert(topic);
    }
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
  if (options.map_builder_options.use_trajectory_builder_3d() ||
      (options.map_builder_options.use_trajectory_builder_2d() &&
       options.map_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    imu_subscriber = node.node_handle()->create_subscription<sensor_msgs::msg::Imu>(
                  kImuTopic,
                  [&](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
                        node.map_builder_bridge()
                        ->sensor_bridge(trajectory_id)
                        ->HandleImuMessage(kImuTopic, msg);
                  }, rmw_qos_profile_default);
    expected_sensor_ids.insert(kImuTopic);
  }

  // For both 2D and 3D SLAM, odometry is optional.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber;
  if (options.use_odometry) {
    odometry_subscriber = node.node_handle()->create_subscription<nav_msgs::msg::Odometry>(
        kOdometryTopic,
        [&](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
              node.map_builder_bridge()
                  ->sensor_bridge(trajectory_id)
                  ->HandleOdometryMessage(kOdometryTopic, msg);
        }, rmw_qos_profile_default);
    expected_sensor_ids.insert(kOdometryTopic);
  }

  trajectory_id = node.map_builder_bridge()->AddTrajectory(
      expected_sensor_ids, options.tracking_frame);

  ::rclcpp::service::Service<::cartographer_ros_msgs::srv::FinishTrajectory>::SharedPtr finish_trajectory_server;

  finish_trajectory_server = node.node_handle()->create_service<::cartographer_ros_msgs::srv::FinishTrajectory>(kFinishTrajectoryServiceName,
        [&] (
             const std::shared_ptr<::cartographer_ros_msgs::srv::FinishTrajectory::Request> request,
             std::shared_ptr<::cartographer_ros_msgs::srv::FinishTrajectory::Response> response)
        {
          (void)response;
          const int previous_trajectory_id = trajectory_id;
          trajectory_id = node.map_builder_bridge()->AddTrajectory(
                                                                   expected_sensor_ids, options.tracking_frame);
          node.map_builder_bridge()->FinishTrajectory(previous_trajectory_id);
          node.map_builder_bridge()->WriteAssets(request->stem);
        });

  rclcpp::spin(node.node_handle());

  node.map_builder_bridge()->FinishTrajectory(trajectory_id);
}

}  // namespace
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
  ::rclcpp::shutdown();
}
