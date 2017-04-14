-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  use_odometry = false, -- relying on our lean message instead
  use_laser_scan = true,
  use_multi_echo_laser_scan = false,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.001,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.02,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.sparse_pose_graph.optimize_every_n_scans = 10

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.num_odometry_states = 100
TRAJECTORY_BUILDER_2D.laser_min_range = 0.2
TRAJECTORY_BUILDER_2D.laser_max_range = 12.
-- missing return values inserted at this length:
TRAJECTORY_BUILDER_2D.laser_missing_echo_ray_length = 10.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.5 -- default 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(15.) -- default 20deg

SPARSE_POSE_GRAPH.optimization_problem.huber_scale = 1e2

return options
