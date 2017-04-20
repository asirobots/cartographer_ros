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
  lookup_transform_timeout_sec = 0.01,
  submap_publish_period_sec = 0.4,
  pose_publish_period_sec = 0.04,
  publish_asi_pose = false
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.sparse_pose_graph.optimize_every_n_scans = 50 -- going to expect 20 per second

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.num_odometry_states = 200
TRAJECTORY_BUILDER_2D.laser_min_range = 0.2
TRAJECTORY_BUILDER_2D.laser_max_range = 30.
-- missing return values inserted at this length:
TRAJECTORY_BUILDER_2D.laser_missing_echo_ray_length = 5.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = .2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1)

-- we can do a full 360deg turn in about 4 seconds, that's about 40m distance on the tip per second
-- however, we are doing 20 scans per second
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 2. -- default 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.) -- default 20deg
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.3
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.3

SPARSE_POSE_GRAPH.optimization_problem.huber_scale = 100

return options
