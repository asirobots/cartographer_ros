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

#include "cartographer_ros/time_conversion.h"

#include "cartographer/common/time.h"
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace cartographer_ros {

builtin_interfaces::msg::Time ToRos(::cartographer::common::Time time) {
  int64 uts_timestamp = ::cartographer::common::ToUniversal(time);
  int64 ns_since_unix_epoch =
      (uts_timestamp -
       ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
           10000000ll) *
      100ll;
  builtin_interfaces::msg::Time ros_time;
  ros_time.sec = static_cast<builtin_interfaces::msg::Time::_sec_type>(ns_since_unix_epoch / 1000000000);
  ros_time.nanosec = ns_since_unix_epoch % 1000000000;
  return ros_time;
}

// TODO(pedrofernandez): Write test.
::cartographer::common::Time FromRos(const builtin_interfaces::msg::Time& time) {
  // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
  // exactly 719162 days before the Unix epoch.
  return ::cartographer::common::FromUniversal(
      (time.sec +
       ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
          10000000ll +
      (time.nanosec + 50) / 100);  // + 50 to get the rounding correct.
}

}  // namespace cartographer_ros