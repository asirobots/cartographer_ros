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

#ifndef CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_TIME_CONVERSION_H_
#define CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_TIME_CONVERSION_H_

#include "cartographer/common/time.h"
#include <rcl/time.h>
#include <builtin_interfaces/msg/time.hpp>

namespace cartographer_ros {

builtin_interfaces::msg::Time ToRos(::cartographer::common::Time time);

::cartographer::common::Time FromRos(const builtin_interfaces::msg::Time& time);

    class TimeStub // stolen rclcpp::Time in a newer version
    {
    public:
        template<rcl_time_source_type_t ClockT = RCL_SYSTEM_TIME>
        static TimeStub
        now()
        {
          rcl_time_point_value_t rcl_now = 0;
          rcl_ret_t ret = RCL_RET_ERROR;
          if (ClockT == RCL_ROS_TIME) {
            throw std::runtime_error("RCL_ROS_TIME is currently not implemented.");
            ret = false;
          } else if (ClockT == RCL_SYSTEM_TIME) {
            ret = rcl_system_time_now(&rcl_now);
          } else if (ClockT == RCL_STEADY_TIME) {
            ret = rcl_steady_time_now(&rcl_now);
          }
          if (ret != RCL_RET_OK) {
            throw "Could not get current time";
          }

          return TimeStub(std::move(rcl_now));
        }

        operator builtin_interfaces::msg::Time() const
        {
          builtin_interfaces::msg::Time msg_time;
          msg_time.sec = static_cast<std::int32_t>(RCL_NS_TO_S(rcl_time_));
          msg_time.nanosec = static_cast<std::uint32_t>(rcl_time_ % (1000 * 1000 * 1000));
          return msg_time;
        }

    private:
        rcl_time_point_value_t rcl_time_;

        TimeStub(std::uint32_t sec, std::uint32_t nanosec)
          : rcl_time_(RCL_S_TO_NS(sec) + nanosec)
        {}

        explicit TimeStub(rcl_time_point_value_t && rcl_time)
          : rcl_time_(std::forward<decltype(rcl_time)>(rcl_time))
        {}
    };


}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_TIME_CONVERSION_H_
