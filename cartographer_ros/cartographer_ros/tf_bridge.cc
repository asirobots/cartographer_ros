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

#include "cartographer/common/make_unique.h"

#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/tf_bridge.h"


namespace cartographer_ros {

TfBridge::TfBridge(const string& tracking_frame,
                   const double lookup_transform_timeout_sec,
                   const tf2_ros::Buffer* buffer)
    : tracking_frame_(tracking_frame),
      lookup_transform_timeout_sec_(lookup_transform_timeout_sec),
      buffer_(buffer) {}

std::unique_ptr<::cartographer::transform::Rigid3d> TfBridge::LookupToTracking(
    const ::cartographer::common::Time time, const string& frame_id) const {
  try {
    const auto requested_tf_time = ToRos(time);
    const auto requested_time = std::chrono::seconds(requested_tf_time.sec)
                                + std::chrono::nanoseconds(requested_tf_time.nanosec);
    // auto timeout = ::tf2::durationFromSec(lookup_transform_timeout_sec_); // won't compile? lame
    tf2::Duration timeout(tf2::Duration::rep(lookup_transform_timeout_sec_ * 1e9));
    auto timestamp = ::tf2::TimePoint(requested_time);

  TRY_AGAIN:
    try {
      auto transform = buffer_->lookupTransform(tracking_frame_, frame_id, timestamp, timeout);
      return ::cartographer::common::make_unique<cartographer::transform::Rigid3d>(ToRigid3d(transform));
    } catch (const tf2::ExtrapolationException& ex) {
      timestamp = tf2::TimePointZero; // just take latest if our timestamps match or there is only one
      timeout = tf2::Duration::zero();
      goto TRY_AGAIN;
    }
  } catch (const tf2::TransformException& ex) {
    LOG(WARNING) << ex.what();
  }
  return nullptr;
}

}  // namespace cartographer_ros
