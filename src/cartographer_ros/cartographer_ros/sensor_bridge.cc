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

#include "cartographer_ros/sensor_bridge.h"

#include "cartographer/kalman_filter/pose_tracker.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

namespace {

const string& CheckNoLeadingSlash(const string& frame_id) {
  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/');
  }
  return frame_id;
}

}  // namespace

SensorBridge::SensorBridge(
    const string& tracking_frame, const double lookup_transform_timeout_sec,
    tf2_ros::Buffer* const tf_buffer,
    carto::mapping::TrajectoryBuilder* const trajectory_builder)
    : tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),
      trajectory_builder_(trajectory_builder) {}

void SensorBridge::HandleOdometryMessage(
                                         const string& sensor_id, const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  const carto::common::Time time = FromRos(msg->header.stamp);
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->child_frame_id));
  if (sensor_to_tracking != nullptr) {
    trajectory_builder_->AddOdometerData(
        sensor_id, time,
        ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse());
  }
}

void SensorBridge::HandleImuMessage(const string& sensor_id,
                                    const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  CHECK_NE(msg->linear_acceleration_covariance[0], -1);
  CHECK_NE(msg->angular_velocity_covariance[0], -1);
  const carto::common::Time time = FromRos(msg->header.stamp);
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->header.frame_id));
  if (sensor_to_tracking != nullptr) {
    CHECK(sensor_to_tracking->translation().norm() < 1e-5)
        << "The IMU frame must be colocated with the tracking frame. "
           "Transforming linear acceleration into the tracking frame will "
           "otherwise be imprecise.";
    trajectory_builder_->AddImuData(
        sensor_id, time,
        sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),
        sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity));
  }
}

void SensorBridge::HandleLaserScanMessage(
                                          const string& sensor_id, const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
  HandleRangefinder(sensor_id, FromRos(msg->header.stamp), msg->header.frame_id,
                    carto::sensor::ToPointCloud(ToCartographer(*msg)));
}

void SensorBridge::HandleMultiEchoLaserScanMessage(
    const string& sensor_id,
    const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr& msg) {
  HandleRangefinder(sensor_id, FromRos(msg->header.stamp), msg->header.frame_id,
                    carto::sensor::ToPointCloud(ToCartographer(*msg)));
}

#define I_AM_LITTLE (((union { unsigned x; unsigned char c; }){1}).c)

std::function<float(std::size_t)> GenerateAccessorFunction(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, std::string name){
  auto fieldIter = std::find_if(msg->fields.begin(), msg->fields.end(), [name](const sensor_msgs::msg::PointField& f){ return f.name == name; });
  if (fieldIter == msg->fields.end())
    return [](std::size_t){ return 0; }; // write zeros if we don't have that field
  auto field = *fieldIter;

  // shouldn't need to use msg->is_dense because we aren't iterating on width & height

  if ((I_AM_LITTLE && msg->is_bigendian) || (!I_AM_LITTLE && !msg->is_bigendian))
    throw "Not implemented!"; // not sure what the right-cross platform call is to reverse bytes

  switch (field.datatype){
    case sensor_msgs::msg::PointField::INT8:
      return [msg, field](std::size_t t){ return static_cast<float>(*reinterpret_cast<int8_t*>(&msg->data[t + field.offset])); };
    case sensor_msgs::msg::PointField::UINT8:
      return [msg, field](std::size_t t){ return static_cast<float>(msg->data[t + field.offset]); };
    case sensor_msgs::msg::PointField::INT16:
      return [msg, field](std::size_t t){ return static_cast<float>(*reinterpret_cast<int16_t*>(&msg->data[t + field.offset])); };
    case sensor_msgs::msg::PointField::UINT16:
      return [msg, field](std::size_t t){ return static_cast<float>(*reinterpret_cast<uint16_t*>(&msg->data[t + field.offset])); };
    case sensor_msgs::msg::PointField::INT32:
      return [msg, field](std::size_t t){ return static_cast<float>(*reinterpret_cast<int32_t*>(&msg->data[t + field.offset])); };
    case sensor_msgs::msg::PointField::UINT32:
      return [msg, field](std::size_t t){ return static_cast<float>(*reinterpret_cast<uint32_t*>(&msg->data[t + field.offset])); };
    case sensor_msgs::msg::PointField::FLOAT32:
      return [msg, field](std::size_t t){ return *reinterpret_cast<float*>(&msg->data[t + field.offset]); };
    case sensor_msgs::msg::PointField::FLOAT64:
      return [msg, field](std::size_t t){ return static_cast<float>(*reinterpret_cast<double*>(&msg->data[t + field.offset])); };
  }

  throw "Not implemented!";
}
carto::sensor::PointCloud RosPointCloudToCarto(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {

  auto x_accessor = GenerateAccessorFunction(msg, "x");
  auto y_accessor = GenerateAccessorFunction(msg, "y");
  auto z_accessor = GenerateAccessorFunction(msg, "z");

  carto::sensor::PointCloud point_cloud;
  // TODO: reserve the right amount of space

  for(std::size_t p = 0; p < msg->data.size(); p += msg->point_step) {
    auto x = x_accessor(p);
    auto y = y_accessor(p);
    auto z = z_accessor(p);

    point_cloud.emplace_back(x, y, z);
  }
  return point_cloud; // should do a move
}

void SensorBridge::HandlePointCloud2Message(
  const string& sensor_id, const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {

  auto point_cloud = RosPointCloudToCarto(msg);
  HandleRangefinder(sensor_id, FromRos(msg->header.stamp), msg->header.frame_id, point_cloud);
}

const TfBridge& SensorBridge::tf_bridge() const { return tf_bridge_; }

void SensorBridge::HandleRangefinder(const string& sensor_id,
                                     const carto::common::Time time,
                                     const string& frame_id,
                                     const carto::sensor::PointCloud& ranges) {
  const auto sensor_to_tracking =
      tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash(frame_id));
  if (sensor_to_tracking != nullptr) {
    trajectory_builder_->AddRangefinderData(
        sensor_id, time, sensor_to_tracking->translation().cast<float>(),
        carto::sensor::TransformPointCloud(ranges,
                                           sensor_to_tracking->cast<float>()));
  }
}

}  // namespace cartographer_ros
