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

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

namespace pcl_conversions {
  inline
  void toPCL(const builtin_interfaces::msg::Time &stamp, pcl::uint64_t &pcl_stamp)
  {
    pcl_stamp = stamp.sec * 1000000 + stamp.nanosec / 1000ull;
  }

  inline
  void toPCL(const sensor_msgs::msg::PointField &pf, pcl::PCLPointField &pcl_pf)
  {
    pcl_pf.name = pf.name;
    pcl_pf.offset = pf.offset;
    pcl_pf.datatype = pf.datatype;
    pcl_pf.count = pf.count;
  }

  inline
  void toPCL(const std::vector<sensor_msgs::msg::PointField> &pfs, std::vector<pcl::PCLPointField> &pcl_pfs)
  {
    pcl_pfs.resize(pfs.size());
    std::vector<sensor_msgs::msg::PointField>::const_iterator it = pfs.begin();
    int i = 0;
    for(; it != pfs.end(); ++it, ++i) {
      toPCL(*(it), pcl_pfs[i]);
    }
  }

  inline
  void toPCL(const std_msgs::msg::Header &header, pcl::PCLHeader &pcl_header)
  {
    toPCL(header.stamp, pcl_header.stamp);
    // FIXME: Seq doesn't exist anymore
    //pcl_header.seq = header.seq;
    pcl_header.seq = 0;
    pcl_header.frame_id = header.frame_id;
  }

  inline
  void copyPointCloud2MetaData(const sensor_msgs::msg::PointCloud2 &pc2, pcl::PCLPointCloud2 &pcl_pc2)
  {
    toPCL(pc2.header, pcl_pc2.header);
    pcl_pc2.height = pc2.height;
    pcl_pc2.width = pc2.width;
    toPCL(pc2.fields, pcl_pc2.fields);
    pcl_pc2.is_bigendian = pc2.is_bigendian;
    pcl_pc2.point_step = pc2.point_step;
    pcl_pc2.row_step = pc2.row_step;
    pcl_pc2.is_dense = pc2.is_dense;
  }

  inline
  void toPCL(const sensor_msgs::msg::PointCloud2 &pc2, pcl::PCLPointCloud2 &pcl_pc2)
  {
    copyPointCloud2MetaData(pc2, pcl_pc2);
    pcl_pc2.data = pc2.data;
  }
}

namespace pcl {
  template<typename T>
  void fromROSMsg(const sensor_msgs::msg::PointCloud2 &cloud, pcl::PointCloud<T> &pcl_cloud)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
  }
}

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
  CHECK_NE(msg->linear_acceleration_covariance[0], -1)
      << "Your IMU data claims to not contain linear acceleration measurements "
         "by setting linear_acceleration_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
  CHECK_NE(msg->angular_velocity_covariance[0], -1)
      << "Your IMU data claims to not contain angular velocity measurements "
         "by setting angular_velocity_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";

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
                    ToPointCloudWithIntensities(*msg).points);
}

void SensorBridge::HandleMultiEchoLaserScanMessage(
    const string& sensor_id,
    const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr& msg) {
  HandleRangefinder(sensor_id, FromRos(msg->header.stamp), msg->header.frame_id,
                    ToPointCloudWithIntensities(*msg).points);
}

void SensorBridge::HandlePointCloud2Message(
    const string& sensor_id, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
  pcl::fromROSMsg(*msg, pcl_point_cloud);
  carto::sensor::PointCloud point_cloud;
  for (const auto& point : pcl_point_cloud) {
    point_cloud.emplace_back(point.x, point.y, point.z);
  }
  HandleRangefinder(sensor_id, FromRos(msg->header.stamp), msg->header.frame_id,
                    point_cloud);
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
