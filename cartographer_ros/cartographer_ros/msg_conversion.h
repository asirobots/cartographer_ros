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

#ifndef CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_MSG_CONVERSION_H_
#define CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_MSG_CONVERSION_H_

#include <boost/array.hpp>

#include "cartographer/common/port.h"
#include "cartographer/kalman_filter/pose_tracker.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/conversions.h"
#include "pcl/PCLPointCloud2.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

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

sensor_msgs::msg::PointCloud2 ToPointCloud2Message(
    int64 timestamp, const string& frame_id,
    const ::cartographer::sensor::PointCloud& point_cloud);

geometry_msgs::msg::Transform ToGeometryMsgTransform(
    const ::cartographer::transform::Rigid3d& rigid3d);

geometry_msgs::msg::Pose ToGeometryMsgPose(
    const ::cartographer::transform::Rigid3d& rigid3d);

::cartographer::sensor::PointCloudWithIntensities ToPointCloudWithIntensities(
    const sensor_msgs::msg::LaserScan& msg);

::cartographer::sensor::PointCloudWithIntensities ToPointCloudWithIntensities(
    const sensor_msgs::msg::MultiEchoLaserScan& msg);

::cartographer::sensor::PointCloudWithIntensities ToPointCloudWithIntensities(
    const sensor_msgs::msg::PointCloud2& message);

::cartographer::transform::Rigid3d ToRigid3d(
    const geometry_msgs::msg::TransformStamped& transform);

::cartographer::transform::Rigid3d ToRigid3d(const geometry_msgs::msg::Pose& pose);

Eigen::Vector3d ToEigen(const geometry_msgs::msg::Vector3& vector3);

Eigen::Quaterniond ToEigen(const geometry_msgs::msg::Quaternion& quaternion);

::cartographer::kalman_filter::PoseCovariance ToPoseCovariance(
    const boost::array<double, 36>& covariance);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_MSG_CONVERSION_H_
