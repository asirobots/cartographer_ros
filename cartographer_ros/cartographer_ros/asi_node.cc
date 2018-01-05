//
// Created by brannon on 8/7/17.
//

#include "asi_node.h"
#include "time_conversion.h"
#include "msg_conversion.h"
#include "tf2/time.h"
#include "cartographer/transform/rigid_transform.h"
#include "asiframework_msgs/msg/asi_time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "framework/os/Time.hpp"

DEFINE_string(odometry_output_topic, "", "Output topic for nav_msgs::Odometry");

DEFINE_string(pose_output_topic, "", "Output topic for geometry_msgs::PoseStamped");

DEFINE_string(pose_with_covariance_output_topic, "", "Output topic for geometry_msgs::PoseWithCovarianceStamped");

DEFINE_string(pose_input_topic, "", "Input topic for geometry_msgs::PoseStamped");

DEFINE_string(pose_with_covariance_input_topic, "", "Input topic for geometry_msgs::PoseWithCovarianceStamped");

DEFINE_string(twist_input_topic, "", "Input topic for geometry_msgs::TwistStamped");

DEFINE_string(asi_clock_input_topic, "", "Input topic for asiframework::AsiTime");

DEFINE_bool(disable_default_imu_topic, true, "By default, the default IMU topic is disabled");

cartographer_ros::AsiNode::AsiNode(const cartographer_ros::NodeOptions &node_options, tf2_ros::Buffer *tf_buffer)
    : Node(node_options, tf_buffer) {

  tf_buffer_ = tf_buffer;

  if (!FLAGS_odometry_output_topic.empty()) {
    odometry_publisher_ = node_handle_->create_publisher<nav_msgs::msg::Odometry>(
        FLAGS_odometry_output_topic, rmw_qos_profile_default);
  }
  if (!FLAGS_pose_output_topic.empty()) {
    pose3d_publisher_ = node_handle_->create_publisher<geometry_msgs::msg::PoseStamped>(
        FLAGS_pose_output_topic, rmw_qos_profile_default);
  }
  if (!FLAGS_pose_with_covariance_output_topic.empty()) {
    pose3d_cov_publisher_ = node_handle_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        FLAGS_pose_with_covariance_output_topic, rmw_qos_profile_default);
  }
}

static geometry_msgs::msg::Pose MakePose(const cartographer::transform::Rigid3<double>& map_to_publishing){
  auto &orientation = map_to_publishing.rotation();
  auto &position = map_to_publishing.translation();
  geometry_msgs::msg::Pose pose;
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();
  pose.orientation.w = orientation.w();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  return pose;
}

void cartographer_ros::AsiNode::PublishOtherOdometry(const std_msgs::msg::Header::_stamp_type &timestamp,
                                                     const cartographer_ros::MapBuilderBridge::TrajectoryState &trajectory_state,
                                                     const cartographer::transform::Rigid3d &tracking_to_local,
                                                     const cartographer::transform::Rigid3d &tracking_to_map) {
  Node::PublishOtherOdometry(timestamp, trajectory_state, tracking_to_local, tracking_to_map);

  if (!FLAGS_pose_output_topic.empty()) {
    geometry_msgs::msg::PoseStamped poseMsg;
    poseMsg.header.stamp = timestamp;
    poseMsg.header.frame_id = node_options_.map_frame;

    auto map_to_publishing = tracking_to_map * (*trajectory_state.published_to_tracking);
    poseMsg.pose = MakePose(map_to_publishing);
    pose3d_publisher_->publish(poseMsg);
  }
  if (!FLAGS_pose_with_covariance_output_topic.empty()) {
    geometry_msgs::msg::PoseWithCovarianceStamped poseMsg;
    poseMsg.header.stamp = timestamp;
    poseMsg.header.frame_id = node_options_.map_frame;

    auto map_to_publishing = tracking_to_map * (*trajectory_state.published_to_tracking);
    poseMsg.pose.pose = MakePose(map_to_publishing);

    // TODO: fix this once cartographer exposes some form of position confidence
    poseMsg.pose.covariance = {};
    poseMsg.pose.covariance[0] = poseMsg.pose.covariance[7] = poseMsg.pose.covariance[14] = poseMsg.pose.covariance[21] =
        poseMsg.pose.covariance[28] = poseMsg.pose.covariance[35] = 0.05;
    pose3d_cov_publisher_->publish(poseMsg);
  }

  if (!FLAGS_odometry_output_topic.empty() && last_pose_estimate_.time > cartographer::common::Time::min()) {
    auto delta_time = cartographer::common::ToSeconds(trajectory_state.pose_estimate.time - last_pose_estimate_.time);
    if (delta_time > 0.0) {
      nav_msgs::msg::Odometry poseMsg;
      poseMsg.header.stamp = timestamp;
      poseMsg.header.frame_id = node_options_.map_frame;
      poseMsg.child_frame_id = trajectory_state.trajectory_options.published_frame;

      auto map_to_publishing = tracking_to_map * (*trajectory_state.published_to_tracking);
      poseMsg.pose.pose = MakePose(map_to_publishing);
      auto delta_pose = trajectory_state.pose_estimate.pose.inverse() * last_pose_estimate_.pose;
      auto linear_velocity = delta_pose.translation() / delta_time;
      poseMsg.twist.twist.linear.x = linear_velocity.x();
      poseMsg.twist.twist.linear.y = linear_velocity.y();
      poseMsg.twist.twist.linear.z = linear_velocity.z();
      poseMsg.twist.twist.angular.x = delta_pose.rotation().x() * 2.0 / delta_time;
      poseMsg.twist.twist.angular.y = delta_pose.rotation().y() * 2.0 / delta_time;
      poseMsg.twist.twist.angular.z = delta_pose.rotation().z() * 2.0 / delta_time;

      poseMsg.pose.covariance = {};
      poseMsg.pose.covariance[0] = poseMsg.pose.covariance[7] = poseMsg.pose.covariance[14] = poseMsg.pose.covariance[21] =
        poseMsg.pose.covariance[28] = poseMsg.pose.covariance[35] = 0.05;
      poseMsg.twist.covariance = {};
      poseMsg.twist.covariance[0] = poseMsg.pose.covariance[7] = poseMsg.pose.covariance[14] = poseMsg.pose.covariance[21] =
      poseMsg.twist.covariance[28] = poseMsg.pose.covariance[35] = 0.1;

      odometry_publisher_->publish(poseMsg);
    }
  }

  if (!FLAGS_asi_acceleration_output_topic.empty()) {
    throw std::runtime_error("Acceleration output publisher is not implemented. Pull it from the IMU data instead.");
  }

  last_pose_estimate_ = trajectory_state.pose_estimate;
}

void cartographer_ros::AsiNode::LaunchSubscribers(const cartographer_ros::TrajectoryOptions &options,
                                                  const cartographer_ros_msgs::msg::SensorTopics &topics,
                                                  int trajectory_id) {
  Node::LaunchSubscribers(options, topics, trajectory_id);

  if (!FLAGS_pose_input_topic.empty()) {
    subscribers_.push_back(node_handle()->
        create_subscription<geometry_msgs::msg::PoseStamped>(
        FLAGS_pose_input_topic,
        [trajectory_id, this](geometry_msgs::msg::PoseStamped::ConstSharedPtr lean_msg) {

          if (!std::isnan(lean_msg->pose.position.x)) {
            auto msg = std::make_shared<nav_msgs::msg::Odometry>();
            msg->header = lean_msg->header;
            msg->child_frame_id = "base_link"; // same as options.publishing_frame?
            msg->pose.pose = lean_msg->pose;
            // msg->pose.covariance not used
            // msg->twist not used
            HandleOdometryMessage(trajectory_id, FLAGS_pose_input_topic, msg);
          }
        }, rmw_qos_profile_default));
  }

  if (!FLAGS_pose_with_covariance_input_topic.empty()) {
    subscribers_.push_back(node_handle()->
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        FLAGS_pose_with_covariance_input_topic,
        [trajectory_id, this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr lean_msg) {

          if (!std::isnan(lean_msg->pose.pose.position.x)) {
            auto msg = std::make_shared<nav_msgs::msg::Odometry>();
            msg->header = lean_msg->header;
            msg->child_frame_id = "base_link"; // same as options.publishing_frame?
            msg->pose = lean_msg->pose;
            // msg->twist not used
            HandleOdometryMessage(trajectory_id, FLAGS_pose_with_covariance_input_topic, msg);
          }
        }, rmw_qos_profile_default));
  }

  if (!FLAGS_twist_input_topic.empty()) {
    subscribers_.push_back(node_handle()->
        create_subscription<geometry_msgs::msg::TwistStamped>(
        FLAGS_twist_input_topic,
        [trajectory_id, this](geometry_msgs::msg::TwistStamped::ConstSharedPtr lean_msg) {
          // only the relative position of the odometry is used; we can start it with identity
          auto current_twist_odometry_time = framework::toSecondsAsDouble(lean_msg->header.stamp);
          auto delta_time = current_twist_odometry_time - last_twist_odometry_time_;
          last_twist_odometry_time_ = current_twist_odometry_time;
          if (delta_time > 0.0 && !std::isnan(lean_msg->twist.linear.x)) {

            //std::cout << "TWIST: " << lean_msg->twist.linear.x << ", " << lean_msg->twist.angular.z << ", " << delta_time << "\n";
            //std::cout << "PREV_TRANS_ROT: " << last_twist_odometry_.DebugString() << "\n";

            const auto linear_delta = Eigen::Vector3d(lean_msg->twist.linear.x * delta_time, 0.0, 0.0);
            const auto angular_delta = cartographer::transform::RollPitchYaw(0.0, 0.0, lean_msg->twist.angular.z * delta_time);

            const auto translation_estimate = last_twist_odometry_.rotation() * linear_delta + last_twist_odometry_.translation();
            const auto rotation_estimate = angular_delta * last_twist_odometry_.rotation();

            last_twist_odometry_ = {translation_estimate, rotation_estimate};

            auto msg = std::make_shared<nav_msgs::msg::Odometry>();
            msg->header = lean_msg->header;
            msg->child_frame_id = "base_link"; //options.published_frame?
            msg->pose.pose.position.x = translation_estimate.x();
            msg->pose.pose.position.y = translation_estimate.y();
            msg->pose.pose.position.z = translation_estimate.z();
            msg->pose.pose.orientation.w = rotation_estimate.w();
            msg->pose.pose.orientation.x = rotation_estimate.x();
            msg->pose.pose.orientation.y = rotation_estimate.y();
            msg->pose.pose.orientation.z = rotation_estimate.z();
            // msg->pose.covariance not used
            // msg->twist not used
            HandleOdometryMessage(trajectory_id, FLAGS_twist_input_topic, msg);
          }
        }, rmw_qos_profile_default));
  }


  if (!FLAGS_asi_clock_input_topic.empty()) {
    subscribers_.push_back(node_handle()->create_subscription<asiframework_msgs::msg::AsiTime>(
        FLAGS_asi_clock_input_topic,
        [this](asiframework_msgs::msg::AsiTime::ConstSharedPtr msg) {
          map_builder_bridge_.last_time = msg->time;
        }));
  }


std::unordered_set<string>
cartographer_ros::AsiNode::ComputeExpectedTopics(const cartographer_ros::TrajectoryOptions &options,
                                                 const cartographer_ros_msgs::msg::SensorTopics &topics) {
  auto ret = Node::ComputeExpectedTopics(options, topics);
  if (!FLAGS_pose_input_topic.empty())
    ret.insert(FLAGS_pose_input_topic);
  if (!FLAGS_pose_with_covariance_input_topic.empty())
    ret.insert(FLAGS_pose_with_covariance_input_topic);
  if (!FLAGS_twist_input_topic.empty())
    ret.insert(FLAGS_twist_input_topic);
  if (!FLAGS_asi_pose_input_topic.empty())
    ret.insert(FLAGS_asi_pose_input_topic);
  if (!FLAGS_asi_twistlean_input_topic.empty())
    ret.insert(FLAGS_asi_twistlean_input_topic);
  if (!FLAGS_asi_imulean_input_topic.empty())
    ret.insert(FLAGS_asi_imulean_input_topic);
  return ret;
}

cartographer_ros_msgs::msg::SensorTopics cartographer_ros::AsiNode::DefaultSensorTopics() {
  auto topics = Node::DefaultSensorTopics();
  if (FLAGS_disable_default_imu_topic)
    topics.imu_topic = "";
  return topics;
}
