//
// Created by brannon on 8/7/17.
//

#include "asi_node.h"
#include "time_conversion.h"
#include "asiframework_msgs/msg/asi_time.hpp"
#include "localization_msgs/msg/body_velocity_stamped.hpp"
#include "localization_msgs/msg/imu_lean_stamped.hpp"
#include "framework/os/Time.hpp"

DEFINE_string(asi_pose2d_output_topic, "", "Output topic for localization_msgs::Pose2DWithCovarianceRelativeStamped");

DEFINE_string(asi_velocity_output_topic, "",
              "Output topic for localization_msgs::BodyVelocityWithCovarianceLeanStamped");

DEFINE_string(asi_acceleration_output_topic, "",
              "Output topic for localization_msgs::BodyAccelWithCovarianceLeanStamped");

DEFINE_string(asi_pose2d_input_topic, "", "Input topic for localization_msgs::Pose2DWithCovarianceRelativeStamped");

DEFINE_string(asi_imulean_input_topic, "", "Input topic for localization_msgs::ImuLeanStamped");

DEFINE_string(asi_twistlean_input_topic, "", "Input topic for localization_msgs::BodyVelocityStamped");

DEFINE_string(asi_clock_input_topic, "", "Input topic for asiframework::AsiTime");

DEFINE_bool(disable_default_imu_topic, true, "By default, the default IMU topic is disabled");

cartographer_ros::AsiNode::AsiNode(const cartographer_ros::NodeOptions &node_options, tf2_ros::Buffer *tf_buffer)
    : Node(node_options, tf_buffer) {

  if (!FLAGS_asi_pose2d_output_topic.empty()) {
    lean_pose_publisher_ = node_handle_->create_publisher<localization_msgs::msg::Pose2DWithCovarianceRelativeStamped>(
        FLAGS_asi_pose2d_output_topic, rmw_qos_profile_default);
  }
  if (!FLAGS_asi_velocity_output_topic.empty()) {
    velocity_publisher_ = node_handle_->create_publisher<localization_msgs::msg::BodyVelocityWithCovarianceLeanStamped>(
        FLAGS_asi_velocity_output_topic, rmw_qos_profile_default);
  }
  if (!FLAGS_asi_acceleration_output_topic.empty()) {
    acceleration_publisher_ = node_handle_->create_publisher<localization_msgs::msg::BodyAccelWithCovarianceLeanStamped>(
        FLAGS_asi_acceleration_output_topic, rmw_qos_profile_default);
  }
}

void cartographer_ros::AsiNode::PublishOtherOdometry(const std_msgs::msg::Header::_stamp_type &timestamp,
                                                     const cartographer_ros::MapBuilderBridge::TrajectoryState &trajectory_state,
                                                     const cartographer::transform::Rigid3d &tracking_to_local) {
  Node::PublishOtherOdometry(timestamp, trajectory_state, tracking_to_local);

  if (!FLAGS_asi_pose2d_output_topic.empty()) {

    localization_msgs::msg::Pose2DWithCovarianceRelativeStamped poseMsg;
    poseMsg.header.stamp = timestamp;
    poseMsg.header.frame_id = node_options_.map_frame;
    poseMsg.child_frame_id = trajectory_state.trajectory_options.odom_frame;

    auto &orientation = tracking_to_local.rotation();
    auto &position = tracking_to_local.translation();
    poseMsg.pose2d.x = position.x();
    poseMsg.pose2d.y = position.y();
    // float yaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)); for q0 = w, q1 = x
    auto q0 = orientation.w();
    auto q1 = orientation.x();
    auto q2 = orientation.y();
    auto q3 = orientation.z();
    poseMsg.pose2d.theta = std::atan2(2.0 * (q3 * q0 + q1 * q2), -1.0 + 2.0 * (q0 * q0 + q1 * q1));

    if (trajectory_state.trajectory_options.trajectory_builder_options.pure_localization()) {
      poseMsg.position_covariance = {0.02, 0.0, 0.02};
      poseMsg.yaw_covariance = 0.04;
    } else {
      poseMsg.position_covariance = {0.08, 0.0, 0.08};
      poseMsg.yaw_covariance = 0.16;
    }

    lean_pose_publisher_->publish(poseMsg);
  }

  if (!FLAGS_asi_velocity_output_topic.empty() && last_pose_estimate_.time > cartographer::common::Time::min()) {
    auto delta_pose = trajectory_state.pose_estimate.pose.inverse() * last_pose_estimate_.pose;
    auto delta_time = cartographer::common::ToSeconds(trajectory_state.pose_estimate.time - last_pose_estimate_.time);
    if (delta_time > 0.0) {
      auto velocityMsg = std::make_shared<localization_msgs::msg::BodyVelocityWithCovarianceLeanStamped>();
      velocityMsg->header.stamp = timestamp;
      velocityMsg->header.frame_id = trajectory_state.trajectory_options.tracking_frame;
      auto linear_velocity = delta_pose.translation() / delta_time;
      velocityMsg->twist.linear.x = linear_velocity.x();
      velocityMsg->twist.linear.y = linear_velocity.y();
      velocityMsg->twist.linear.z = linear_velocity.z();
      velocityMsg->twist.angular.x = delta_pose.rotation().x() * 2.0 / delta_time;
      velocityMsg->twist.angular.y = delta_pose.rotation().y() * 2.0 / delta_time;
      velocityMsg->twist.angular.z = delta_pose.rotation().z() * 2.0 / delta_time;

      velocity_publisher_->publish(velocityMsg);
    }
  }

  if (!FLAGS_asi_acceleration_output_topic.empty()) {
    throw std::runtime_error("Acceleration output publisher is not implemented. Pull it from the IMU data instead.");
  }

  last_pose_estimate_ = trajectory_state.pose_estimate;
}

double DropNaN(double value) {
  if (std::isnan(value)) return 0.0;
  return value;
}

void cartographer_ros::AsiNode::LaunchSubscribers(const cartographer_ros::TrajectoryOptions &options,
                                                  const cartographer_ros_msgs::msg::SensorTopics &topics,
                                                  int trajectory_id) {
  Node::LaunchSubscribers(options, topics, trajectory_id);

  if (!FLAGS_asi_clock_input_topic.empty()) {
    asi_clock_subscriber_ = node_handle()->create_subscription<asiframework_msgs::msg::AsiTime>(
        FLAGS_asi_clock_input_topic,
        [this](asiframework_msgs::msg::AsiTime::ConstSharedPtr msg) {
          map_builder_bridge_.last_time = msg->time;
        });
  }

  if (!FLAGS_asi_pose2d_input_topic.empty()) {
    lean_odometry_subscriber_ = node_handle()->
        create_subscription<localization_msgs::msg::Pose2DWithCovarianceRelativeStamped>(
        FLAGS_asi_pose2d_input_topic,
        [trajectory_id, this](localization_msgs::msg::Pose2DWithCovarianceRelativeStamped::ConstSharedPtr lean_msg) {

          if (!std::isnan(lean_msg->pose2d.x)) {
            tf2::Quaternion quaternion;
            quaternion.setRPY(0.0, 0.0, lean_msg->pose2d.theta);

            auto msg = std::make_shared<nav_msgs::msg::Odometry>();
            msg->header = lean_msg->header;
            msg->child_frame_id = lean_msg->child_frame_id;
            msg->pose.pose.position.x = lean_msg->pose2d.x;
            msg->pose.pose.position.y = lean_msg->pose2d.y;
            msg->pose.pose.position.z = 0.0;
            msg->pose.pose.orientation.x = quaternion.getX();
            msg->pose.pose.orientation.y = quaternion.getY();
            msg->pose.pose.orientation.z = quaternion.getZ();
            msg->pose.pose.orientation.w = quaternion.getW();
            // msg->pose.covariance not used
            // msg->twist not used
            HandleOdometryMessage(trajectory_id, FLAGS_asi_pose2d_input_topic, msg);
          }
        }, rmw_qos_profile_default);
  }

  if (!FLAGS_asi_twistlean_input_topic.empty()) {
    lean_twist_subscriber_ = node_handle()->
        create_subscription<localization_msgs::msg::BodyVelocityStamped>(
        FLAGS_asi_twistlean_input_topic,
        [trajectory_id, this](localization_msgs::msg::BodyVelocityStamped::ConstSharedPtr lean_msg) {
          // only the relative position of the odometry is used; we can start it with identity

          auto current_twist_odometry_time = framework::toSecondsAsDouble(lean_msg->header.stamp);
          auto delta_time = current_twist_odometry_time - last_twist_odometry_time_;
          last_twist_odometry_time_ = current_twist_odometry_time;
          if (delta_time > 0.0 && !std::isnan(lean_msg->twist.linear.x)) {

            auto linear_delta = Eigen::Vector3d(DropNaN(lean_msg->twist.linear.x) * delta_time,
                                                   DropNaN(lean_msg->twist.linear.y) * delta_time,
                                                   DropNaN(lean_msg->twist.linear.z) * delta_time);
            auto angular_delta = Eigen::Quaterniond(0.0,
                                                    DropNaN(lean_msg->twist.angular.x) * delta_time * 0.5, // the .5 is a quaternion thing
                                                    DropNaN(lean_msg->twist.angular.y) * delta_time * 0.5,
                                                    DropNaN(lean_msg->twist.angular.z) * delta_time * 0.5);

            auto rotation_estimate = angular_delta * last_twist_odometry_.rotation();
            // sufficient if velocity is for the control point itself (otherwise we have to add wx(wxT)):
            auto translation_estimate = linear_delta + last_twist_odometry_.translation();

            auto msg = std::make_shared<nav_msgs::msg::Odometry>();
            msg->header = lean_msg->header;
            msg->child_frame_id = lean_msg->header.frame_id;
            msg->pose.pose.position.x = translation_estimate.x();
            msg->pose.pose.position.y = translation_estimate.y();
            msg->pose.pose.position.z = translation_estimate.z();
            msg->pose.pose.orientation.w = rotation_estimate.w();
            msg->pose.pose.orientation.x = rotation_estimate.x();
            msg->pose.pose.orientation.y = rotation_estimate.y();
            msg->pose.pose.orientation.z = rotation_estimate.z();
            // msg->pose.covariance not used
            // msg->twist not used

            last_twist_odometry_ = {translation_estimate, rotation_estimate};

            HandleOdometryMessage(trajectory_id, FLAGS_asi_twistlean_input_topic, msg);
          }
        }, rmw_qos_profile_default);

  }

  if (!FLAGS_asi_imulean_input_topic.empty()) {
    lean_imu_subscriber_ = node_handle()->
        create_subscription<localization_msgs::msg::ImuLeanStamped>(
        FLAGS_asi_imulean_input_topic,
        [trajectory_id, this](localization_msgs::msg::ImuLeanStamped::ConstSharedPtr lean_msg) {

          auto msg = std::make_shared<sensor_msgs::msg::Imu>();
          msg->header = lean_msg->header;
          msg->angular_velocity = lean_msg->imu.angular_rates;
          msg->linear_acceleration = lean_msg->imu.linear_acceleration;
          // msg->*_covariance not used
          // msg->orientation not used

          HandleImuMessage(trajectory_id, FLAGS_asi_imulean_input_topic, msg);
        }, rmw_qos_profile_default);
  }
}


std::unordered_set<string>
cartographer_ros::AsiNode::ComputeExpectedTopics(const cartographer_ros::TrajectoryOptions &options,
                                                 const cartographer_ros_msgs::msg::SensorTopics &topics) {
  auto ret = Node::ComputeExpectedTopics(options, topics);
  if (!FLAGS_asi_pose2d_input_topic.empty())
    ret.insert(FLAGS_asi_pose2d_input_topic);
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
