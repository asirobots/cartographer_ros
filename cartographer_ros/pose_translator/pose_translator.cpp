#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "localization_msgs/msg/pose_with_covariance_lean_relative_stamped.hpp"

int main(int argc, char** argv) {
  ::rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("occupancy_grid_node");

  auto publisher = node->create_publisher<localization_msgs::msg::PoseWithCovarianceLeanRelativeStamped>("Vehicle_Pose");
  auto subscriber = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("fusor_base_link_pose",
    [&publisher](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){

      localization_msgs::msg::PoseWithCovarianceLeanRelativeStamped new_msg;
      new_msg.header = msg->header;
      new_msg.child_frame_id = "base_link";
      new_msg.pose = msg->pose.pose;

      new_msg.position_covariance[0] = msg->pose.covariance[0];
      new_msg.position_covariance[1] = msg->pose.covariance[1];
      new_msg.position_covariance[2] = msg->pose.covariance[2];
      new_msg.position_covariance[3] = msg->pose.covariance[7];
      new_msg.position_covariance[4] = msg->pose.covariance[8];
      new_msg.position_covariance[5] = msg->pose.covariance[14];

      new_msg.orientation_covariance[0] = msg->pose.covariance[21];
      new_msg.orientation_covariance[1] = msg->pose.covariance[22];
      new_msg.orientation_covariance[2] = msg->pose.covariance[23];
      new_msg.orientation_covariance[3] = msg->pose.covariance[28];
      new_msg.orientation_covariance[4] = msg->pose.covariance[29];
      new_msg.orientation_covariance[5] = msg->pose.covariance[35];

      publisher->publish(new_msg);
    });

  ::rclcpp::spin(node);
  ::rclcpp::shutdown();
}

