#include <algorithm>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "turtlelib/diff_drive.hpp"

class Odometry : public rclcpp::Node
{
public:
  OdometryNode()
  : Node("odometry")
  {
    this->declare_parameter("body_id", "base_footprint");
    this->declare_parameter("odom_id", "odom");
    this->declare_parameter("wheel_left", "");
    this->declare_parameter("wheel_right", "");

    body_id = this->get_parameter("body_id").as_string();
    odom_id = this->get_parameter("odom_id").as_string();
    wheel_left = this->get_parameter("wheel_left").as_string();
    wheel_right = this->get_parameter("wheel_right").as_string();

    if (wheel_left.empty() || wheel_right.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "wheel_left and wheel_right parameters must be set. Exiting.");
      rclcpp::shutdown();
      return;
    }

    this->declare_parameter<double>("track_width", 0.16);
    this->declare_parameter<double>("wheel_radius", 0.033);
    track_width = this->get_parameter("track_width").as_double();
    wheel_radius = this->get_parameter("wheel_radius").as_double();

    diff = turtlelib::DiffDrive(track, radius);
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    joint_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&OdometryNode::joint_callback, this, std::placeholders::_1)
    );

  }
private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // Parameters
  std::string body_id;
  std::string odom_id;
  std::string wheel_left;
  std::string wheel_right;
  double track_width;
  double wheel_radius;

  turtlelib::DiffDrive diff;

  void joint_callback(const sensor_msgs::msg::JointState & js)
  {
    
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}