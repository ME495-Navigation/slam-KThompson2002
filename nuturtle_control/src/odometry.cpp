/// \file
/// \brief ROS 2 node that computes and publishes differential-drive odometry.
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
#include "nuturtle_control_interfaces/srv/initial_pose.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

/// \brief Node that estimates the robot pose from wheel encoder positions.
///
/// The node reads wheel joint names and kinematic parameters from ROS parameters,
/// then uses forward kinematics to update the pose and publish odom + TF.
class Odometry : public rclcpp::Node
{
public:
  /// \brief Construct the Odometry node.
  Odometry()
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

    diff = std::make_unique<turtlelib::DiffDrive>(track_width, wheel_radius);
    diff->setPose(turtlelib::Transform2D(turtlelib::Vector2D{0.0, 0.0}, 0.0));
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    joint_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&Odometry::joint_callback, this, std::placeholders::_1)
    );

    init_pose = this->create_service<nuturtle_control_interfaces::srv::InitialPose>(
      "initial_pose",
      std::bind(&Odometry::initial_pose_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );

    nav_path = this->create_publisher<nav_msgs::msg::Path>(
      "nav_path",
      10
    );
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  rclcpp::Service<nuturtle_control_interfaces::srv::InitialPose>::SharedPtr init_pose;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nav_path;

  // Parameters
  std::string body_id;
  std::string odom_id;
  std::string wheel_left;
  std::string wheel_right;
  double track_width;
  double wheel_radius;
  std::unique_ptr<turtlelib::DiffDrive> diff;
  turtlelib::Wheel last_wheels;

  std::vector<geometry_msgs::msgs::PoseStamped> poses;

  /// \brief JointState subscriber callback.
  ///
  /// Extracts the left/right wheel positions from the incoming JointState message,
  /// updates the DiffDrive pose via forward kinematics, then publishes odom + TF.
  /// \param js [in] JointState message containing wheel positions.
  void joint_callback(const sensor_msgs::msg::JointState & js)
  {
    turtlelib::Wheel wheels;
    for (size_t i = 0; i < js.name.size(); ++i) {
      if (js.name[i] == wheel_left) {
        wheels.left = js.position[i];
      } else if (js.name[i] == wheel_right) {
        wheels.right = js.position[i];
      }
    }
    last_wheels = wheels;

    turtlelib::Twist2D Vb = diff->forwardKinematics(wheels);

    publish_odom_and_tf(js.header.stamp, Vb);
  }

  /// \brief Service callback to reset the odometry pose.
  ///
  /// Sets the internal DiffDrive pose to the requested (x, y, theta).
  /// \param req [in] Requested initial pose.
  /// \param res [out] Response indicating success.
  void initial_pose_callback(
    const std::shared_ptr<nuturtle_control_interfaces::srv::InitialPose::Request> req,
    std::shared_ptr<nuturtle_control_interfaces::srv::InitialPose::Response> res)
  {
    turtlelib::Transform2D T0(turtlelib::Vector2D{req->x, req->y}, req->theta);
    diff->setPose(T0);

    res->success = true;
  }

  /// \brief Publish odometry and broadcast the odom->base transform.
  ///
  /// \param stamp [in] Timestamp to use for the messages.
  /// \param Vb [in] Body-frame twist computed from wheel motion.
  void publish_odom_and_tf(
    const builtin_interfaces::msg::Time & stamp,
    const turtlelib::Twist2D & Vb)
  {
    const turtlelib::Transform2D T = diff->pose();
    const turtlelib::Vector2D p = T.translation();
    const double yaw = T.rotation();

    // Create Odometry Message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_id;
    odom.child_frame_id = body_id;

    odom.pose.pose.position.x = p.x;
    odom.pose.pose.position.y = p.y;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = Vb.x;
    odom.twist.twist.linear.y = Vb.y;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = Vb.omega;

    odom_pub->publish(odom);

    auto path = nav_msgs::msg::Path();
    path.header.stamp = this->get_clock()->now();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = get_clock()->now();
    pose.header.frame_id = "nusim/world";
    pose.pose.position.x = p.x;
    pose.pose.position.y = p.y;

    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    path.header.frame_id = "nusim/world";
    path.poses.push_back(pose);
    nav_path->publish(path);

    // Create Transform message
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_id;
    tf.child_frame_id = body_id;

    tf.transform.translation.x = p.x;
    tf.transform.translation.y = p.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    tf_broadcaster->sendTransform(tf);
  }
};

/// \brief Entry point for the odometry node.
/// \return 0 on clean shutdown.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
