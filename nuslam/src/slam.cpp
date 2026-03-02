/// \file
/// \brief ROS 2 node that performs EKF-SLAM for a differential-drive robot.
#include <cmath>
#include <set>
#include <string>
#include <vector>

#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "turtlelib/diff_drive.hpp"
#include "nuslam/ekf.hpp"

using std::placeholders::_1;

class Slam : public rclcpp::Node
{
public:
  Slam()
  : Node("slam")
  {
    declare_parameter("body_id", "blue/base_footprint");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", "");
    declare_parameter("wheel_right", "");
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.16);
    declare_parameter("process_noise", 1e-3);
    declare_parameter("sensing_noise", 1e-2);

    body_id_ = get_parameter("body_id").as_string();
    odom_id_ = get_parameter("odom_id").as_string();
    wheel_left_ = get_parameter("wheel_left").as_string();
    wheel_right_ = get_parameter("wheel_right").as_string();
    const double wheel_radius = get_parameter("wheel_radius").as_double();
    const double track_width = get_parameter("track_width").as_double();
    const double process_noise = get_parameter("process_noise").as_double();
    const double sensing_noise = get_parameter("sensing_noise").as_double();

    if (wheel_left_.empty() || wheel_right_.empty()) {
      RCLCPP_ERROR(get_logger(), "wheel_left and wheel_right parameters must be set.");
      rclcpp::shutdown();
      return;
    }

    diff_ = std::make_unique<turtlelib::DiffDrive>(track_width, wheel_radius);
    ekf_ = std::make_unique<nuslam::EKF>(
      arma::eye(3, 3) * process_noise,
      arma::eye(2, 2) * sensing_noise
    );

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);
    map_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/map", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&Slam::joint_callback, this, _1)
    );

    sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "fake_sensor", 10,
      std::bind(&Slam::sensor_callback, this, _1)
    );
  }

private:
  // Parameters
  std::string body_id_;
  std::string odom_id_;
  std::string wheel_left_;
  std::string wheel_right_;

  // State
  std::unique_ptr<turtlelib::DiffDrive> diff_;
  std::unique_ptr<nuslam::EKF> ekf_;
  nav_msgs::msg::Path slam_path_;
  std::set<int> seen_ids_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_pub_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sensor_sub_;

  /// \brief JointState callback: propagates the EKF prediction step and publishes odometry.
  ///
  /// Extracts wheel positions from the message, computes the body-frame twist via
  /// forward kinematics, feeds it to the EKF predict step, then publishes the
  /// SLAM odometry estimate and broadcasts the map->odom transform.
  void joint_callback(const sensor_msgs::msg::JointState & js)
  {
    // Extract wheel positions by joint name
    turtlelib::Wheel wheels;
    for (std::size_t i = 0; i < js.name.size(); ++i) {
      if (js.name[i] == wheel_left_) {
        wheels.left = js.position[i];
      } else if (js.name[i] == wheel_right_) {
        wheels.right = js.position[i];
      }
    }

    // Forward kinematics: updates diff_ internal pose and returns body-frame twist
    const turtlelib::Twist2D twist = diff_->forwardKinematics(wheels);

    // EKF prediction step
    ekf_->predict(twist);

    // Publish SLAM odometry and broadcast map->odom TF
    publish_odom_and_tf(js.header.stamp);
  }

  void sensor_callback(const visualization_msgs::msg::MarkerArray & msg)
  {
    // --- EKF update step ---
    for (const auto & marker : msg.markers) {
      if (marker.action == visualization_msgs::msg::Marker::DELETE) {
        continue;
      }

      const turtlelib::Transform2D robot_pose = ekf_->pose();
      const double dx = marker.pose.position.x - robot_pose.translation().x;
      const double dy = marker.pose.position.y - robot_pose.translation().y;
      const double r = std::sqrt(dx * dx + dy * dy);
      const double phi = turtlelib::normalize_angle(std::atan2(dy, dx) - robot_pose.rotation());

      ekf_->update(marker.id, r, phi);
      seen_ids_.insert(marker.id);
    }

    // Re-broadcast map->odom now that the EKF pose has been corrected.
    // Without this, the TF stays at the stale post-predict value until the
    // next joint state arrives, causing the green robot to visibly snap.
    if (!msg.markers.empty()) {
      publish_odom_and_tf(msg.markers.front().header.stamp);
    }

    // --- Publish estimated landmark positions ---
    visualization_msgs::msg::MarkerArray map_markers;
    for (const int id : seen_ids_) {
      const turtlelib::Vector2D pos = ekf_->landmark(id);

      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = this->now();
      m.id = id;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = pos.x;
      m.pose.position.y = pos.y;
      m.pose.position.z = 0.125;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.076;
      m.scale.y = 0.076;
      m.scale.z = 0.25;
      m.color.a = 1.0;
      m.color.r = 0.0;
      m.color.g = 1.0;
      m.color.b = 0.0;

      map_markers.markers.push_back(m);
    }
    map_pub_->publish(map_markers);
  }

  /// \brief Publish SLAM odometry and broadcast the map->odom correction transform.
  ///
  /// The EKF pose is T_map_base (SLAM estimate of robot in map frame).
  /// The DiffDrive pose is T_odom_base (dead-reckoning in odom frame).
  /// The map->odom correction is: T_map_odom = T_map_base * T_odom_base.inv()
  void publish_odom_and_tf(const builtin_interfaces::msg::Time & stamp)
  {
    const turtlelib::Transform2D T_map_base = ekf_->pose();
    const turtlelib::Transform2D T_odom_base = diff_->pose();
    const turtlelib::Transform2D T_map_odom = T_map_base * T_odom_base.inv();

    const turtlelib::Vector2D p = T_map_base.translation();
    const double yaw = T_map_base.rotation();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    // SLAM odometry (map frame)
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = body_id_;
    odom.pose.pose.position.x = p.x;
    odom.pose.pose.position.y = p.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom_pub_->publish(odom);

    // Accumulate and publish path
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = stamp;
    ps.header.frame_id = "map";
    ps.pose = odom.pose.pose;
    slam_path_.header.stamp = stamp;
    slam_path_.header.frame_id = "map";
    slam_path_.poses.push_back(ps);
    path_pub_->publish(slam_path_);

    // Broadcast map -> odom
    const turtlelib::Vector2D t_mo = T_map_odom.translation();
    const double yaw_mo = T_map_odom.rotation();
    tf2::Quaternion q_mo;
    q_mo.setRPY(0.0, 0.0, yaw_mo);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "map";
    tf.child_frame_id = odom_id_;
    tf.transform.translation.x = t_mo.x;
    tf.transform.translation.y = t_mo.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = q_mo.x();
    tf.transform.rotation.y = q_mo.y();
    tf.transform.rotation.z = q_mo.z();
    tf.transform.rotation.w = q_mo.w();
    tf_broadcaster_->sendTransform(tf);
  }
};

/// \brief Entry point for the slam node.
/// \return 0 on clean shutdown.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}
