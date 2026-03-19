/// \file
/// \brief ROS 2 node that performs EKF-SLAM for a differential-drive robot.
#include <algorithm>
#include <cmath>
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
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "turtlelib/diff_drive.hpp"
#include "nuslam/ekf.hpp"

using std::placeholders::_1;

class Slam_data : public rclcpp::Node
{
public:
  Slam_data()
  : Node("slam_data")
  {
    declare_parameter("body_id", "green/base_footprint");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", "");
    declare_parameter("wheel_right", "");
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.16);
    declare_parameter("process_noise", 1e-3);
    declare_parameter("sensing_noise", 1e-2);
    declare_parameter("threshold", 15.0);
    declare_parameter("min_observations", 3);
    declare_parameter("provisional_match_dist", 0.3);
    declare_parameter("max_provisional_age", 5);

    body_id_ = get_parameter("body_id").as_string();
    odom_id_ = get_parameter("odom_id").as_string();
    wheel_left_ = get_parameter("wheel_left").as_string();
    wheel_right_ = get_parameter("wheel_right").as_string();
    const double wheel_radius = get_parameter("wheel_radius").as_double();
    const double track_width = get_parameter("track_width").as_double();
    const double process_noise = get_parameter("process_noise").as_double();
    const double sensing_noise = get_parameter("sensing_noise").as_double();
    threshold = get_parameter("threshold").as_double();
    min_observations_ = get_parameter("min_observations").as_int();
    provisional_match_dist_ = get_parameter("provisional_match_dist").as_double();
    max_provisional_age_ = get_parameter("max_provisional_age").as_int();

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
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&Slam_data::joint_callback, this, _1)
    );

    sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "landmark_pub", 10,
      std::bind(&Slam_data::sensor_callback, this, _1)
    );
  }

private:
  struct Provisional
  {
    double x, y;  // map-frame position estimate (running average)
    int count;    // number of times matched
    int age;      // scan callbacks since last matched
  };

  // Parameters
  std::string body_id_;
  std::string odom_id_;
  std::string wheel_left_;
  std::string wheel_right_;
  double threshold;
  int min_observations_;
  double provisional_match_dist_;
  int max_provisional_age_;

  // State
  std::unique_ptr<turtlelib::DiffDrive> diff_;
  std::unique_ptr<nuslam::EKF> ekf_;
  nav_msgs::msg::Path slam_path_;
  std::vector<Provisional> provisionals_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_pub_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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
    // Age all provisionals at the start of each scan
    for (auto & p : provisionals_) {
      p.age++;
    }
    double radius = 0.0;

    for (const auto & marker : msg.markers) {
      if (marker.action == visualization_msgs::msg::Marker::DELETE) {
        continue;
      }

      // Inline Citation - [7]
      // Markers arrive in the robot body frame (red/base_footprint).
      const double mx = marker.pose.position.x;
      const double my = marker.pose.position.y;
      radius = marker.scale.x;
      const double r = std::sqrt(mx * mx + my * my);
      const double phi = turtlelib::normalize_angle(std::atan2(my, mx));
      
      // End Inline Citation

      // 1. Try Mahalanobis association with confirmed EKF landmarks
      const int ekf_id = ekf_->try_associate(r, phi, threshold);
      if (ekf_id >= 0) {
        ekf_->update(ekf_id, r, phi);
        continue;
      }

      // 2. No confirmed match — project measurement to map frame
      const turtlelib::Transform2D ekf_pose = ekf_->pose();
      const double theta = ekf_pose.rotation();
      const turtlelib::Vector2D t = ekf_pose.translation();
      const double map_x = t.x + r * std::cos(phi + theta);
      const double map_y = t.y + r * std::sin(phi + theta);

      // 3. Find closest provisional landmark
      int best_prov = -1;
      double best_dist = provisional_match_dist_;
      for (int k = 0; k < static_cast<int>(provisionals_.size()); k++) {
        const double d = std::hypot(map_x - provisionals_[k].x, map_y - provisionals_[k].y);
        if (d < best_dist) {
          best_dist = d;
          best_prov = k;
        }
      }

      if (best_prov >= 0) {
        auto & prov = provisionals_.at(best_prov);
        prov.count++;
        prov.age = 0;
        // Running average of map-frame position
        prov.x = (prov.x * (prov.count - 1) + map_x) / prov.count;
        prov.y = (prov.y * (prov.count - 1) + map_y) / prov.count;

        if (prov.count >= min_observations_) {
          ekf_->update_with_association(r, phi, threshold);
          // const int new_id = ekf_->num_landmarks();
          // ekf_->initialize_landmark_at(new_id, prov.x, prov.y);
          // ekf_->update(new_id, r, phi);
          provisionals_.erase(provisionals_.begin() + best_prov);
        }
      } else {
        // 4. New provisional
        provisionals_.push_back({map_x, map_y, 1, 0});
      }
    }

    // Remove stale provisionals
    provisionals_.erase(
      std::remove_if(
        provisionals_.begin(), provisionals_.end(),
        [this](const Provisional & p) {return p.age > max_provisional_age_;}),
      provisionals_.end());

    // Re-broadcast map->odom
    if (!msg.markers.empty()) {
      publish_odom_and_tf(msg.markers.front().header.stamp);
    }

    // Publish confirmed EKF landmarks
    visualization_msgs::msg::MarkerArray map_markers;
    for (int id = 0; id < static_cast<int>(ekf_->num_landmarks()); ++id) {
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
      m.scale.x = radius;
      m.scale.y = radius;
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
  rclcpp::spin(std::make_shared<Slam_data>());
  rclcpp::shutdown();
  return 0;
}
