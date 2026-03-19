/// \file
/// \brief ROS 2 node that detects circle landmarks from 2d laser scans
#include <cmath>
#include <set>
#include <string>
#include <vector>

#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nuslam/circle.hpp"
#include "turtlelib/geometry2d.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


using std::placeholders::_1;
using Cluster = std::vector<turtlelib::Point2D>;

class Landmark : public rclcpp::Node
{
public:
  Landmark()
  : Node("landmark")
  {
    this->declare_parameter("cluster_threshold", 0.1);
    this->declare_parameter("min_cluster", 2);
    this->declare_parameter("min_radius", 0.01);
    this->declare_parameter("max_radius", 0.2);
    this->declare_parameter("min_angle", M_PI / 2.0);
    this->declare_parameter("max_angle", 3.0 * M_PI / 4.0);
    this->declare_parameter("body_id", "red/base_footprint");

    cluster_threshold = this->get_parameter("cluster_threshold").as_double();
    min_cluster = this->get_parameter("min_cluster").as_int();
    min_radius = this->get_parameter("min_radius").as_double();
    max_radius = this->get_parameter("max_radius").as_double();
    min_angle = this->get_parameter("min_angle").as_double();
    max_angle = this->get_parameter("max_angle").as_double();
    body_id = this->get_parameter("body_id").as_string();

    laser_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "red/laser_scan",
      10,
      std::bind(&Landmark::laser_callback, this, _1)
    );

    landmark_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/landmark_pub",
      10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_pub;

  // Circle Parameters
  double cluster_threshold = 0.1;
  double min_cluster = 3;
  double min_radius = 0.0;
  double max_radius = 5.0;
  double min_angle = M_PI / 2.0;
  double max_angle = 3.0 * M_PI / 4.0;
  std::string body_id;

  void laser_callback(const sensor_msgs::msg::LaserScan & msg)
  {
    std::vector<turtlelib::Point2D> points{};
    for (size_t i = 0; i < msg.ranges.size(); i++) {
      auto r = msg.ranges[i];
      if (r < msg.range_min || r > msg.range_max) {
        continue;
      }
      auto angle = msg.angle_min + i * msg.angle_increment;
      points.push_back(turtlelib::Point2D{r * std::cos(angle), r * std::sin(angle)});
    }

    RCLCPP_INFO(get_logger(), "scan: %zu raw ranges, %zu valid points",
      msg.ranges.size(), points.size());

    if (points.size() == 0) {
      RCLCPP_WARN(get_logger(), "no valid points after range filter, skipping");
      return;
    }

    auto clusters = nuslam::cluster_points(points, cluster_threshold);
    RCLCPP_INFO(get_logger(), "clustered into %zu clusters", clusters.size());
    
    std::vector<nuslam::Circle> detected{};
    for (const auto & cluster : clusters) {
      if (!nuslam::is_circle(cluster, min_angle, max_angle)) {
        continue;
      }
      auto circle = nuslam::fit_circle(cluster);
      RCLCPP_INFO(get_logger(), "  cluster size %zu -> circle (%.3f, %.3f) r=%.3f",
        cluster.size(), circle.x, circle.y, circle.r);
      if (circle.r < min_radius || circle.r > max_radius) {
        RCLCPP_INFO(get_logger(), "    rejected by radius filter (min=%.3f max=%.3f)",
          min_radius, max_radius);
        continue;
      }
      detected.push_back(circle);
    }

    RCLCPP_INFO(get_logger(), "%zu circles passed all filters", detected.size());

    visualization_msgs::msg::MarkerArray landmarks;
    for (std::size_t i = 0; i < detected.size(); i++) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = msg.header.stamp;
      m.header.frame_id = body_id;
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = detected[i].x;
      m.pose.position.y = detected[i].y;
      m.pose.position.z = 0.125;
      m.pose.orientation.w = 1.0;
      m.scale.x = detected[i].r * 2.0;
      m.scale.y = detected[i].r * 2.0;
      m.scale.z = 0.25;
      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 1.0;
      m.color.a = 1.0;
      landmarks.markers.push_back(m);
    }

    landmark_pub->publish(landmarks);
  }

};

/// \brief Entry point for the landmark node.
/// \return 0 on clean shutdown.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmark>());
  rclcpp::shutdown();
  return 0;
}
