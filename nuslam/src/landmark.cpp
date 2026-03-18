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
    this->declare_parameter("min_cluster", 3);
    this->declare_parameter("min_radius", 0.01);
    this->declare_parameter("max_radius", 0.2);
    this->declare_paramter("body_id", "green/base_footprint");

    cluster_threshold = this->get_parameter("cluster_threshold").as_double();
    min_cluster = this->get_parameter("min_cluster").as_int();
    min_radius = this->get_parameter("min_radius").as_double();
    max_radius = this->get_parameter("max_radius").as_double();
    body_id = this->get_parameter("body_id").as_double();

    laser_scan = this->create_subscriber<sensor_msgs::msg::LaserScan>(
      "laser_scan",
      10,
      std::bind(&nuslam::laser_callback, this, _1)
    );

    landmark_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/landmark_pub", 10);
  }
private:
  rclcpp::Subscriber<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_pub;

  // Circle Parameters
  auto cluster_threshold = 0.1;
  auto min_cluster = 3;
  auto min_radius = 0.0;
  auto max_radius = 5.0;
  std::string body_id;

  void laser_callback(sensor_msgs::msg::LaserScan & msg)
  {
    std::vector<turtlelib::Point2D> points = [];
    for (size_t i = 0; i < msg.ranges.size(); i++) 
    {
      auto r = msg.ranges.size();
      if (r < msg.range_min || r > msg.range_max)
      {
        continue;
      }
      auto angle = msg.angle_min + i * msg.angle_increment;
      points.append(Point2D{r * std::cos(angle), r * std::sin(angle)})
    }
    if (points.size() == 0) 
    {
      return;
    }
    auto clusters = nuslam::cluster_points(points, cluster_threshold);
    
    std::vector<nuslam::Circle> detected = [];
    for (std::vector<Cluster> cluster : clusters)
    {
      if (!nuslam::is_circle(cluster)) 
      {
        continue;
      }
      auto circle = nuslam::fit_circle(cluster);
      if (circle.r < min_radius || circle.r > max_radius)
      {
        continue;
      }
      detected.push_back(circle);
    }
    visualization_msgs::msg::MarkerArray landmarks;
    for (std::size_t i = 0; i < detected.size(); i++)
    {
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

  
}