// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class nusimulator : public rclcpp::Node
{
public:
  nusimulator()
  : Node("nusimulator")
  {
    this->declare_parameter("rate", 100);
    this->declare_parameter("x0", 0.0);
    this->declare_parameter("y0", 0.0);
    this->declare_parameter("theta0", 0.0);
    this->declare_parameter("arena_x_length", 7.0);
    this->declare_parameter("arena_y_length", 7.0);
    this->declare_parameter<std::vector<double>>("obstacles.x", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("obstacles.y", std::vector<double>{});
    this->declare_parameter<double>("obstacles.r", 0.0);

    const int rate = this->get_parameter("rate").as_int();
    x0_ = this->get_parameter("x0").as_double();
    y0_ = this->get_parameter("y0").as_double();
    theta0_ = this->get_parameter("theta0").as_double();
    arena_x_length = this->get_parameter("arena_x_length").as_double();
    arena_y_length = this->get_parameter("arena_y_length").as_double();
    xs = this->get_parameter("obstacles.x").as_double_array();
    ys = this->get_parameter("obstacles.y").as_double_array();
    r = this->get_parameter("obstacles.r").as_double();

    if (xs.size() != ys.size()) {
      RCLCPP_INFO(this->get_logger(), "arrays of different length");
      rclcpp::shutdown();
    }

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    marker_walls = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_walls", 10);
    marker_obs = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_obstacles", 10);
    const auto period = std::chrono::duration<double>(1.0 / static_cast<double>(rate));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&nusimulator::timer_callback, this)
    );

    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&nusimulator::reset_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_walls;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_obs;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  std_msgs::msg::UInt64 timestep_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double x0_;
  double y0_;
  double theta0_;
  double arena_x_length;
  double arena_y_length;
  std::vector<double> xs;
  std::vector<double> ys;
  double r;

  visualization_msgs::msg::Marker makeCylinder(
    int id,
    double x, double y,
    double radius)
  {
    visualization_msgs::msg::Marker m;
    
    m.header.frame_id = "nusim/world";
    m.header.stamp = this->now();

    m.ns = "red";
    m.id = id;

    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = 0.125;
    m.pose.orientation.w = 1.0; 

    m.scale.x = 2 * radius;
    m.scale.y = 2 * radius;
    m.scale.z = 0.25;

    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.lifetime = rclcpp::Duration::from_nanoseconds(0);

    return m;
  }

  visualization_msgs::msg::Marker makeRectangle(
    int id,
    double x, double y,
    double x_length,
    double y_length)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "nusim/world";
    m.header.stamp = this->now();

    m.ns = "red";
    m.id = id;

    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = 0.125;
    m.pose.orientation.w = 1.0;

    m.scale.x = x_length;
    m.scale.y = y_length;
    m.scale.z = 0.25;

    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.lifetime = rclcpp::Duration::from_nanoseconds(0);

    return m;
  }

  void timer_callback()
  {
    timestep_.data++;
    this->publisher_->publish(timestep_);

    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = x0_;
    t.transform.translation.y = y0_;
    t.transform.translation.z = 0.0;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    q.setRPY(0, 0, this->theta0_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    visualization_msgs::msg::MarkerArray arr;
    const double px = arena_x_length / 2;
    const double py = arena_y_length / 2;
    arr.markers.push_back(makeRectangle(0, px + 0.05, 0, 0.1, arena_y_length + 0.2));
    arr.markers.push_back(makeRectangle(1, 0, py + 0.05, arena_x_length, 0.1));
    arr.markers.push_back(makeRectangle(2, -px - 0.05, 0, 0.1, arena_y_length + 0.2));
    arr.markers.push_back(makeRectangle(3, 0, -py - 0.05, arena_x_length, 0.1));

    marker_walls->publish(arr);

    visualization_msgs::msg::MarkerArray obs;
    for (std::size_t i = 0; i < xs.size(); i++) {
      obs.markers.push_back(makeCylinder(i, xs.at(i), ys.at(i), r));
    }

    marker_obs->publish(obs);
  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
  {
    timestep_.data = 0;
    RCLCPP_INFO(this->get_logger(), "Reset timestep to 0");
    x0_ = this->get_parameter("x0").as_double();
    y0_ = this->get_parameter("y0").as_double();
    theta0_ = this->get_parameter("theta0").as_double();
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nusimulator>());
  rclcpp::shutdown();
  return 0;
}