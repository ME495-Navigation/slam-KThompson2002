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
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class turtle_control : public rclcpp::Node
{
public:
  turtle_control()
  : Node("turtle_control")
  {
    this->declare_parameter<double>("wheel_radius", rclcpp::PARAMETER_NOT_SET);
    this->declare_parameter<double>("track_width",  rclcpp::PARAMETER_NOT_SET);

    if (!load_required_params()) {
      rclcpp::shutdown();
      return;
    }
    twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10,
        std::bind(&turtle_control::twist_callback, this, _1)
    );

  }
private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_;
  double wheel_radius;
  double track_width;
  double v;
  double w;


  bool load_required_params()
  {
    auto pr = this->get_parameter("wheel_radius");
    if (pr.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
        "Missing required parameter: wheel_radius");
      return false;
    }
     wheel_radius = pr.as_double();
    

    auto pt = this->get_parameter("track_width");
    if (pt.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
        "Missing required parameter: track_width");
      return false;
    }
    track_width = pt.as_double();
    return true;
  }

  void twist_callback(const geometry_msgs::msg::Twist & msg)
  {
    v = msg.linear.x;
    w = msg.angular.z;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtle_control>());
  rclcpp::shutdown();
  return 0;
}
