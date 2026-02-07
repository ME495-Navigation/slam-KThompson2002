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
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class turtle_control : public rclcpp::Node
{
public:
  turtle_control()
  : Node("turtle_control"),
  diff(0.0, 0.0)
  {
    this->declare_parameter<double>("wheel_radius", rclcpp::PARAMETER_NOT_SET);
    this->declare_parameter<double>("track_width",  rclcpp::PARAMETER_NOT_SET);
    this->declare_parameter<double>("motor_cmd_max", rclcpp::PARAMETER_NOT_SET);
    this->declare_parameter<double>("motor_cmd_per_rad_sec", rclcpp::PARAMETER_NOT_SET);
    this->declare_parameter<double>("encoder_ticks_per_rad", rclcpp::PARAMETER_NOT_SET);

    if (!load_required_params()) {
      rclcpp::shutdown();
      return;
    }
    diff = turtlelib::DiffDrive(track_width, wheel_radius);

    // Establish Subscribers
    cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10,
        std::bind(&turtle_control::twist_callback, this, _1)
    );
    sensor_data = this->create_subscription<nuturtlebot_msgs::msg::SensorData>(
        "sensor_data",
        10,
        std::bind(&turtle_control::sensor_callback, this, _1)
    );

    // Establish Publishers
    wheel_cmd = this->create_publisher<nuturtlebot_msgs::msg::WheelCommands>(
        "wheel_cmd",
        10
    );
    joint_states = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states",
        10
    )

  };
private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states;

  // diff_params;
  double wheel_radius;
  double track_width;
  double motor_cmd_max;
  double motor_cmd_per_rad_sec;
  double encoder_ticks_per_rad;

  turtlelib::DiffDrive diff;
  double v;
  double w;

  nuturtlebot_msgs::msg::SensorData prev_sensor_;
  bool have_prev_ = false;


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

    auto pm = this->get_parameter("motor_cmd_max");
    if (pm.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
        "Missing required parameter: motor_cmd_max");
      return false;
    }
    motor_cmd_max = pm.as_double();

    auto ps = this->get_parameter("motor_cmd_per_rad_sec");
    if (ps.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
        "Missing required parameter: motor_cmd_per_rad_sec");
      return false;
    }
    motor_cmd_per_rad_sec = ps.as_double();

    auto pe = this->get_parameter("encoder_ticks_per_rad");
    if (pe.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
        "Missing required parameter: encoder_ticks_per_rad");
      return false;
    }
    encoder_ticks_per_rad = pe.as_double();
    return true;
  }

  void twist_callback(const geometry_msgs::msg::Twist & msg)
  {
    turtlelib::Twist2D twist;
    twist.x = msg.linear.x;
    twist.omega = msg.angular.z;
    twist.y = 0.0;

    turtlelib::Wheel wdot = diff.inverseKinematics(twist);

    int left_mcu  = static_cast<int>(std::round(wdot.left  / motor_cmd_per_rad_sec));
    int right_mcu = static_cast<int>(std::round(wdot.right /  motor_cmd_per_rad_sec));

    left_mcu  = std::clamp(left_mcu,  -motor_cmd_max, motor_cmd_max);
    right_mcu = std::clamp(right_mcu, -motor_cmd_max, motor_cmd_max);

    auto cmd = nuturtlebot_msgs::msg::WheelCommands();
    cmd.left_velocity = left_mcu;
    cmd.right_velocity = right_mcu;
    wheel_cmd->publish(cmd);
  }

  void sensor_callback(const nuturtlebot_msgs::msg::SensorData & msgs)
  {
    turtlelib::Wheel wdot{0.0, 0.0};
    wdot.left = msgs.left_encoder;
    wdot.right = msgs.right_encoder;

    turtlelib::Twist2D twist = diff.forwardKinematics();
    auto cmd = sensor_msgs::msg::JointState();
    cmd.name = ["wheel_left_joint", "wheel_right_joint"]
    cmd.position = [wdot.left, wdot.right]
    cmd.velocity = ;
    joint_states->publish(cmd);
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtle_control>());
  rclcpp::shutdown();
  return 0;
}
