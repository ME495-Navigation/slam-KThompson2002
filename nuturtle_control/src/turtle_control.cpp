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

/// \file
/// \brief ROS 2 node that converts cmd_vel into wheel commands and publishes joint states.
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "turtlelib/diff_drive.hpp"

#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

/// \brief Node that bridges high-level velocity commands to low-level wheel commands.
///
/// It also republishes wheel encoder data as JointState for downstream consumers.
class turtle_control : public rclcpp::Node
{
public:
  /// \brief Construct the turtle_control node.
  ///
  /// Declares required parameters, loads them, initializes the DiffDrive model,
  /// and creates the necessary publishers and subscribers.
  turtle_control()
  : Node("turtle_control"),
    diff(1.0, 1.0)
  {
    this->declare_parameter<double>("wheel_radius", rclcpp::PARAMETER_NOT_SET);
    this->declare_parameter<double>("track_width", rclcpp::PARAMETER_NOT_SET);
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
    );

  }

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

  /// \brief Load and validate required ROS parameters.
  ///
  /// Reads wheel_radius, track_width, motor_cmd_max, motor_cmd_per_rad_sec, and
  /// encoder_ticks_per_rad. Logs an error and returns false if any are unset.
  /// \return true if all required parameters are available and loaded.
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

  /// \brief cmd_vel subscriber callback.
  ///
  /// Converts a body twist command into left/right wheel angular velocities via
  /// inverse kinematics, scales to motor command units, clamps to motor_cmd_max,
  /// and publishes a WheelCommands message.
  /// \param msg [in] Incoming twist command in the body frame.
  void twist_callback(const geometry_msgs::msg::Twist & msg)
  {
    turtlelib::Twist2D twist;
    twist.x = msg.linear.x;
    twist.omega = msg.angular.z;
    twist.y = 0.0;
    turtlelib::Wheel wdot = diff.inverseKinematics(twist);

    int left_mcu = static_cast<int>(std::round(wdot.left / motor_cmd_per_rad_sec));
    int right_mcu = static_cast<int>(std::round(wdot.right / motor_cmd_per_rad_sec));
    int cmd_max = static_cast<int>(motor_cmd_max);
    left_mcu = std::clamp(left_mcu, -cmd_max, cmd_max);
    right_mcu = std::clamp(right_mcu, -cmd_max, cmd_max);

    auto cmd = nuturtlebot_msgs::msg::WheelCommands();
    cmd.left_velocity = left_mcu;
    cmd.right_velocity = right_mcu;
    wheel_cmd->publish(cmd);
  }

  /// \brief sensor_data subscriber callback.
  ///
  /// Converts encoder ticks to wheel joint positions (rad) and estimates wheel
  /// velocities by finite differencing successive messages, then publishes a
  /// JointState message.
  /// \param msgs [in] Incoming sensor data containing encoder tick counts and stamp.
  void sensor_callback(const nuturtlebot_msgs::msg::SensorData & msgs)
  {
    rclcpp::Time stamp(msgs.stamp.sec, msgs.stamp.nanosec, RCL_ROS_TIME);
    double left_pos = static_cast<double>(msgs.left_encoder) / encoder_ticks_per_rad;
    double right_pos = static_cast<double>(msgs.right_encoder) / encoder_ticks_per_rad;

    double left_vel = 0.0;
    double right_vel = 0.0;

    if (have_prev_) {
      rclcpp::Time prev_stamp(prev_sensor_.stamp.sec,
        prev_sensor_.stamp.nanosec,
        RCL_ROS_TIME);

      double dt = (stamp - prev_stamp).seconds();

      int32_t d_left_ticks = msgs.left_encoder - prev_sensor_.left_encoder;
      int32_t d_right_ticks = msgs.right_encoder - prev_sensor_.right_encoder;

      left_vel = static_cast<double>(d_left_ticks) / (encoder_ticks_per_rad * dt);
      right_vel = static_cast<double>(d_right_ticks) / (encoder_ticks_per_rad * dt);
    }

    auto cmd = sensor_msgs::msg::JointState();
    cmd.header.stamp = stamp;
    cmd.name = {"wheel_left_joint", "wheel_right_joint"};
    cmd.position = {left_pos, right_pos};
    cmd.velocity = {left_vel, right_vel};
    joint_states->publish(cmd);

    prev_sensor_ = msgs;
    have_prev_ = true;
  }
};

/// \brief Entry point for the turtle_control node.
/// \return 0 on clean shutdown.
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtle_control>());
  rclcpp::shutdown();
  return 0;
}
