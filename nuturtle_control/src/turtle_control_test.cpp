#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include <numbers>
#include <cmath>
#include <chrono>

double vel_left;
double vel_right;
turtlelib::Wheel wheel_pos;
static bool got_cmd_msg = false;
static bool got_joint_msg = false;

void wheel_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msgs)
{
  got_cmd_msg = true;
  vel_left = msgs->left_velocity;
  vel_right = msgs->right_velocity;
}

void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msgs)
{
  got_joint_msg = true;
  vel_left = msgs->velocity[0];
  vel_right = msgs->velocity[1];

  wheel_pos.left = msgs->position[0];
  wheel_pos.right = msgs->position[1];
}

TEST_CASE("Turtle Control Tests")
{
    auto node = std::make_shared<rclcpp::Node>("turtle_control_test");

    node->declare_parameter<double>("wheel_radius");
    node->declare_parameter<double>("track_width");
    node->declare_parameter<double>("motor_cmd_max");
    node->declare_parameter<double>("motor_cmd_per_rad_sec");

    const double wheel_radius =
    node->get_parameter("wheel_radius").get_parameter_value().get<double>();

    const double track_width =
    node->get_parameter("track_width").get_parameter_value().get<double>();

    const double motor_cmd_max =
    node->get_parameter("motor_cmd_max").get_parameter_value().get<double>();

    const double motor_cmd_per_rad_sec =
    node->get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();

    auto cmd_vel = node->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10
    );

    auto sensor_data = node->create_publisher<nuturtlebot_msgs::msg::SensorData>(
        "sensor_data",
        10
    );

    auto wheel_cmd = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
        "wheel_cmd",
        10,
        &wheel_callback
    );

    auto joint_states = node->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states",
        10,
        &joint_callback
    );

    SECTION("Pure Translation")
    {
        vel_left = 0.0;
        vel_right = 0.0;

        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0.2;
        twist.angular.z = 0.0;

        turtlelib::DiffDrive dd(track_width, wheel_radius);

        turtlelib::Twist2D Vb;
        Vb.omega = twist.angular.z;
        Vb.x = twist.linear.x;
        Vb.y = 0.0;

        const turtlelib::Wheel wdot = dd.inverseKinematics(Vb);

        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        const auto start_time = steady_clock.now();
        const auto test_duration = rclcpp::Duration::from_seconds(1.0);

        auto to_motor_cmd = [&](double w) {
        double cmd = w / motor_cmd_per_rad_sec;

        if (cmd > motor_cmd_max) {cmd = motor_cmd_max;}
        if (cmd < -motor_cmd_max) {cmd = -motor_cmd_max;}

        return static_cast<int>(cmd);
      };

        const double expected_left = to_motor_cmd(wdot.left);
        const double expected_right = to_motor_cmd(wdot.right);

        rclcpp::Rate rate(200);
        while (rclcpp::ok() && (steady_clock.now() - start_time) < test_duration) {
      rclcpp::spin_some(node);
      cmd_vel->publish(twist);
      rate.sleep();
        }
        REQUIRE(got_cmd_msg);
        CAPTURE(vel_left, vel_right);

        REQUIRE_THAT(vel_left, Catch::Matchers::WithinAbs(expected_left, 1.0));
        REQUIRE_THAT(vel_right, Catch::Matchers::WithinAbs(expected_right, 1.0));
    }

    SECTION("Pure Rotation")
    {
        vel_left = 0.0;
        vel_right = 0.0;

        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0.0;
        twist.angular.z = 1.0;

        turtlelib::DiffDrive dd(track_width, wheel_radius);

        turtlelib::Twist2D Vb;
        Vb.omega = twist.angular.z;
        Vb.x = twist.linear.x;
        Vb.y = 0.0;

        const turtlelib::Wheel wdot = dd.inverseKinematics(Vb);

        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        const auto start_time = steady_clock.now();
        const auto test_duration = rclcpp::Duration::from_seconds(1.0);

        auto to_motor_cmd = [&](double w) {
        double cmd = w / motor_cmd_per_rad_sec;

        if (cmd > motor_cmd_max) {cmd = motor_cmd_max;}
        if (cmd < -motor_cmd_max) {cmd = -motor_cmd_max;}

        return static_cast<int>(cmd);
      };

        const double expected_left = to_motor_cmd(wdot.left);
        const double expected_right = to_motor_cmd(wdot.right);

        rclcpp::Rate rate(200);
        while (rclcpp::ok() && (steady_clock.now() - start_time) < test_duration) {
      rclcpp::spin_some(node);
      cmd_vel->publish(twist);
      rate.sleep();
        }
        REQUIRE(got_cmd_msg);
        CAPTURE(vel_left, vel_right);

        REQUIRE_THAT(vel_left, Catch::Matchers::WithinAbs(expected_left, 1.0));
        REQUIRE_THAT(vel_right, Catch::Matchers::WithinAbs(expected_right, 1.0));
    }

    SECTION("Encoder to Joint State Conversion")
    {
      got_joint_msg = false;
      vel_left = 0.0;
      vel_right = 0.0;
      wheel_pos.left = 0.0;
      wheel_pos.right = 0.0;
    }
}
