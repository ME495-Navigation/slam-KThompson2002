#include <catch2/catch_test_macros.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <string>

using namespace std::chrono_literals;

static int msg_count = 0;

void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr)
{
  msg_count++;
}

TEST_CASE("circle cmd_vel frequency", "[integration][circle]")
{
  auto node = rclcpp::Node::make_shared("turtle_circle_test_node");

  node->declare_parameter<double>("expected_hz");
  node->declare_parameter<double>("tolerance_hz", 10.0);
  node->declare_parameter<double>("test_duration", 2.0);
  node->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");

  const double expected_hz =
    node->get_parameter("expected_hz").get_parameter_value().get<double>();
  const double tolerance_hz =
    node->get_parameter("tolerance_hz").as_double();
  const double test_duration =
    node->get_parameter("test_duration").get_parameter_value().get<double>();
  const std::string topic =
    node->get_parameter("cmd_vel_topic").as_string();

  msg_count = 0;

  auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
    topic, 50, cmd_vel_callback);

  rclcpp::Clock clock(RCL_STEADY_TIME);
  auto start = clock.now();
  auto duration = rclcpp::Duration::from_seconds(test_duration);

  rclcpp::Rate rate(200);
  while (rclcpp::ok() && (clock.now() - start) < duration) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  REQUIRE(msg_count > 0);

  const double observed_hz = msg_count / test_duration;

  CAPTURE(msg_count, observed_hz);

  REQUIRE(std::abs(observed_hz - expected_hz) <= tolerance_hz);
}
