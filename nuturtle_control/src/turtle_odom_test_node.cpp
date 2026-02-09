#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nuturtle_control_interfaces/srv/initial_pose.hpp"
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

bool wait_for_service(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::ClientBase::SharedPtr & client,
  std::chrono::milliseconds timeout)
{
  rclcpp::Clock steady(RCL_STEADY_TIME);
  auto start = steady.now();

  while (rclcpp::ok() && (steady.now() - start) < rclcpp::Duration(timeout)) {
    if (client->wait_for_service(0s)) {
      return true;
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

bool wait_for_transform(
  const rclcpp::Node::SharedPtr & node,
  tf2_ros::Buffer & buffer,
  const std::string & parent,
  const std::string & child,
  std::chrono::milliseconds timeout)
{
  rclcpp::Clock steady(RCL_STEADY_TIME);
  auto start = steady.now();

  while (rclcpp::ok() && (steady.now() - start) < rclcpp::Duration(timeout)) {
    if (buffer.canTransform(parent, child, tf2::TimePointZero)) {
      return true;
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

TEST_CASE("initial_pose service works")
{
  auto node = rclcpp::Node::make_shared("turtle_odom_test_node");

  node->declare_parameter<std::string>("odom_id", "odom");
  node->declare_parameter<std::string>("body_id", "base_footprint");
  node->declare_parameter<std::string>("wheel_left", "");
  node->declare_parameter<std::string>("wheel_right", "");

  const auto odom_id = node->get_parameter("odom_id").as_string();
  const auto body_id = node->get_parameter("body_id").as_string();
  const auto wheel_left = node->get_parameter("wheel_left").as_string();
  const auto wheel_right = node->get_parameter("wheel_right").as_string();

  REQUIRE_FALSE(wheel_left.empty());
  REQUIRE_FALSE(wheel_right.empty());

  auto client = node->create_client<nuturtle_control_interfaces::srv::InitialPose>("initial_pose");
  REQUIRE(wait_for_service(node, client, 1500ms));

  auto req = std::make_shared<nuturtle_control_interfaces::srv::InitialPose::Request>();
  req->x = 0.0;
  req->y = 0.0;
  req->theta = 0.0;

  auto future = client->async_send_request(req);

  rclcpp::Clock steady(RCL_STEADY_TIME);
  auto start = steady.now();
  while (rclcpp::ok() && (steady.now() - start) < rclcpp::Duration::from_seconds(1.5)) {
    rclcpp::spin_some(node);
    if (future.wait_for(0s) == std::future_status::ready) {
      break;
    }
    std::this_thread::sleep_for(10ms);
  }

  REQUIRE(future.wait_for(0s) == std::future_status::ready);
  const auto res = future.get();
  REQUIRE(res->success);
}

TEST_CASE("odom->base_footprint TF is published")
{
  auto node = rclcpp::Node::make_shared("turtle_odom_test_node");

  node->declare_parameter<std::string>("odom_id", "odom");
  node->declare_parameter<std::string>("body_id", "base_footprint");
  node->declare_parameter<std::string>("wheel_left", "");
  node->declare_parameter<std::string>("wheel_right", "");

  const auto odom_id = node->get_parameter("odom_id").as_string();
  const auto body_id = node->get_parameter("body_id").as_string();
  const auto wheel_left = node->get_parameter("wheel_left").as_string();
  const auto wheel_right = node->get_parameter("wheel_right").as_string();

  REQUIRE_FALSE(wheel_left.empty());
  REQUIRE_FALSE(wheel_right.empty());

  auto js_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  rclcpp::Clock steady(RCL_STEADY_TIME);
  auto start = steady.now();
  const auto pub_duration = rclcpp::Duration::from_seconds(0.5);

  rclcpp::Rate rate(100);

  while (rclcpp::ok() && (steady.now() - start) < pub_duration) {
    sensor_msgs::msg::JointState js;
    js.header.stamp = node->get_clock()->now();
    js.name = {wheel_left, wheel_right};
    js.position = {0.0, 0.0};
    js.velocity = {0.0, 0.0};

    js_pub->publish(js);
    rclcpp::spin_some(node);
    rate.sleep();
  }

  REQUIRE(wait_for_transform(node, tf_buffer, odom_id, body_id, 1500ms));

  const auto tf = tf_buffer.lookupTransform(odom_id, body_id, tf2::TimePointZero);

  REQUIRE_THAT(tf.transform.translation.x, Catch::Matchers::WithinAbs(0.0, 1e-6));
  REQUIRE_THAT(tf.transform.translation.y, Catch::Matchers::WithinAbs(0.0, 1e-6));
  REQUIRE_THAT(tf.transform.translation.z, Catch::Matchers::WithinAbs(0.0, 1e-6));

  REQUIRE_THAT(tf.transform.rotation.x, Catch::Matchers::WithinAbs(0.0, 1e-6));
  REQUIRE_THAT(tf.transform.rotation.y, Catch::Matchers::WithinAbs(0.0, 1e-6));
  REQUIRE_THAT(tf.transform.rotation.z, Catch::Matchers::WithinAbs(0.0, 1e-6));
  REQUIRE_THAT(tf.transform.rotation.w, Catch::Matchers::WithinAbs(1.0, 1e-6));
}
