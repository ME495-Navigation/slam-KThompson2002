/// \file
/// \brief ROS 2 node that publishes cmd_vel commands to drive the robot in a circle.
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"

#include "nuturtle_control_interfaces/srv/circle_control.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/// \brief Node that generates a circular velocity command.
///
/// Publishes geometry_msgs::msg::Twist on the cmd_vel topic at a configured frequency.
/// The motion can be controlled via services (control, reverse, stop).
class Circle : public rclcpp::Node
{
public:
  /// \brief Construct the Circle node.
  ///
  /// Declares/reads the "frequency" parameter, creates the cmd_vel publisher,
  /// sets up control services, and starts a wall timer to publish commands.
  Circle()
  : Node("circle")
  {
    this->declare_parameter("frequency", 100.0);
    freq = this->get_parameter("frequency").as_double();

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10
    );

    control_srv_ = this->create_service<nuturtle_control_interfaces::srv::CircleControl>(
      "control",
      std::bind(&Circle::control_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    reverse_srv_ = this->create_service<std_srvs::srv::Empty>(
      "reverse",
      std::bind(&Circle::reverse_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    stop_srv_ = this->create_service<std_srvs::srv::Empty>(
      "stop",
      std::bind(&Circle::stop_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    const auto period = std::chrono::duration<double>(1.0 / static_cast<double>(freq));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Circle::timer_callback, this)
    );

  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<nuturtle_control_interfaces::srv::CircleControl>::SharedPtr control_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;

  double freq;
  bool running = false;
  double vel = 0.0;
  double radius = 1.0;

  /// \brief Timer callback to publish the current velocity command.
  ///
  /// When running is false, publishes zero linear and angular velocity.
  /// Otherwise publishes a twist corresponding to the current (velocity, radius).
  void timer_callback()
  {
    geometry_msgs::msg::Twist cmd;

    if (!running) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    } else {
      cmd.linear.x = vel * radius;
      cmd.angular.z = vel;
    }

    cmd_pub_->publish(cmd);
  }

  /// \brief Service callback to start/modify circular motion.
  ///
  /// \param req [in] Requested angular velocity (rad/s) and radius (m).
  /// \param res [out] Response indicating success.
  void control_callback(
    const std::shared_ptr<nuturtle_control_interfaces::srv::CircleControl::Request> req,
    std::shared_ptr<nuturtle_control_interfaces::srv::CircleControl::Response> res)
  {
    vel = req->velocity;
    radius = req->radius;
    running = true;

    res->success = true;
    RCLCPP_INFO(this->get_logger(), "Control set: vel%.3f rad/s, radius=%.3f m", vel, radius);
  }

  /// \brief Service callback to reverse the direction of rotation.
  ///
  /// Negates the stored angular velocity while preserving the current radius.
  /// \param req [in] Unused empty request.
  /// \param res [out] Unused empty response.
  void reverse_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    vel = -vel;
    RCLCPP_INFO(this->get_logger(), "Reversed: vel=%.3f rad/s", vel);
  }

  /// \brief Service callback to stop publishing motion commands.
  ///
  /// Sets running to false and zeros the stored velocity.
  /// \param req [in] Unused empty request.
  /// \param res [out] Unused empty response.
  void stop_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    running = false;
    vel = 0;
    RCLCPP_INFO(this->get_logger(), "Stopped.");
  }
};

/// \brief Entry point for the circle node.
/// \return 0 on clean shutdown.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
