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

class Circle : public rclcpp::Node
{
public:
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

  void reverse_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    vel = -vel;
    RCLCPP_INFO(this->get_logger(), "Reversed: vel=%.3f rad/s", vel);
  }

  void stop_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    running = false;
    vel = 0;
    RCLCPP_INFO(this->get_logger(), "Stopped.");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
