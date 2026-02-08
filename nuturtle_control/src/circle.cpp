#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"

#include "nuturtle_control_interfaces/srv/circle_control.hpp"

using namespace std::chrono_literals;

class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle")
  {
    this->declare_parameter("frequency", 100.0);
    freq = this->get_parameter("frequency").get_double();

    control_srv_ = this->create_service<nuturtle_control_interfaces::srv::CircleControl>(
      "control",
      std::bind(&Circle::control_cb, this, std::placeholders::_1, std::placeholders::_2)
    );

    reverse_srv_ = this->create_service<std_srvs::srv::Empty>(
      "reverse",
      std::bind(&Circle::reverse_cb, this, std::placeholders::_1, std::placeholders::_2)
    );

    stop_srv_ = this->create_service<std_srvs::srv::Empty>(
      "stop",
      std::bind(&Circle::stop_cb, this, std::placeholders::_1, std::placeholders::_2)
    );

    const auto period = std::chrono::duration<double>(1.0 / static_cast<double>(freq));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&nusimulator::timer_callback, this)
    );

  }
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<nuturtle_control_interfaces::srv::CircleControl>::SharedPtr control_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;

  double freq;
  double vel;
  bool running = false;
  double vel = 0.0;
  double radius = 1.0;

  void timer_cb()
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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}