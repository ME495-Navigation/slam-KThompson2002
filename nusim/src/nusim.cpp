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

using namespace std::chrono_literals;

class nusimulator : public rclcpp::Node
{
public:
  nusimulator()
  : Node("nusimulator")
  {
    this->declare_parameter("rate", 100);
    const int rate = this->get_parameter("rate").as_int();

    publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
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
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  std_msgs::msg::UInt64 timestep_;

  void timer_callback()
  {
        timestep_.data++;
        this->publisher_->publish(timestep_);
  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
  {
    timestep_.data = 0;
    RCLCPP_INFO(this->get_logger(), "Reset timestep to 0");
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nusimulator>());
  rclcpp::shutdown();
  return 0;
}