/// \file
/// \brief ROS 2 node that computes and publishes differential-drive odometry.
#include <algorithm>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"

using std::placeholders::_1

class Slam : public rclcpp::Node
{
public:
  Slam()
  : Node("slam") 
  {

  }
private:
  
}

/// \brief Entry point for the slam node.
/// \return 0 on clean shutdown.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}
