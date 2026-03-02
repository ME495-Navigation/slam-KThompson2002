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

/**
 * \file nusim.cpp
 * \brief A simple 2D simulator node that publishes time, a static robot pose TF, and visualization markers.
 *
 * nusimulator:
 *  - Publishes a timestep counter on `~/timestep` as std_msgs::msg::UInt64.
 *  - Broadcasts a TF transform from `nusim/world` to `red/base_footprint` using the configured initial pose.
 *  - Publishes MarkerArray walls on `~/real_walls` and obstacle cylinders on `~/real_obstacles`.
 *  - Provides a `~/reset` service that resets timestep to 0 and reloads initial pose parameters.
 *
 * Parameters:
 *  - rate (int): Timer update rate in Hz (default 100)
 *  - x0 (double): Initial x position of the robot in world frame (default 0.0)
 *  - y0 (double): Initial y position of the robot in world frame (default 0.0)
 *  - theta0 (double): Initial yaw of the robot in world frame [rad] (default 0.0)
 *  - arena_x_length (double): Arena length in x [m] (default 7.0)
 *  - arena_y_length (double): Arena length in y [m] (default 7.0)
 *  - obstacles.x (double[]): Obstacle x coordinates [m] (default empty)
 *  - obstacles.y (double[]): Obstacle y coordinates [m] (default empty)
 *  - obstacles.r (double): Obstacle radius [m] (default 0.0)
 */

#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/**
 * \class nusimulator
 * \brief ROS 2 node that periodically publishes simulation time, TF, and RViz markers.
 *
 * The node runs a wall timer at a configurable rate. Each tick:
 *  - Increments and publishes an internal timestep counter.
 *  - Broadcasts the robot pose transform using the current x0_, y0_, theta0_ state.
 *  - Publishes wall markers that form a rectangle boundary using arena_x_length and arena_y_length.
 *  - Publishes obstacle cylinder markers using the `obstacles.*` parameters.
 *
 * The `~/reset` service resets the timestep counter and reloads x0, y0, theta0 from parameters.
 */
class nusimulator : public rclcpp::Node
{
public:
  /**
   * \brief Construct the nusimulator node and initialize ROS interfaces.
   *
   * Declares and reads parameters, validates obstacle coordinate arrays, sets up:
   *  - TF broadcaster
   *  - Publishers: `~/timestep`, `~/real_walls`, `~/real_obstacles`
   *  - Wall timer running at `rate` Hz
   *  - Reset service `~/reset`
   *
   * If obstacle x and y arrays have different lengths, the node logs and shuts down.
   */
  nusimulator()
  : Node("nusimulator")
  {
    this->declare_parameter("rate", 100);
    this->declare_parameter("x0", 0.0);
    this->declare_parameter("y0", 0.0);
    this->declare_parameter("theta0", 0.0);
    this->declare_parameter("arena_x_length", 7.0);
    this->declare_parameter("arena_y_length", 7.0);
    this->declare_parameter("track_width", 0.16);
    this->declare_parameter("wheel_radius", 0.033);
    this->declare_parameter("encoder_ticks_per_rad", 651.9);
    this->declare_parameter("motor_cmd_per_rad_sec", 0.024);
    this->declare_parameter("range_min", 0.12);
    this->declare_parameter("range_max", 3.5);
    this->declare_parameter("angle_increments", 0.01745);
    this->declare_parameter("num_samples", 360);
    this->declare_parameter("resolution", 10.0);
    this->declare_parameter("noise", 180.0);
    this->declare_parameter("input_noise", 0.0);
    this->declare_parameter("slip_fraction", 0.0);
    this->declare_parameter("collision_radius", 0.11);
    this->declare_parameter<std::vector<double>>("obstacles.x", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("obstacles.y", std::vector<double>{});
    this->declare_parameter<double>("obstacles.r", 0.0);


    rate = this->get_parameter("rate").as_int();
    x_ = this->get_parameter("x0").as_double();
    y_ = this->get_parameter("y0").as_double();
    theta_ = this->get_parameter("theta0").as_double();
    arena_x_length = this->get_parameter("arena_x_length").as_double();
    arena_y_length = this->get_parameter("arena_y_length").as_double();
    track_width = this->get_parameter("track_width").as_double();
    wheel_radius = this->get_parameter("wheel_radius").as_double();
    encoder_ticks_per_rad = this->get_parameter("encoder_ticks_per_rad").as_double();
    motor_cmd_per_rad_sec = this->get_parameter("motor_cmd_per_rad_sec").as_double();
    xs = this->get_parameter("obstacles.x").as_double_array();
    ys = this->get_parameter("obstacles.y").as_double_array();
    r = this->get_parameter("obstacles.r").as_double();
    range_min = this->get_parameter("range_min").as_double();
    range_max = this->get_parameter("range_max").as_double();
    angle_increment = this->get_parameter("angle_increments").as_double();
    num_samples = this->get_parameter("num_samples").as_int();
    resolution = this->get_parameter("resolution").as_double();
    noise = this->get_parameter("noise").as_double();
    input_noise = this->get_parameter("input_noise").as_double();
    slip_fraction = this->get_parameter("slip_fraction").as_double();
    collision_radius = this->get_parameter("collision_radius").as_double();


    if (xs.size() != ys.size()) {
      RCLCPP_INFO(this->get_logger(), "arrays of different length");
      rclcpp::shutdown();
    }

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    diff = std::make_unique<turtlelib::DiffDrive>(track_width, wheel_radius);
    diff->setPose(turtlelib::Transform2D(turtlelib::Vector2D{x_, y_}, theta_));

    publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    marker_walls = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_walls", 10);
    marker_obs = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_obstacles",
      10);
    fake_sensor = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/fake_sensor", 10);
    const auto period = std::chrono::duration<double>(1.0 / static_cast<double>(rate));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&nusimulator::timer_callback, this)
    );
    const auto sensor_period = std::chrono::duration<double>(1.0 / static_cast<double>(sensor_hz));
    sensor_timer = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(sensor_period),
      std::bind(&nusimulator::sensor_timer_callback, this)
    );

    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&nusimulator::reset_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    wheel_cmd = this->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
        "red/wheel_cmd",
        10,
        std::bind(&nusimulator::wheel_callback, this, _1)
    );

    sensor_data = this->create_publisher<nuturtlebot_msgs::msg::SensorData>(
        "red/sensor_data",
        10
    );

    joint_states = this->create_publisher<sensor_msgs::msg::JointState>(
        "red/joint_states",
        10
    );

    nav_path = this->create_publisher<nav_msgs::msg::Path>(
      "red/nav_path",
      10
    );

    laser_scan = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "red/laser_scan",
      10
    );
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr sensor_timer;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_walls;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_obs;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data;
  std_msgs::msg::UInt64 timestep_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nav_path;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan;

  int rate;
  double sensor_hz = 5.0;

  double x_;
  double y_;
  double theta_;
  double arena_x_length;
  double arena_y_length;
  std::vector<double> xs;
  std::vector<double> ys;
  double r;
  turtlelib::Wheel wheel_pos{0.0, 0.0};
  double left_wheel_vel = 0.0;
  double right_wheel_vel = 0.0;
  std::unique_ptr<turtlelib::DiffDrive> diff;
  double track_width;
  double wheel_radius;
  double encoder_ticks_per_rad;
  double motor_cmd_per_rad_sec;
  double collision_radius;
  // Laser Scan Constants
  double range_min;
  double range_max;
  double angle_increment;
  int num_samples;
  double resolution;
  double noise;

  // Error Constants
  double input_noise = 0.0;
  double slip_fraction = 0.0;


  std::vector<geometry_msgs::msg::PoseStamped> poses;

  /**
   * \brief Create a cylindrical RViz marker representing an obstacle.
   *
   * Marker properties:
   *  - frame_id: "nusim/world"
   *  - type: CYLINDER
   *  - color: opaque red
   *  - scale: diameter = 2*radius in x/y, fixed height in z
   *
   * \param id Unique marker ID within its namespace.
   * \param x Obstacle x position in world frame [m].
   * \param y Obstacle y position in world frame [m].
   * \param radius Obstacle radius [m].
   * \return A fully-populated visualization_msgs::msg::Marker.
   */
  visualization_msgs::msg::Marker makeCylinder(
    int id,
    double x, double y,
    double radius)
  {
    visualization_msgs::msg::Marker m;

    m.header.frame_id = "nusim/world";
    m.header.stamp = this->now();

    m.ns = "red";
    m.id = id;

    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = 0.125;
    m.pose.orientation.w = 1.0;

    m.scale.x = 2 * radius;
    m.scale.y = 2 * radius;
    m.scale.z = 0.25;

    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.lifetime = rclcpp::Duration::from_nanoseconds(0);

    return m;
  }

  visualization_msgs::msg::Marker makeNoisyCylinder(
    int id,
    double x, double y,
    double radius)
  {
    visualization_msgs::msg::Marker m;

    m.header.frame_id = "nusim/world";
    m.header.stamp = this->now();

    m.ns = "red";
    m.id = id;

    m.type = visualization_msgs::msg::Marker::CYLINDER;

    const double dx = x - x_;
    const double dy = y - y_;
    if (std::sqrt(dx * dx + dy * dy) > range_max) {
      m.action = visualization_msgs::msg::Marker::DELETE;
      return m;
    }

    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = 0.125;
    m.pose.orientation.w = 1.0;

    m.scale.x = 2 * radius;
    m.scale.y = 2 * radius;
    m.scale.z = 0.25;

    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.lifetime = rclcpp::Duration::from_nanoseconds(0);

    return m;
  }

    /**
   * \brief Create a rectangular RViz marker (CUBE) representing a wall segment.
   *
   * Marker properties:
   *  - frame_id: "nusim/world"
   *  - type: CUBE
   *  - color: opaque red
   *  - scale: (x_length, y_length, fixed height)
   *
   * \param id Unique marker ID within its namespace.
   * \param x Rectangle center x position in world frame [m].
   * \param y Rectangle center y position in world frame [m].
   * \param x_length Rectangle length along x [m].
   * \param y_length Rectangle length along y [m].
   * \return A fully-populated visualization_msgs::msg::Marker.
   */
  visualization_msgs::msg::Marker makeRectangle(
    int id,
    double x, double y,
    double x_length,
    double y_length)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "nusim/world";
    m.header.stamp = this->now();

    m.ns = "red";
    m.id = id;

    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = 0.125;
    m.pose.orientation.w = 1.0;

    m.scale.x = x_length;
    m.scale.y = y_length;
    m.scale.z = 0.25;

    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.lifetime = rclcpp::Duration::from_nanoseconds(0);

    return m;
  }

  void wheel_callback(const nuturtlebot_msgs::msg::WheelCommands & msgs)
  {
    auto left_wheel_ui = static_cast<double>(msgs.left_velocity) * motor_cmd_per_rad_sec;
    auto right_wheel_ui = static_cast<double>(msgs.right_velocity) * motor_cmd_per_rad_sec;
    
    std::normal_distribution<double> d(0.0, input_noise);
    left_wheel_vel = left_wheel_ui + d(get_random());
    right_wheel_vel = right_wheel_ui + d(get_random());
  }

  /**
   * \brief Periodic timer callback that advances simulation outputs.
   *
   * Actions performed each tick:
   *  - Increment timestep_ and publish on `~/timestep`.
   *  - Broadcast TF: "nusim/world" -> "red/base_footprint" using (x0_, y0_, theta0_).
   *  - Publish wall MarkerArray forming an arena boundary on `~/real_walls`.
   *  - Publish obstacle cylinder MarkerArray using xs/ys and radius r on `~/real_obstacles`.
   */
  void timer_callback()
  {
    const double dt = 1.0 / rate;
    timestep_.data++;
    this->publisher_->publish(timestep_);

    std::uniform_real_distribution<double> slip_dist(-slip_fraction, slip_fraction);
    wheel_pos.left += left_wheel_vel * (1.0 + slip_dist(get_random())) * dt;
    wheel_pos.right += right_wheel_vel * (1.0 + slip_dist(get_random())) * dt;

    nuturtlebot_msgs::msg::SensorData msg;
    msg.stamp = this->get_clock()->now();

    msg.left_encoder = static_cast<int32_t>(std::round(wheel_pos.left * encoder_ticks_per_rad));
    msg.right_encoder = static_cast<int32_t>(std::round(wheel_pos.right * encoder_ticks_per_rad));
    sensor_data->publish(msg);

    (void)diff->forwardKinematics(wheel_pos);

    const turtlelib::Transform2D T = diff->pose();
    const turtlelib::Vector2D p = T.translation();
    x_ = p.x;
    y_ = p.y;
    theta_ = T.rotation();

    // Collision detection and response (one cylinder at a time)
    for (std::size_t i = 0; i < xs.size(); i++) {
      const double dx = x_ - xs.at(i);
      const double dy = y_ - ys.at(i);
      const double dist = std::sqrt(dx * dx + dy * dy);
      const double min_dist = collision_radius + r;
      if (dist < min_dist) {
        // Move robot along the robot-obstacle line until circles are tangent
        x_ = xs.at(i) + min_dist * (dx / dist);
        y_ = ys.at(i) + min_dist * (dy / dist);
        diff->setPose(turtlelib::Transform2D(turtlelib::Vector2D{x_, y_}, theta_));
        break;
      }
    }

    auto cmd = sensor_msgs::msg::JointState();
    cmd.header.stamp = this->get_clock()->now();
    cmd.name = {"wheel_left_joint", "wheel_right_joint"};
    cmd.position = {wheel_pos.left, wheel_pos.right};
    cmd.velocity = {left_wheel_vel, right_wheel_vel};
    joint_states->publish(cmd);


    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";

    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, this->theta_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->get_clock()->now();
    pose.header.frame_id = "nusim/world";
    pose.pose.position.x = x_;
    pose.pose.position.y = y_;
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    poses.push_back(pose);

    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "nusim/world";
    path.poses = poses;
    nav_path->publish(path);

    // Laser Scan implementation
    
    visualization_msgs::msg::MarkerArray arr;
    const double px = arena_x_length / 2;
    const double py = arena_y_length / 2;
    arr.markers.push_back(makeRectangle(0, px + 0.05, 0, 0.1, arena_y_length + 0.2));
    arr.markers.push_back(makeRectangle(1, 0, py + 0.05, arena_x_length, 0.1));
    arr.markers.push_back(makeRectangle(2, -px - 0.05, 0, 0.1, arena_y_length + 0.2));
    arr.markers.push_back(makeRectangle(3, 0, -py - 0.05, arena_x_length, 0.1));

    marker_walls->publish(arr);

    visualization_msgs::msg::MarkerArray obs;
    for (std::size_t i = 0; i < xs.size(); i++) {
      obs.markers.push_back(makeCylinder(i, xs.at(i), ys.at(i), r));
    }

    marker_obs->publish(obs);
  }

  void sensor_timer_callback()
  {
    visualization_msgs::msg::MarkerArray fake_sensor_;
    for (std::size_t i = 0; i < xs.size(); i++) {
      fake_sensor_.markers.push_back(makeNoisyCylinder(i, xs.at(i), ys.at(i), r));
    }
    fake_sensor->publish(fake_sensor_);

    // --- Simulated LaserScan ---
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = this->get_clock()->now();
    scan.header.frame_id = "red/base_footprint";
    scan.angle_min = 0.0;
    scan.angle_max = scan.angle_min + (num_samples - 1) * angle_increment;
    scan.angle_increment = angle_increment;
    scan.time_increment = 0.0;
    scan.scan_time = 1.0 / sensor_hz;
    scan.range_min = range_min;
    scan.range_max = range_max;
    scan.ranges.resize(num_samples, 0.0f);

    std::normal_distribution<double> noise_dist(0.0, noise);

    const double half_x = arena_x_length / 2.0;
    const double half_y = arena_y_length / 2.0;

    for (int s = 0; s < num_samples; s++) {
      // Ray direction in world frame: body angle + per-sample angle
      const double ray_angle = theta_ + scan.angle_min + s * angle_increment;
      const double cos_a = std::cos(ray_angle);
      const double sin_a = std::sin(ray_angle);

      double min_t = range_max + 1.0;  // start beyond max so we know if nothing was hit

      // Ray-cylinder intersections
      // Ray: P(t) = (x_ + t*cos_a, y_ + t*sin_a)
      // Solve: |P(t) - C|^2 = r^2  =>  t^2 + b*t + c = 0
      for (std::size_t i = 0; i < xs.size(); i++) {
        const double dx = x_ - xs.at(i);
        const double dy = y_ - ys.at(i);
        const double b = 2.0 * (dx * cos_a + dy * sin_a);
        const double c = dx * dx + dy * dy - r * r;
        const double disc = b * b - 4.0 * c;
        if (disc < 0.0) {
          continue;
        }
        const double sqrt_disc = std::sqrt(disc);
        const double t1 = (-b - sqrt_disc) / 2.0;
        const double t2 = (-b + sqrt_disc) / 2.0;
        // Smallest positive root is the entry point
        const double t = (t1 > 0.0) ? t1 : ((t2 > 0.0) ? t2 : -1.0);
        if (t > 0.0 && t < min_t) {
          min_t = t;
        }
      }

      // Ray-wall intersections (axis-aligned bounding box)
      // Right wall: x = +half_x
      if (std::abs(cos_a) > 1e-9) {
        const double t = (half_x - x_) / cos_a;
        if (t > 0.0 && t < min_t && std::abs(y_ + t * sin_a) <= half_y) {
          min_t = t;
        }
      }
      // Left wall: x = -half_x
      if (std::abs(cos_a) > 1e-9) {
        const double t = (-half_x - x_) / cos_a;
        if (t > 0.0 && t < min_t && std::abs(y_ + t * sin_a) <= half_y) {
          min_t = t;
        }
      }
      // Top wall: y = +half_y
      if (std::abs(sin_a) > 1e-9) {
        const double t = (half_y - y_) / sin_a;
        if (t > 0.0 && t < min_t && std::abs(x_ + t * cos_a) <= half_x) {
          min_t = t;
        }
      }
      // Bottom wall: y = -half_y
      if (std::abs(sin_a) > 1e-9) {
        const double t = (-half_y - y_) / sin_a;
        if (t > 0.0 && t < min_t && std::abs(x_ + t * cos_a) <= half_x) {
          min_t = t;
        }
      }

      // Only report a return if the hit is within sensor range
      if (min_t >= range_min && min_t <= range_max) {
        const double noisy_range = min_t + noise_dist(get_random());
        scan.ranges[s] = static_cast<float>(
          std::clamp(noisy_range, static_cast<double>(range_min), static_cast<double>(range_max)));
      }
      // else leave as 0.0 (no return / out of range)
    }

    laser_scan->publish(scan);
  }

   /**
   * \brief Reset service callback.
   *
   * Resets the timestep counter to 0 and reloads x0, y0, theta0 from parameters.
   *
   * \param req Empty request (unused).
   * \param res Empty response (unused).
   */
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
  {
    timestep_.data = 0;
    RCLCPP_INFO(this->get_logger(), "Reset timestep to 0");
    x_ = this->get_parameter("x0").as_double();
    y_ = this->get_parameter("y0").as_double();
    theta_ = this->get_parameter("theta0").as_double();
    diff->setPose(turtlelib::Transform2D(turtlelib::Vector2D{x_, y_}, theta_));

    wheel_pos = {0.0, 0.0};
    left_wheel_vel = 0.0;
    right_wheel_vel = 0.0;
  }

  std::mt19937 & get_random()
  {
     // static variables inside a function are created once and persist for the remainder of the program
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     // we return a reference to the pseudo-random number genrator object. This is always the
     // same object every time get_random is called
     return mt;
  }
};

/**
 * \brief Entry point. Initializes ROS 2 and spins the nusimulator node.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nusimulator>());
  rclcpp::shutdown();
  return 0;
}
