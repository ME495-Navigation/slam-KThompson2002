/// \file
/// \brief EKF-SLAM implementation for a differential-drive robot.

#include "nuslam/ekf.hpp"
#include "turtlelib/angle.hpp"

#include <cmath>
#include <stdexcept>

namespace nuslam
{

EKF::EKF(arma::mat Q_robot, arma::mat R)
: Q_robot_{Q_robot},
  R_{R},
  state_{arma::zeros(3)},
  P_{arma::zeros(3, 3)}
{}

arma::mat EKF::build_Q() const
{
  const auto n = num_landmarks();
  const auto size = 3 + 2 * n;
  arma::mat Q_Bar = amra::zeros(size, size);
  Q_bar.submat(0, 0, 2, 2) = Q_robot_;
  return Q_bar;
}

arma::mat EKF::state_transition_jacobian(turtlelib::Twist2D twist) const
{
  const auto n = num_landmarks();
  const auto theta = state_(0);
  const auto dx = twist.x;
  const auto dtheta = twist.omega;
  const auto size = 3 + 2 * n;
  arma::mat A = arma::eye<arma::mat>(size, size);
  if (dtheta == 0) 
  {
    A(1, 0) = -dx * std::sin(theta);
    A(2, 0) =  dx * std::cos(theta);
  }
  else 
  {
    A(1, 0) = -(dx / dtheta) * std::cos(theta) + (dx / dtheta) * std::cos(theta + dtheta);
    A(2, 0) = -(dx / dtheta) * std::sin(theta) + (dx / dtheta) * std::sin(theta + dtheta);
  }
  return A;
}

void EKF::predict(turtlelib::Twist2D twist)
{
  const auto theta = state_(0);
  const auto dx = twist.x;
  const auto dtheta = twist.omega;

  arma::mat A = state_transition_jacobian(twist);

  if (dtheta == 0)
  {
    state_(0) += dx * std::cos(theta)
  }
}

std::size_t EKF::num_landmarks() const
{
  return landmark_indices_.size();
}

void EKF::initialize_landmark(int id, double r, double phi)
{
  const std::size_t j = landmark_indices_.size();
  landmark_indices_[id] = j;

  // Invert the measurement model to get map-frame position (eqs. 23-24)
  const double theta = state_(0);
  const double x     = state_(1);
  const double y     = state_(2);
  const double mx    = x + r * std::cos(phi + theta);
  const double my    = y + r * std::sin(phi + theta);

  // Grow the state vector by two elements
  state_.resize(state_.n_elem + 2);
  state_(3 + 2 * j)     = mx;
  state_(3 + 2 * j + 1) = my;

  // Grow the covariance matrix, keeping the existing block intact.
  // The new landmark's diagonal is set large (high uncertainty).
  const std::size_t old_size = 3 + 2 * j;
  const std::size_t new_size = old_size + 2;
  arma::mat P_new = arma::zeros(new_size, new_size);
  P_new.submat(0, 0, old_size - 1, old_size - 1) = P_;
  P_new(old_size,     old_size)     = 1e6;
  P_new(old_size + 1, old_size + 1) = 1e6;
  P_ = P_new;
}

}  // namespace nuslam
