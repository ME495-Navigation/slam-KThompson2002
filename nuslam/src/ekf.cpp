/// \file
/// \brief EKF-SLAM implementation for a differential-drive robot.

#include "nuslam/ekf.hpp"
#include "turtlelib/angle.hpp"

#include <cmath>
#include <stdexcept>

namespace nuslam
{

EKF::EKF(arma::mat Q_robot, arma::mat R)
: state_{arma::zeros(3)},
  P_{arma::zeros(3, 3)},
  Q_robot_{Q_robot},
  R_{R}
{}

arma::mat EKF::build_Q() const
{
  const auto n = num_landmarks();
  const auto size = 3 + 2 * n;
  arma::mat Q_bar = arma::zeros(size, size);
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
  constexpr double eps = 1e-9;
  if (std::abs(dtheta) < eps) {
    A(1, 0) = -dx * std::sin(theta);
    A(2, 0) = dx * std::cos(theta);
  } else {
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

  constexpr double eps = 1e-9;
  if (std::abs(dtheta) < eps) {
    state_(1) += dx * std::cos(theta);
    state_(2) += dx * std::sin(theta);
  } else {
    state_(0) += dtheta;
    state_(1) += -(dx / dtheta) * std::sin(theta) + (dx / dtheta) * std::sin(theta + dtheta);
    state_(2) += (dx / dtheta) * std::cos(theta) - (dx / dtheta) * std::cos(theta + dtheta);
  }

  state_(0) = turtlelib::normalize_angle(state_(0));

  P_ = A * P_ * A.t() + build_Q();
}

void EKF::update(int id, double r, double phi)
{
  if (landmark_indices_.count(id) == 0) {
    initialize_landmark(id, r, phi);
  }

  const auto j = landmark_indices_[id];
  const arma::vec z = {r, phi};
  const auto z_hat = predicted_measurement(j);
  const auto H = measurement_jacobian(j);

  const arma::mat S = H * P_ * H.t() + R_;
  const arma::mat K = P_ * H.t() * S.i();

  arma::vec dz = z - z_hat;
  dz(1) = turtlelib::normalize_angle(dz(1));

  state_ += K * dz;
  state_(0) = turtlelib::normalize_angle(state_(0));
  const auto size = state_.n_elem;
  P_ = (arma::eye(size, size) - K * H) * P_;
}

std::size_t EKF::num_landmarks() const
{
  return landmark_indices_.size();
}

arma::mat EKF::covariance() const
{
  return P_;
}

void EKF::initialize_landmark(int id, double r, double phi)
{
  const std::size_t j = landmark_indices_.size();
  landmark_indices_[id] = j;

  // Invert the measurement model to get map-frame position (eqs. 23-24)
  const auto theta = state_(0);
  const auto x = state_(1);
  const auto y = state_(2);
  const auto mx = x + r * std::cos(phi + theta);
  const auto
    my = y + r * std::sin(phi + theta);

  // Grow the state vector by two elements
  state_.resize(state_.n_elem + 2);
  state_(3 + 2 * j) = mx;
  state_(3 + 2 * j + 1) = my;

  // Grow the covariance matrix, keeping the existing block intact.
  // The new landmark's diagonal is set large (high uncertainty).
  const std::size_t old_size = 3 + 2 * j;
  const std::size_t new_size = old_size + 2;
  arma::mat P_new = arma::zeros(new_size, new_size);
  P_new.submat(0, 0, old_size - 1, old_size - 1) = P_;
  P_new(old_size, old_size) = 1e6;
  P_new(old_size + 1, old_size + 1) = 1e6;
  P_ = P_new;
}

arma::vec EKF::predicted_measurement(std::size_t map_idx) const
{
  const auto theta = state_(0);
  const auto x = state_(1);
  const auto y = state_(2);

  const auto mx = state_(3 + 2 * map_idx);
  const auto my = state_(3 + 2 * map_idx + 1);

  const auto dx = mx - x;
  const auto dy = my - y;
  const auto d = dx * dx + dy * dy;
  auto r_hat = std::sqrt(d);
  auto phi_hat = turtlelib::normalize_angle(std::atan2(dy, dx) - theta);

  arma::vec pred = {r_hat, phi_hat};
  return pred;
}

arma::mat EKF::measurement_jacobian(std::size_t map_idx) const
{
  const auto x = state_(1);
  const auto y = state_(2);

  const auto mx = state_(3 + 2 * map_idx);
  const auto my = state_(3 + 2 * map_idx + 1);

  const auto dx = mx - x;
  const auto dy = my - y;
  const auto d = dx * dx + dy * dy;

  const auto size = 3 + 2 * num_landmarks();
  arma::mat H = arma::zeros(2, size);

  H(0, 0) = 0.0;
  H(0, 1) = -dx / std::sqrt(d);
  H(0, 2) = -dy / std::sqrt(d);
  H(1, 0) = -1.0;
  H(1, 1) = dy / d;
  H(1, 2) = -dx / d;

  H(0, 3 + 2 * map_idx) = dx / std::sqrt(d);
  H(0, 3 + 2 * map_idx + 1) = dy / std::sqrt(d);
  H(1, 3 + 2 * map_idx) = -dy / d;
  H(1, 3 + 2 * map_idx + 1) = dx / d;

  return H;
}


turtlelib::Transform2D EKF::pose() const
{
  return turtlelib::Transform2D{{state_(1), state_(2)}, state_(0)};
}

turtlelib::Vector2D EKF::landmark(int id) const
{
  const auto it = landmark_indices_.find(id);
  if (it == landmark_indices_.end()) {
    throw std::out_of_range("EKF::landmark: id has never been observed");
  }
  const auto j = it->second;
  return {state_(3 + 2 * j), state_(3 + 2 * j + 1)};
}

}  // namespace nuslam
