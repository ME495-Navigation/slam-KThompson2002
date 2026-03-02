#ifndef NUSLAM_EKF_HPP
#define NUSLAM_EKF_HPP
/// \file
/// \brief Extended Kalman Filter for EKF-SLAM on a differential-drive robot.
///
/// State vector layout: [ theta, x, y | mx_0, my_0, mx_1, my_1, ... ]
///   - Index 0:         robot heading (theta)
///   - Indices 1-2:     robot position (x, y)
///   - Indices 3+2j, 3+2j+1: position of the j-th landmark (in map frame)
///
/// The state grows dynamically as new landmarks are encountered.

#include <armadillo>
#include <unordered_map>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace nuslam
{

class EKF
{
public:
  /// \brief Construct an EKF with known noise parameters.
  /// \param Q_robot 3x3 process noise covariance for the robot pose.
  ///        Applied only to the pose block; landmark entries are noise-free
  ///        in the motion model.
  /// \param R 2x2 measurement noise covariance (range, bearing).
  EKF(arma::mat Q_robot, arma::mat R);

  /// \brief Prediction step: propagate state and covariance through the motion model.
  /// \param twist Body-frame displacement for this timestep, as returned by
  ///        turtlelib::DiffDrive::forwardKinematics().
  ///        twist.omega is the angular displacement (rad);
  ///        twist.x is the forward displacement (m).
  void predict(turtlelib::Twist2D twist);

  /// \brief Update step: incorporate one landmark measurement into the state.
  ///        If the landmark has not been seen before it is initialized and added
  ///        to the state vector before the update is applied.
  /// \param id   Unique integer identifier for the landmark.
  /// \param r    Measured range to the landmark (m), in the robot frame.
  /// \param phi  Measured bearing to the landmark (rad), in the robot frame.
  void update(int id, double r, double phi);

  /// \brief Return the current robot pose estimate.
  turtlelib::Transform2D pose() const;

  /// \brief Return the current estimated position of a landmark in the map frame.
  /// \param id Landmark identifier (must have been seen at least once).
  /// \throws std::out_of_range if id has never been observed.
  turtlelib::Vector2D landmark(int id) const;

  /// \brief Return the number of landmarks currently tracked.
  std::size_t num_landmarks() const;

  /// \brief Return the current state covariance matrix, size (3+2N) x (3+2N).
  arma::mat covariance() const;

private:
  /// \brief Add a new landmark to the state vector and covariance matrix.
  ///        Converts the first measurement (r, phi) into a map-frame position
  ///        using the current robot pose estimate.
  /// \param id  Landmark identifier.
  /// \param r   First range measurement (m).
  /// \param phi First bearing measurement (rad).
  void initialize_landmark(int id, double r, double phi);

  /// \brief Compute the predicted measurement h(x) for the j-th landmark.
  /// \param map_idx Zero-based index into the landmark portion of the state,
  ///        i.e. the same value stored in landmark_indices_.
  /// \return 2-vector [r_hat, phi_hat].
  arma::vec predicted_measurement(std::size_t map_idx) const;

  /// \brief Compute the 2 x (3 + 2N) measurement Jacobian H for the j-th landmark.
  /// \param map_idx Zero-based landmark index.
  /// \return H matrix with non-zero columns only at the pose block and the
  ///         landmark's two state entries.
  arma::mat measurement_jacobian(std::size_t map_idx) const;

  /// \brief Compute the (3+2N) x (3+2N) state transition Jacobian A_t.
  ///        Two cases: zero vs non-zero rotational velocity (eqs. 9-10).
  /// \param twist The same twist passed to predict().
  arma::mat state_transition_jacobian(turtlelib::Twist2D twist) const;

  /// \brief Build the full (3+2N) x (3+2N) process noise matrix.
  ///        The top-left 3x3 block is Q_robot_; all other entries are zero.
  arma::mat build_Q() const;

  /// Full state vector: [theta, x, y, mx_0, my_0, mx_1, my_1, ...]
  arma::vec state_;

  /// State covariance matrix, size (3+2N) x (3+2N).
  arma::mat P_;

  /// 3x3 process noise covariance for the robot pose block.
  arma::mat Q_robot_;

  /// 2x2 measurement noise covariance [range, bearing].
  arma::mat R_;

  /// Maps a landmark's external id to its zero-based column index j.
  /// The landmark's state entries sit at indices 3+2j and 3+2j+1 in state_.
  std::unordered_map<int, std::size_t> landmark_indices_;
};

}  // namespace nuslam

#endif  // NUSLAM_EKF_HPP
