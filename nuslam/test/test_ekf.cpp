#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <armadillo>
#include <cmath>
#include <numbers>
#include "nuslam/ekf.hpp"

using nuslam::EKF;
using Catch::Matchers::WithinAbs;

static const arma::mat Q = arma::eye(3, 3) * 1e-3;
static const arma::mat R = arma::eye(2, 2) * 1e-2;

TEST_CASE("Initial state", "[ekf]")
{
  const EKF ekf{Q, R};

  SECTION("No landmarks at construction")
  {
    CHECK(ekf.num_landmarks() == 0);
  }

  SECTION("Pose is identity at construction")
  {
    const auto p = ekf.pose();
    CHECK_THAT(p.rotation(), WithinAbs(0.0, 1e-9));
    CHECK_THAT(p.translation().x, WithinAbs(0.0, 1e-9));
    CHECK_THAT(p.translation().y, WithinAbs(0.0, 1e-9));
  }

  SECTION("Covariance is zero at construction")
  {
    CHECK_THAT(arma::norm(ekf.covariance(), "fro"), WithinAbs(0.0, 1e-9));
  }
}

TEST_CASE("predict() - pure translation", "[ekf]")
{
  // Robot at origin, heading 0, moves 0.5 m forward.
  // Zero rotation branch (dtheta = 0):
  //   x_new = 0 + 0.5 * cos(0) = 0.5
  //   y_new = 0 + 0.5 * sin(0) = 0.0
  //   theta_new = 0
  EKF ekf{Q, R};
  ekf.predict({0.0, 0.5, 0.0});

  const auto p = ekf.pose();
  CHECK_THAT(p.rotation(), WithinAbs(0.0, 1e-9));
  CHECK_THAT(p.translation().x, WithinAbs(0.5, 1e-9));
  CHECK_THAT(p.translation().y, WithinAbs(0.0, 1e-9));
}

TEST_CASE("predict() - pure rotation", "[ekf]")
{
  // Robot at origin, rotates pi/4 in place (dx = 0).
  // x and y are unchanged regardless of dtheta.
  EKF ekf{Q, R};
  ekf.predict({std::numbers::pi / 4.0, 0.0, 0.0});

  const auto p = ekf.pose();
  CHECK_THAT(p.rotation(), WithinAbs(std::numbers::pi / 4.0, 1e-9));
  CHECK_THAT(p.translation().x, WithinAbs(0.0, 1e-9));
  CHECK_THAT(p.translation().y, WithinAbs(0.0, 1e-9));
}

TEST_CASE("predict() - arc motion", "[ekf]")
{
  // dtheta = pi/2, dx = pi/2  =>  dx/dtheta = 1
  // Starting at (theta=0, x=0, y=0):
  //   x_new = -(1)*sin(0)   + (1)*sin(pi/2) = 1
  //   y_new =  (1)*cos(0)   - (1)*cos(pi/2) = 1
  //   theta_new = pi/2
  EKF ekf{Q, R};
  ekf.predict({std::numbers::pi / 2.0, std::numbers::pi / 2.0, 0.0});

  const auto p = ekf.pose();
  CHECK_THAT(p.rotation(), WithinAbs(std::numbers::pi / 2.0, 1e-9));
  CHECK_THAT(p.translation().x, WithinAbs(1.0, 1e-9));
  CHECK_THAT(p.translation().y, WithinAbs(1.0, 1e-9));
}

TEST_CASE("predict() - covariance grows", "[ekf]")
{
  // Any non-trivial motion must increase the trace of P (uncertainty grows).
  // Initial P is zero, so any positive Q contribution makes trace(P) > 0.
  EKF ekf{Q, R};
  const double trace_before = arma::trace(ekf.covariance());
  ekf.predict({0.0, 0.1, 0.0});
  CHECK(arma::trace(ekf.covariance()) > trace_before);
}

TEST_CASE("update() - initialize a landmark")
{
  // Verify initialize_landmark inverts measurement model
  EKF ekf{Q, R};

  ekf.update(0, 1.0, 0.0);
  CHECK(ekf.num_landmarks() == 1);
  CHECK_THAT(ekf.landmark(0).x, WithinAbs(1.0, 1e-9));
  CHECK_THAT(ekf.landmark(0).y, WithinAbs(0.0, 1e-9));

  ekf.update(1, 1.0, std::numbers::pi / 2.0);
  CHECK(ekf.num_landmarks() == 2);
  CHECK_THAT(ekf.landmark(1).x, WithinAbs(0.0, 1e-9));
  CHECK_THAT(ekf.landmark(1).y, WithinAbs(1.0, 1e-9));
}

TEST_CASE("update() - covariance shrinks after observation")
{
  // After a predict(), P has grown. An update() must reduce the trace of P
  // because incorporating a measurement always reduces uncertainty.
  EKF ekf{Q, R};
  ekf.predict({0.0, 0.1, 0.0});
  ekf.update(0, 1.0, 0.0);

  const double trace_after_predict = arma::trace(ekf.covariance());

  ekf.update(0, 1.0, 0.0);
  CHECK(arma::trace(ekf.covariance()) < trace_after_predict);
}

TEST_CASE("update() - repeated observations converge")
{
  // True landmark at (2, 0). Robot at origin facing forward.
  // Perfect range-bearing measurements: r=2.0, phi=0.0.
  // After enough observations the landmark estimate should converge
  // tightly to (2, 0) and the robot pose should remain at the origin.
  EKF ekf{Q, R};

  for (int i = 0; i < 50; ++i) {
    ekf.update(0, 2.0, 0.0);
  }

  CHECK_THAT(ekf.landmark(0).x, WithinAbs(2.0, 1e-3));
  CHECK_THAT(ekf.landmark(0).y, WithinAbs(0.0, 1e-3));

  const auto p = ekf.pose();
  CHECK_THAT(p.translation().x, WithinAbs(0.0, 1e-3));
  CHECK_THAT(p.translation().y, WithinAbs(0.0, 1e-3));
  CHECK_THAT(p.rotation(), WithinAbs(0.0, 1e-3));
}

TEST_CASE("landmark() - unseen ID throws")
{
  EKF ekf{Q, R};
  CHECK_THROWS_AS(ekf.landmark(99), std::out_of_range);
}
