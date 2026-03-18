#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <armadillo>
#include <cmath>
#include <numbers>
#include "nuslam/circle.hpp"

using Catch::Matchers::WithinAbs;
static constexpr double TOL = 1e-4;

TEST_CASE("Circle fit - test 1", "[circle]")
{
  const nuslam::Cluster cluster{
    {1.0, 7.0}, {2.0, 6.0}, {5.0, 8.0},
    {7.0, 7.0}, {9.0, 5.0}, {3.0, 7.0}
  };

  const nuslam::Circle c = nuslam::fit_circle(cluster);

  CHECK_THAT(c.x, WithinAbs(4.615482, TOL));
  CHECK_THAT(c.y, WithinAbs(2.807354, TOL));
  CHECK_THAT(c.r, WithinAbs(4.8275, TOL));
}

TEST_CASE("Circle fit - test 2", "[circle]")
{
  const nuslam::Cluster cluster{
    {-1.0, 0.0}, {-0.3, -0.06}, {0.3, 0.1}, {1.0, 0.0}
  };

  const nuslam::Circle c = nuslam::fit_circle(cluster);

  CHECK_THAT(c.x, WithinAbs(0.4908357, TOL));
  CHECK_THAT(c.y, WithinAbs(-22.15212, TOL));
  CHECK_THAT(c.r, WithinAbs(22.17979, TOL));
}

TEST_CASE("Circle classification - semicircle is a circle", "[circle]")
{
  // 5 points on a semicircle (radius 1, centre origin).
  // By Thales' theorem every interior inscribed angle = pi/2,
  // so mean = pi/2, std_dev = 0 — passes [pi/2, 3pi/4].
  const nuslam::Cluster cluster{
    {1.0, 0.0}, {0.7071, 0.7071}, {0.0, 1.0}, {-0.7071, 0.7071}, {-1.0, 0.0}
  };

  CHECK(nuslam::is_circle(cluster, std::numbers::pi / 2.0, 3.0 * std::numbers::pi / 4.0));
}

TEST_CASE("Circle classification - collinear points are not a circle", "[circle]")
{
  // Evenly spaced points on a straight line.
  // Every interior inscribed angle = pi (180 deg) — mean > 3pi/4, fails.
  const nuslam::Cluster cluster{
    {-1.0, 0.0}, {-0.5, 0.0}, {0.0, 0.0}, {0.5, 0.0}, {1.0, 0.0}
  };

  CHECK_FALSE(nuslam::is_circle(cluster, std::numbers::pi / 2.0, 3.0 * std::numbers::pi / 4.0));
}
