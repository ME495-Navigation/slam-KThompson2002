#ifndef NUSLAM_CIRCLE_HPP
#define NUSLAM_CIRCLE_HPP
/// \file
/// \brief Landmark circle detection from 2D point clusters.
///
/// Pipeline:
///   1. cluster_points() — group raw Cartesian scan points by proximity
///   2. fit_circle()     — fit a circle to each cluster (Al-Sharadqah & Chernov 2009)
///   3. is_circle()      — classify whether the cluster is actually a circle (inscribed angle)
///
/// Reference: Al-Sharadqah & Chernov, "Error analysis for circle fitting algorithms",
///            Electronic Journal of Statistics, 2009.
///
/// The functions here are ROS-free. Convert sensor_msgs/LaserScan to a
/// std::vector<turtlelib::Point2D> (polar → Cartesian) before calling cluster_points().

#include <vector>
#include <armadillo>
#include <turtlelib>

#include "turtlelib/geometry2d.hpp"

namespace nuslam
{

/// \brief Parameters of a fitted circle in the sensor frame.
struct Circle
{
  double x;  ///< x-coordinate of circle centre (m)
  double y;  ///< y-coordinate of circle centre (m)
  double r;  ///< radius (m)
};

/// \brief An ordered list of 2D points belonging to a single cluster.
using Cluster = std::vector<turtlelib::Point2D>;

/// \brief Partition a flat list of Cartesian scan points into clusters.
/// \param points    Cartesian points in sensor frame, in scan order.
///                  Invalid ranges should already have been removed.
/// \param threshold Distance threshold (m) for splitting clusters (e.g. 0.1).
/// \return          Vector of clusters, each with ≥ 3 points.
std::vector<Cluster> cluster_points(
  const std::vector<turtlelib::Point2D> & points,
  double threshold);

/// \brief Fit a circle to a cluster using the Hyper algebraic fit.
/// \param cluster At least 3 points (caller should ensure this).
/// \return        Fitted Circle in the same frame as the input points.
Circle fit_circle(const Cluster & cluster);

/// \brief Classify a cluster as a circle (landmark) or non-circle (wall/noise).
/// \param cluster At least 3 points.
/// \return        true → treat as circle landmark; false → discard.
bool is_circle(const Cluster & cluster);

}  // namespace nuslam

#endif  // NUSLAM_CIRCLE_HPP
