#include "nuslam/circle.hpp"
#include <armadillo>
#include <cmath>
#include <numeric>

namespace
{
/// \brief Euclidean distance between two 2D points.
double euclidean(const turtlelib::Point2D & a, const turtlelib::Point2D & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::hypot(dx, dy);
}
}  // namespace

namespace nuslam
{

std::vector<Cluster> cluster_points(
  const std::vector<turtlelib::Point2D> & points,
  double threshold)
{
  if (points.empty()) {
    return {};
  }

  std::vector<Cluster> clusters{Cluster{points[0]}};

  for (size_t i = 1; i < points.size(); i++) {
    if (euclidean(points[i], points[i - 1]) <= threshold) {
      clusters.back().push_back(points[i]);
    } else {
      clusters.push_back(Cluster{points[i]});
    }
  }

  if (clusters.size() > 1 &&
    euclidean(points.back(), points.front()) <= threshold)
  {
    for (const auto & p : clusters.back()) {
      clusters.front().push_back(p);
    }
    clusters.pop_back();
  }

  std::vector<Cluster> result;
  for (const auto & c : clusters) {
    if (c.size() >= 4) {
      result.push_back(c);
    }
  }
  return result;
}

Circle fit_circle(const Cluster & cluster)
{
  const double n = static_cast<double>(cluster.size());

  const double x_bar = std::accumulate(
    cluster.begin(), cluster.end(), 0.0,
    [](double sum, const turtlelib::Point2D & p) {return sum + p.x;}) / n;

  const double y_bar = std::accumulate(
    cluster.begin(), cluster.end(), 0.0,
    [](double sum, const turtlelib::Point2D & p) {return sum + p.y;}) / n;

  arma::mat Z(cluster.size(), 4);
  for (size_t i = 0; i < cluster.size(); i++) {
    const auto u = cluster[i].x - x_bar;
    const auto v = cluster[i].y - y_bar;
    const auto z = u * u + v * v;
    Z(i, 0) = z;
    Z(i, 1) = u;
    Z(i, 2) = v;
    Z(i, 3) = 1.0;
  }

  const arma::mat M = (Z.t() * Z) / n;

  const auto z_bar = arma::mean(Z.col(0));

  arma::mat H = arma::zeros(4, 4);
  H(0, 0) = 8.0 * z_bar;
  H(0, 3) = 2.0;
  H(1, 1) = 1.0;
  H(2, 2) = 1.0;
  H(3, 0) = 2.0;

  arma::mat H_i = arma::zeros(4, 4);
  H_i(0, 3) = 0.5;
  H_i(1, 1) = 1.0;
  H_i(2, 2) = 1.0;
  H_i(3, 3) = -2.0 * z_bar;
  H_i(3, 0) = 0.5;

  arma::mat U, V;
  arma::vec s;
  arma::svd(U, s, V, Z);

  arma::vec A_vec;
  if (s(s.n_elem - 1) < 1e-12) {
    A_vec = V.col(3);
  } else {
    arma::mat Sigma = arma::zeros(4, 4);
    for (size_t j = 0; j < s.n_elem; j++) {
      Sigma(j, j) = s(j);
    }
    const arma::mat Y = V * Sigma * V.t();
    const arma::mat Q = Y * H_i * Y;

    arma::vec eigvals;
    arma::mat eigvecs;
    arma::eig_sym(eigvals, eigvecs, Q);

    for (size_t i = 0; i < eigvals.n_elem; i++) {
      if (eigvals(i) > 0.0) {
        A_vec = arma::pinv(Y) * eigvecs.col(i);
        break;
      }
    }
  }

  if (A_vec.is_empty()) {
    return Circle{0.0, 0.0, 0.0};
  }

  const auto a = -A_vec(1) / (2.0 * A_vec(0));
  const auto b = -A_vec(2) / (2.0 * A_vec(0));
  const auto R = std::sqrt((A_vec(1) * A_vec(1) + A_vec(2) * A_vec(2) - 4.0 * A_vec(0) * A_vec(3)) /
      (4.0 * A_vec(0) * A_vec(0)));

  return Circle{a + x_bar, b + y_bar, R};
}


bool is_circle(const Cluster & cluster, const double min_angle, const double max_angle)
{
  auto P1 = cluster.front();
  auto P2 = cluster.back();

  std::vector<double> angles{};
  for (size_t i = 1; i < cluster.size() - 1; i++) {
    auto angle_i = turtlelib::angle(P1 - cluster.at(i), P2 - cluster.at(i));
    angles.push_back(angle_i);
  }

  auto mean = std::accumulate(angles.begin(), angles.end(), 0.0) / angles.size();

  const auto variance = std::accumulate(
    angles.begin(), angles.end(), 0.0,
    [mean](double sum, double a) {return sum + (a - mean) * (a - mean);}
    ) / angles.size();

  const auto std_dev = std::sqrt(variance);

  return (std_dev < 0.15) && (mean > min_angle) && (mean < max_angle);
}

}
