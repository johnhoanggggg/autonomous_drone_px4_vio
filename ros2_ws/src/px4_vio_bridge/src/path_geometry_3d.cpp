#include "px4_vio_bridge/path_geometry_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace px4_vio_bridge
{

double distance_3d(const Point3 & first, const Point3 & second)
{
  return std::sqrt(
    (first.x - second.x) * (first.x - second.x) +
    (first.y - second.y) * (first.y - second.y) +
    (first.z - second.z) * (first.z - second.z));
}

Point3 subtract_3d(const Point3 & first, const Point3 & second)
{
  return {first.x - second.x, first.y - second.y, first.z - second.z};
}

Point3 add_3d(const Point3 & first, const Point3 & second)
{
  return {first.x + second.x, first.y + second.y, first.z + second.z};
}

Point3 scale_3d(const Point3 & point, double scale)
{
  return {point.x * scale, point.y * scale, point.z * scale};
}

Polyline3D::Polyline3D(const std::vector<Point3> & points)
{
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      throw std::invalid_argument("3D path contains non-finite point");
    }
    if (points_.empty() || distance_3d(points_.back(), point) > 1.0e-9) {
      points_.push_back(point);
    }
  }
  if (points_.size() < 2) {
    throw std::invalid_argument("3D path requires two distinct points");
  }
  cumulative_.reserve(points_.size());
  cumulative_.push_back(0.0);
  for (std::size_t index = 1; index < points_.size(); ++index) {
    cumulative_.push_back(
      cumulative_.back() + distance_3d(points_[index - 1], points_[index]));
  }
}

Projection3D Polyline3D::project(const Point3 & point) const
{
  Projection3D best;
  best.distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index + 1 < points_.size(); ++index) {
    const auto delta = subtract_3d(points_[index + 1], points_[index]);
    const auto relative = subtract_3d(point, points_[index]);
    const auto length_squared = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
    const auto fraction = std::clamp(
      (relative.x * delta.x + relative.y * delta.y + relative.z * delta.z) /
      length_squared, 0.0, 1.0);
    const auto projected = add_3d(points_[index], scale_3d(delta, fraction));
    const auto offset = subtract_3d(point, projected);
    const auto candidate_distance = distance_3d(point, projected);
    const auto candidate_along = cumulative_[index] +
      fraction * (cumulative_[index + 1] - cumulative_[index]);
    if (candidate_distance < best.distance - 1.0e-12 ||
      (std::abs(candidate_distance - best.distance) <= 1.0e-12 &&
      candidate_along > best.along))
    {
      best.point = projected;
      best.along = candidate_along;
      best.distance = candidate_distance;
      best.horizontal_distance = std::hypot(offset.x, offset.y);
      best.vertical_distance = std::abs(offset.z);
      best.segment = index;
    }
  }
  return best;
}

Point3 Polyline3D::point_at(double along) const
{
  const auto clamped = std::clamp(along, 0.0, length());
  const auto upper = std::upper_bound(cumulative_.begin(), cumulative_.end(), clamped);
  if (upper == cumulative_.begin()) {return points_.front();}
  if (upper == cumulative_.end()) {return points_.back();}
  const auto index = static_cast<std::size_t>(upper - cumulative_.begin() - 1);
  const auto segment_length = cumulative_[index + 1] - cumulative_[index];
  const auto fraction = (clamped - cumulative_[index]) / segment_length;
  return add_3d(
    points_[index], scale_3d(subtract_3d(points_[index + 1], points_[index]), fraction));
}

}  // namespace px4_vio_bridge
