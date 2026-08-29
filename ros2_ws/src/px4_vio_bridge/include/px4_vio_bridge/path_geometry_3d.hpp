#pragma once

#include <cstddef>
#include <vector>

#include "px4_vio_bridge/voxel_grid.hpp"

namespace px4_vio_bridge
{

struct Projection3D
{
  Point3 point{};
  double along{};
  double distance{};
  double horizontal_distance{};
  double vertical_distance{};
  std::size_t segment{};
};

class Polyline3D
{
public:
  explicit Polyline3D(const std::vector<Point3> & points);

  [[nodiscard]] const std::vector<Point3> & points() const {return points_;}
  [[nodiscard]] double length() const {return cumulative_.back();}
  [[nodiscard]] Projection3D project(const Point3 & point) const;
  [[nodiscard]] Point3 point_at(double along) const;

private:
  std::vector<Point3> points_;
  std::vector<double> cumulative_;
};

[[nodiscard]] double distance_3d(const Point3 & first, const Point3 & second);
[[nodiscard]] Point3 subtract_3d(const Point3 & first, const Point3 & second);
[[nodiscard]] Point3 add_3d(const Point3 & first, const Point3 & second);
[[nodiscard]] Point3 scale_3d(const Point3 & point, double scale);

}  // namespace px4_vio_bridge
