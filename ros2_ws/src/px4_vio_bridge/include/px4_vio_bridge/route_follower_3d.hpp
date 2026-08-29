#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "px4_vio_bridge/path_geometry_3d.hpp"

namespace px4_vio_bridge
{

struct Follower3DConfig
{
  double lookahead{0.35};
  double max_horizontal_speed{0.10};
  double max_vertical_speed{0.05};
  double max_horizontal_acceleration{0.30};
  double max_vertical_acceleration{0.20};
  double max_cross_track{0.05};
  double max_vertical_track{0.05};
  double arrival_tolerance{0.10};
};

struct FollowResult3D
{
  bool valid{};
  bool reached{};
  Point3 carrot{};
  Point3 lookahead{};
  Point3 displacement{};
  Point3 velocity{};
  Point3 acceleration{};
  Projection3D projection{};
  double remaining{};
  std::string reason;
};

class RouteFollower3D
{
public:
  using ChordValidator = std::function<bool(const Point3 &, const Point3 &)>;

  explicit RouteFollower3D(Follower3DConfig config = {});
  void clear();
  bool set_path(const std::vector<Point3> & points, const Point3 & pose);
  [[nodiscard]] FollowResult3D update(
    const Point3 & pose, double dt, const ChordValidator & validator);

private:
  Follower3DConfig config_;
  std::unique_ptr<Polyline3D> path_;
  Point3 carrot_{};
  Point3 velocity_{};
  bool initialized_{};
};

}  // namespace px4_vio_bridge
