#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <rtabmap/core/LocalGrid.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/global_map/OctoMap.h>

namespace px4_vio_bridge
{

struct LocalGridObservation
{
  int node_id{};
  cv::Mat ground;
  cv::Mat obstacles;
  cv::Mat empty;
  float cell_size{};
  cv::Point3f view_point{};
};

struct OctomapBuildMetadata
{
  std::uint64_t source_pose_generation{};
  std::size_t source_nodes{};
  std::size_t ground_cells{};
  std::size_t obstacle_cells{};
  std::size_t empty_cells{};
  double resolution{};
  double min_x{};
  double min_y{};
  double min_z{};
  double max_x{};
  double max_y{};
  double max_z{};
};

// Deterministic RTAB-Map local-grid assembler. Every rebuild starts with an
// empty cache, so optimized pose changes cannot leave stale voxels behind.
class RtabmapOctomapAssembler
{
public:
  RtabmapOctomapAssembler();

  bool rebuild(
    const std::vector<LocalGridObservation> & observations,
    const std::map<int, rtabmap::Transform> & optimized_poses,
    std::string * error = nullptr);

  [[nodiscard]] const rtabmap::RtabmapColorOcTree * tree() const;
  [[nodiscard]] const OctomapBuildMetadata & metadata() const {return metadata_;}

  [[nodiscard]] static std::uint64_t pose_generation(
    const std::map<int, rtabmap::Transform> & optimized_poses);

private:
  rtabmap::LocalGridCache cache_;
  std::unique_ptr<rtabmap::OctoMap> octomap_;
  OctomapBuildMetadata metadata_;
};

}  // namespace px4_vio_bridge
