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

// Deep clone without the upstream custom-node copy constructor slicing children.
std::unique_ptr<rtabmap::RtabmapColorOcTree> clone_rtabmap_tree(
  const rtabmap::RtabmapColorOcTree & source);

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

  // Retain RTAB-Map's local-grid cache and tree between updates. New nodes are
  // inserted incrementally; GlobalMap rebuilds from the cache only when an
  // existing pose moves farther than pose_update_tolerance_m.
  bool update(
    const std::vector<LocalGridObservation> & observations,
    const std::map<int, rtabmap::Transform> & optimized_poses,
    float pose_update_tolerance_m,
    float pose_update_tolerance_rad,
    std::string * error = nullptr);

  [[nodiscard]] const rtabmap::RtabmapColorOcTree * tree() const;
  [[nodiscard]] const OctomapBuildMetadata & metadata() const {return metadata_;}

  // Keep one exact map checkpoint so a transient newest node can be removed
  // by restoring the stable prefix and replaying only newer observations.
  // This is enabled by default and can be disabled for A/B replay.
  void set_use_checkpoints(bool enabled);
  [[nodiscard]] std::size_t checkpoint_restores() const {return checkpoint_restores_;}
  [[nodiscard]] std::size_t full_rebuilds() const {return full_rebuilds_;}
  [[nodiscard]] std::size_t cached_nodes() const {return cache_.size();}

  [[nodiscard]] static std::uint64_t pose_generation(
    const std::map<int, rtabmap::Transform> & optimized_poses);

private:
  bool refresh_metadata(
    const std::map<int, rtabmap::Transform> & optimized_poses,
    std::string * error);

  void invalidate_checkpoint();
  bool capture_checkpoint();
  bool can_restore_checkpoint(
    const std::map<int, rtabmap::Transform> & optimized_poses) const;

  rtabmap::LocalGridCache cache_;
  std::unique_ptr<rtabmap::OctoMap> octomap_;
  OctomapBuildMetadata metadata_;
  float resolution_{};
  float pose_update_tolerance_m_{};
  float pose_update_tolerance_rad_{};
  std::unique_ptr<rtabmap::RtabmapColorOcTree> checkpoint_tree_;
  std::map<int, rtabmap::Transform> checkpoint_poses_;
  double checkpoint_min_[3]{};
  double checkpoint_max_[3]{};
  bool checkpoint_valid_{false};
  bool use_checkpoints_{true};
  std::size_t checkpoint_restores_{};
  std::size_t full_rebuilds_{};
};

}  // namespace px4_vio_bridge
