#include "px4_vio_bridge/rtabmap_octomap.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include <rtabmap/core/Parameters.h>

namespace px4_vio_bridge
{
namespace
{

// The upstream node copy constructor recursively allocates base nodes, slicing
// custom occupancy/color/reference data. Allocate each child through the typed
// tree API instead, preserving topology and every node's state.
class ClonedColorTree final : public rtabmap::RtabmapColorOcTree
{
public:
  explicit ClonedColorTree(const rtabmap::RtabmapColorOcTree & source)
  : rtabmap::RtabmapColorOcTree(source.getResolution())
  {
    occ_prob_thres_log = source.getOccupancyThresLog();
    prob_hit_log = source.getProbHitLog();
    prob_miss_log = source.getProbMissLog();
    clamping_thres_min = source.getClampingThresMinLog();
    clamping_thres_max = source.getClampingThresMaxLog();
    if (source.getRoot() != nullptr) {
      root = new rtabmap::RtabmapColorOcTreeNode();
      tree_size = 1;
      copy_node(source, source.getRoot(), root);
    }
  }

private:
  void copy_node(
    const rtabmap::RtabmapColorOcTree & source,
    const rtabmap::RtabmapColorOcTreeNode * from,
    rtabmap::RtabmapColorOcTreeNode * to)
  {
    to->setLogOdds(from->getLogOdds());
    to->setColor(from->getColor());
    to->setNodeRefId(from->getNodeRefId());
    to->setOccupancyType(from->getOccupancyType());
    to->setPointRef(from->getPointRef());
    for (unsigned int child = 0; child < 8; ++child) {
      if (source.nodeChildExists(from, child)) {
        copy_node(source, source.getNodeChild(from, child), createNodeChild(to, child));
      }
    }
  }
};

// Restore protected GlobalMap bookkeeping alongside its owned tree.
class CheckpointOctoMap final : public rtabmap::OctoMap
{
public:
  using rtabmap::OctoMap::OctoMap;

  void restore(
    const rtabmap::RtabmapColorOcTree & snapshot,
    const std::map<int, rtabmap::Transform> & poses,
    const double min_values[3], const double max_values[3])
  {
    auto * live_tree = const_cast<rtabmap::RtabmapColorOcTree *>(octree());
    if (live_tree == nullptr) {
      throw std::runtime_error("RTAB-Map OctoMap has no owned tree");
    }

    // swapContent() swaps only root and tree_size. clear() first marks the
    // destination's cached metric bounds dirty and releases its old root.
    live_tree->clear();
    auto restored_tree = clone_rtabmap_tree(snapshot);
    live_tree->swapContent(*restored_tree);

    // Restore GlobalMap's private added-node set through its protected API,
    // then restore the bounds that OctoMap maintains separately from the tree.
    rtabmap::GlobalMap::clear();
    for (const auto & pose : poses) {
      addAssembledNode(pose.first, pose.second);
    }
    for (int axis = 0; axis < 3; ++axis) {
      minValues_[axis] = min_values[axis];
      maxValues_[axis] = max_values[axis];
    }
  }
};

std::size_t cell_count(const cv::Mat & cells)
{
  return cells.empty() ? 0U : cells.total();
}

cv::Mat canonical_cells(const cv::Mat & cells)
{
  if (cells.empty()) {
    return {};
  }
  if (cells.type() != CV_32FC3 && cells.type() != CV_32FC4) {
    throw std::invalid_argument(
            "3D local grid cells must use CV_32FC3 or CV_32FC4 (type=" +
            std::to_string(cells.type()) + ", channels=" +
            std::to_string(cells.channels()) + ", rows=" +
            std::to_string(cells.rows) + ", cols=" +
            std::to_string(cells.cols) + ")");
  }
  const auto continuous = cells.isContinuous() ? cells : cells.clone();
  if (continuous.type() == CV_32FC3) {
    return continuous.reshape(3, 1);
  }
  // ROS RTAB-Map retains a packed color/intensity channel in local grids
  // created from RGB-D data. RTAB-Map's OctoMap input contract is XYZ; discard
  // only that fourth visualization channel and preserve every observed cell.
  const auto colored = continuous.reshape(4, 1);
  cv::Mat xyz(1, static_cast<int>(colored.total()), CV_32FC3);
  for (int column = 0; column < colored.cols; ++column) {
    const auto point = colored.at<cv::Vec4f>(0, column);
    xyz.at<cv::Vec3f>(0, column) = {point[0], point[1], point[2]};
  }
  return xyz;
}

void fnv_bytes(std::uint64_t & hash, const void * data, std::size_t size)
{
  constexpr std::uint64_t prime = 1099511628211ULL;
  const auto * bytes = static_cast<const unsigned char *>(data);
  for (std::size_t index = 0; index < size; ++index) {
    hash ^= bytes[index];
    hash *= prime;
  }
}

}  // namespace

std::unique_ptr<rtabmap::RtabmapColorOcTree> clone_rtabmap_tree(
  const rtabmap::RtabmapColorOcTree & source)
{
  return std::make_unique<ClonedColorTree>(source);
}

RtabmapOctomapAssembler::RtabmapOctomapAssembler() = default;

void RtabmapOctomapAssembler::set_use_checkpoints(bool enabled)
{
  use_checkpoints_ = enabled;
  if (!enabled) {
    invalidate_checkpoint();
  }
}

void RtabmapOctomapAssembler::invalidate_checkpoint()
{
  checkpoint_tree_.reset();
  checkpoint_poses_.clear();
  checkpoint_valid_ = false;
  for (int axis = 0; axis < 3; ++axis) {
    checkpoint_min_[axis] = 0.0;
    checkpoint_max_[axis] = 0.0;
  }
}

bool RtabmapOctomapAssembler::capture_checkpoint()
{
  if (!use_checkpoints_ || octomap_ == nullptr || octomap_->octree() == nullptr ||
    octomap_->octree()->size() == 0 || octomap_->addedNodes().empty())
  {
    return false;
  }
  try {
    auto tree = clone_rtabmap_tree(*octomap_->octree());
    std::map<int, rtabmap::Transform> poses = octomap_->addedNodes();
    double min_values[3];
    double max_values[3];
    octomap_->getGridMin(min_values[0], min_values[1], min_values[2]);
    octomap_->getGridMax(max_values[0], max_values[1], max_values[2]);

    checkpoint_tree_ = std::move(tree);
    checkpoint_poses_ = std::move(poses);
    for (int axis = 0; axis < 3; ++axis) {
      checkpoint_min_[axis] = min_values[axis];
      checkpoint_max_[axis] = max_values[axis];
    }
    checkpoint_valid_ = true;
    return true;
  } catch (...) {
    // Do not leave a partially assigned tree/pose pair usable as a
    // checkpoint if allocation or topology validation failed.
    invalidate_checkpoint();
    return false;
  }
}

bool RtabmapOctomapAssembler::can_restore_checkpoint(
  const std::map<int, rtabmap::Transform> & optimized_poses) const
{
  if (!use_checkpoints_ || !checkpoint_valid_ || checkpoint_tree_ == nullptr ||
    checkpoint_poses_.empty())
  {
    return false;
  }

  // A checkpoint is exact: every stable-prefix pose must still be present at
  // the identical transform. This rejects changed or removed evidence.
  for (const auto & checkpoint_pose : checkpoint_poses_) {
    const auto current_pose = optimized_poses.find(checkpoint_pose.first);
    if (current_pose == optimized_poses.end() ||
      !(checkpoint_pose.second == current_pose->second))
    {
      return false;
    }
  }

  // Replaying a node older than the checkpoint can change the ray ordering
  // and node-reference precedence. Such LTM/non-monotonic input must use a
  // complete rebuild to retain RTAB-Map's exact semantics.
  const int checkpoint_last_id = checkpoint_poses_.rbegin()->first;
  for (const auto & cached : cache_.localGrids()) {
    if (optimized_poses.find(cached.first) == optimized_poses.end() ||
      checkpoint_poses_.find(cached.first) != checkpoint_poses_.end())
    {
      continue;
    }
    if (cached.first <= checkpoint_last_id) {
      return false;
    }
  }
  return true;
}

bool RtabmapOctomapAssembler::refresh_metadata(
  const std::map<int, rtabmap::Transform> & optimized_poses,
  std::string * error)
{
  if (octomap_ == nullptr || octomap_->octree() == nullptr || octomap_->octree()->size() == 0) {
    if (error != nullptr) {*error = "RTAB-Map produced an empty OctoMap";}
    return false;
  }
  metadata_ = {};
  for (const auto & item : cache_.localGrids()) {
    if (item.first <= 0 || optimized_poses.find(item.first) == optimized_poses.end()) {
      continue;
    }
    ++metadata_.source_nodes;
    metadata_.ground_cells += cell_count(item.second.groundCells);
    metadata_.obstacle_cells += cell_count(item.second.obstacleCells);
    metadata_.empty_cells += cell_count(item.second.emptyCells);
  }
  metadata_.source_pose_generation = pose_generation(optimized_poses);
  metadata_.resolution = octomap_->octree()->getResolution();
  octomap_->octree()->getMetricMin(
    metadata_.min_x, metadata_.min_y, metadata_.min_z);
  octomap_->octree()->getMetricMax(
    metadata_.max_x, metadata_.max_y, metadata_.max_z);
  return true;
}

bool RtabmapOctomapAssembler::rebuild(
  const std::vector<LocalGridObservation> & observations,
  const std::map<int, rtabmap::Transform> & optimized_poses,
  std::string * error)
{
  cache_.clear();
  octomap_.reset();
  invalidate_checkpoint();
  metadata_ = {};
  resolution_ = 0.0F;
  pose_update_tolerance_m_ = 0.0F;
  pose_update_tolerance_rad_ = 0.0F;
  float resolution = 0.0F;
  for (const auto & observation : observations) {
    if (observation.node_id <= 0 || optimized_poses.find(observation.node_id) == optimized_poses.end()) {
      continue;
    }
    if (!std::isfinite(observation.cell_size) || observation.cell_size <= 0.0F) {
      if (error != nullptr) {*error = "local grid has invalid cell size";}
      return false;
    }
    if (resolution == 0.0F) {
      resolution = observation.cell_size;
    } else if (std::abs(resolution - observation.cell_size) > 1.0e-6F) {
      if (error != nullptr) {*error = "local grids have inconsistent cell sizes";}
      return false;
    }
    if (observation.ground.empty() && observation.obstacles.empty() && observation.empty.empty()) {
      continue;
    }
    cv::Mat ground;
    cv::Mat obstacles;
    cv::Mat empty;
    try {
      ground = canonical_cells(observation.ground);
      obstacles = canonical_cells(observation.obstacles);
      empty = canonical_cells(observation.empty);
    } catch (const std::exception & exception) {
      if (error != nullptr) {*error = exception.what();}
      return false;
    }
    cache_.add(
      observation.node_id, ground, obstacles, empty,
      observation.cell_size, observation.view_point);
    ++metadata_.source_nodes;
    metadata_.ground_cells += cell_count(observation.ground);
    metadata_.obstacle_cells += cell_count(observation.obstacles);
    metadata_.empty_cells += cell_count(observation.empty);
  }
  if (cache_.empty() || resolution <= 0.0F) {
    if (error != nullptr) {*error = "no posed 3D local grids";}
    return false;
  }

  rtabmap::ParametersMap parameters;
  parameters.insert({rtabmap::Parameters::kGridCellSize(), std::to_string(resolution)});
  parameters.insert({rtabmap::Parameters::kGrid3D(), "true"});
  parameters.insert({rtabmap::Parameters::kGridGroundIsObstacle(), "true"});
  parameters.insert({rtabmap::Parameters::kGridRayTracing(), "true"});
  parameters.insert({rtabmap::Parameters::kGridGlobalUpdateError(), "0"});
  parameters.insert({rtabmap::Parameters::kGridGlobalFootprintRadius(), "0"});
  try {
    ++full_rebuilds_;
    octomap_ = std::make_unique<CheckpointOctoMap>(&cache_, parameters);
    if (!octomap_->update(optimized_poses) || octomap_->octree() == nullptr ||
      octomap_->octree()->size() == 0)
    {
      octomap_.reset();
      if (error != nullptr) {*error = "RTAB-Map produced an empty OctoMap";}
      return false;
    }
  } catch (const std::exception & exception) {
    octomap_.reset();
    if (error != nullptr) {*error = std::string("RTAB-Map OctoMap error: ") + exception.what();}
    return false;
  } catch (...) {
    octomap_.reset();
    if (error != nullptr) {*error = "RTAB-Map OctoMap error: unknown exception";}
    return false;
  }
  resolution_ = resolution;
  return refresh_metadata(optimized_poses, error);
}

bool RtabmapOctomapAssembler::update(
  const std::vector<LocalGridObservation> & observations,
  const std::map<int, rtabmap::Transform> & optimized_poses,
  float pose_update_tolerance_m,
  float pose_update_tolerance_rad,
  std::string * error)
{
  if (!std::isfinite(pose_update_tolerance_m) || pose_update_tolerance_m < 0.0F ||
    !std::isfinite(pose_update_tolerance_rad) || pose_update_tolerance_rad < 0.0F)
  {
    if (error != nullptr) {*error = "pose update tolerances must be finite and non-negative";}
    return false;
  }
  if (octomap_ != nullptr &&
    (std::abs(pose_update_tolerance_m_ - pose_update_tolerance_m) > 1.0e-6F ||
    std::abs(pose_update_tolerance_rad_ - pose_update_tolerance_rad) > 1.0e-6F))
  {
    if (error != nullptr) {*error = "pose update tolerances cannot change after initialization";}
    return false;
  }

  // Take the checkpoint before inserting the next observation. It therefore
  // represents the last complete prefix and never includes a node that this
  // update may immediately replace or remove.
  bool has_new_observation = false;
  for (const auto & observation : observations) {
    if (observation.node_id > 0 && cache_.find(observation.node_id) == cache_.end() &&
      !(observation.ground.empty() && observation.obstacles.empty() && observation.empty.empty()))
    {
      has_new_observation = true;
      break;
    }
  }
  if (has_new_observation && octomap_ != nullptr) {
    bool all_existing_poses_match = !octomap_->addedNodes().empty();
    for (const auto & assembled : octomap_->addedNodes()) {
      const auto current_pose = optimized_poses.find(assembled.first);
      if (current_pose == optimized_poses.end() || !(assembled.second == current_pose->second)) {
        all_existing_poses_match = false;
        break;
      }
    }
    if (all_existing_poses_match) {
      capture_checkpoint();
    }
  }

  for (const auto & observation : observations) {
    if (observation.node_id <= 0 ||
      cache_.find(observation.node_id) != cache_.end() ||
      (observation.ground.empty() && observation.obstacles.empty() && observation.empty.empty()))
    {
      continue;
    }
    if (!std::isfinite(observation.cell_size) || observation.cell_size <= 0.0F) {
      if (error != nullptr) {*error = "local grid has invalid cell size";}
      return false;
    }
    if (resolution_ == 0.0F) {
      resolution_ = observation.cell_size;
    } else if (std::abs(resolution_ - observation.cell_size) > 1.0e-6F) {
      if (error != nullptr) {*error = "local grids have inconsistent cell sizes";}
      return false;
    }
    try {
      cache_.add(
        observation.node_id,
        canonical_cells(observation.ground),
        canonical_cells(observation.obstacles),
        canonical_cells(observation.empty),
        observation.cell_size, observation.view_point);
    } catch (const std::exception & exception) {
      if (error != nullptr) {*error = exception.what();}
      return false;
    }
  }
  if (cache_.empty() || resolution_ <= 0.0F) {
    if (error != nullptr) {*error = "no posed 3D local grids";}
    return false;
  }

  // GlobalMap's built-in update threshold is translational. Explicitly detect
  // rotational corrections and removed nodes so loop optimization cannot
  // leave observations at stale orientations or from an old graph.
  bool rebuild_counted = false;
  if (octomap_ != nullptr) {
    bool graph_mismatch = false;
    for (const auto & assembled : octomap_->addedNodes()) {
      const auto corrected = optimized_poses.find(assembled.first);
      if (corrected == optimized_poses.end() ||
        assembled.second.getDistanceSquared(corrected->second) >
        pose_update_tolerance_m_ * pose_update_tolerance_m_ ||
        assembled.second.getQuaternionf().angularDistance(corrected->second.getQuaternionf()) >
        pose_update_tolerance_rad)
      {
        graph_mismatch = true;
        break;
      }
    }
    if (graph_mismatch) {
      if (can_restore_checkpoint(optimized_poses)) {
        try {
          auto * checkpoint_map = static_cast<CheckpointOctoMap *>(octomap_.get());
          checkpoint_map->restore(
            *checkpoint_tree_, checkpoint_poses_, checkpoint_min_, checkpoint_max_);
          ++checkpoint_restores_;
        } catch (...) {
          // restore() clears the owned tree before swapping the clone. If a
          // copy allocation/topology check fails, discard that partial map
          // and assemble the complete cache below.
          octomap_.reset();
          invalidate_checkpoint();
          ++full_rebuilds_;
          rebuild_counted = true;
        }
      } else {
        octomap_.reset();
        invalidate_checkpoint();
        ++full_rebuilds_;
        rebuild_counted = true;
      }
    }
  }
  bool constructed_map = false;
  if (octomap_ == nullptr) {
    rtabmap::ParametersMap parameters;
    parameters.insert({rtabmap::Parameters::kGridCellSize(), std::to_string(resolution_)});
    parameters.insert({rtabmap::Parameters::kGrid3D(), "true"});
    parameters.insert({rtabmap::Parameters::kGridGroundIsObstacle(), "true"});
    parameters.insert({rtabmap::Parameters::kGridRayTracing(), "true"});
    parameters.insert({
      rtabmap::Parameters::kGridGlobalUpdateError(),
      std::to_string(pose_update_tolerance_m)});
    parameters.insert({rtabmap::Parameters::kGridGlobalFootprintRadius(), "0"});
    if (!rebuild_counted) {
      // Initial construction is also a complete cache assembly. A fallback
      // reset above has already counted its discarded map.
      ++full_rebuilds_;
    }
    octomap_ = std::make_unique<CheckpointOctoMap>(&cache_, parameters);
    constructed_map = true;
    pose_update_tolerance_m_ = pose_update_tolerance_m;
    pose_update_tolerance_rad_ = pose_update_tolerance_rad;
  }
  try {
    if (constructed_map && use_checkpoints_) {
      // A previous incremental map may contain tolerated small pose changes,
      // so it cannot seed an exact checkpoint. On a full rebuild assemble the
      // current stable prefix first, then its newest observation. This also
      // establishes a checkpoint after startup or a loop-correction fallback.
      auto prefix = optimized_poses;
      int newest_cached = 0;
      std::size_t posed_grids = 0;
      for (const auto & cached : cache_.localGrids()) {
        if (cached.first > 0 && prefix.find(cached.first) != prefix.end()) {
          newest_cached = std::max(newest_cached, cached.first);
          ++posed_grids;
        }
      }
      if (posed_grids > 1) {
        prefix.erase(newest_cached);
        octomap_->update(prefix);
        capture_checkpoint();
      }
    }
    octomap_->update(optimized_poses);
  } catch (const std::exception & exception) {
    if (error != nullptr) {*error = std::string("RTAB-Map OctoMap error: ") + exception.what();}
    return false;
  } catch (...) {
    if (error != nullptr) {*error = "RTAB-Map OctoMap error: unknown exception";}
    return false;
  }
  return refresh_metadata(optimized_poses, error);
}

const rtabmap::RtabmapColorOcTree * RtabmapOctomapAssembler::tree() const
{
  return octomap_ == nullptr ? nullptr : octomap_->octree();
}

std::uint64_t RtabmapOctomapAssembler::pose_generation(
  const std::map<int, rtabmap::Transform> & optimized_poses)
{
  std::uint64_t hash = 1469598103934665603ULL;
  for (const auto & item : optimized_poses) {
    fnv_bytes(hash, &item.first, sizeof(item.first));
    const auto & matrix = item.second.dataMatrix();
    if (!matrix.empty()) {
      const auto bytes = matrix.total() * matrix.elemSize();
      fnv_bytes(hash, matrix.ptr(), bytes);
    }
  }
  return hash;
}

}  // namespace px4_vio_bridge
