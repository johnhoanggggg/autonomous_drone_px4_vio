#include "px4_vio_bridge/rtabmap_octomap.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include <rtabmap/core/Parameters.h>

namespace px4_vio_bridge
{
namespace
{

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

RtabmapOctomapAssembler::RtabmapOctomapAssembler() = default;

bool RtabmapOctomapAssembler::rebuild(
  const std::vector<LocalGridObservation> & observations,
  const std::map<int, rtabmap::Transform> & optimized_poses,
  std::string * error)
{
  cache_.clear();
  octomap_.reset();
  metadata_ = {};
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
    octomap_ = std::make_unique<rtabmap::OctoMap>(&cache_, parameters);
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
  metadata_.source_pose_generation = pose_generation(optimized_poses);
  metadata_.resolution = octomap_->octree()->getResolution();
  octomap_->octree()->getMetricMin(
    metadata_.min_x, metadata_.min_y, metadata_.min_z);
  octomap_->octree()->getMetricMax(
    metadata_.max_x, metadata_.max_y, metadata_.max_z);
  return true;
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
