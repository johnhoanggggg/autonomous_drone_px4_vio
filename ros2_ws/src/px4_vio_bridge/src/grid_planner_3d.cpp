#include "px4_vio_bridge/grid_planner_3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>

namespace px4_vio_bridge
{
namespace
{

double distance(const Point3 & a, const Point3 & b)
{
  return std::sqrt(
    (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
    (a.z - b.z) * (a.z - b.z));
}

double voxel_distance(const Voxel & a, const Voxel & b)
{
  const auto dx = static_cast<double>(a.x - b.x);
  const auto dy = static_cast<double>(a.y - b.y);
  const auto dz = static_cast<double>(a.z - b.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Slab test against an axis-aligned box. Expanding the voxel box in XYZ by a
// sphere radius encloses the exact rounded Minkowski sum and is conservative.
bool segment_intersects_box(
  const Point3 & start, const Point3 & end, const Point3 & lower, const Point3 & upper)
{
  double t_min = 0.0;
  double t_max = 1.0;
  const std::array<double, 3> a{start.x, start.y, start.z};
  const std::array<double, 3> b{end.x, end.y, end.z};
  const std::array<double, 3> lo{lower.x, lower.y, lower.z};
  const std::array<double, 3> hi{upper.x, upper.y, upper.z};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const auto delta = b[axis] - a[axis];
    if (std::abs(delta) < 1.0e-15) {
      if (a[axis] < lo[axis] || a[axis] > hi[axis]) {
        return false;
      }
      continue;
    }
    auto near = (lo[axis] - a[axis]) / delta;
    auto far = (hi[axis] - a[axis]) / delta;
    if (near > far) {
      std::swap(near, far);
    }
    t_min = std::max(t_min, near);
    t_max = std::min(t_max, far);
    if (t_min > t_max) {
      return false;
    }
  }
  return true;
}

std::size_t flat_index(
  const Voxel & voxel, std::size_t width, std::size_t height)
{
  return (static_cast<std::size_t>(voxel.z) * height +
         static_cast<std::size_t>(voxel.y)) * width + static_cast<std::size_t>(voxel.x);
}

}  // namespace

CostVoxelGrid::CostVoxelGrid(const VoxelGrid & geometry)
: width_(geometry.width()), height_(geometry.height()), depth_(geometry.depth()),
  resolution_(geometry.resolution()), origin_(geometry.origin()),
  data_(geometry.size(), VOXEL_UNKNOWN_COST)
{
}

bool CostVoxelGrid::in_bounds(const Voxel & voxel) const
{
  return voxel.x >= 0 && voxel.y >= 0 && voxel.z >= 0 &&
         static_cast<std::size_t>(voxel.x) < width_ &&
         static_cast<std::size_t>(voxel.y) < height_ &&
         static_cast<std::size_t>(voxel.z) < depth_;
}

std::size_t CostVoxelGrid::index(const Voxel & voxel) const
{
  if (!in_bounds(voxel)) {
    throw std::out_of_range("voxel outside cost grid");
  }
  return flat_index(voxel, width_, height_);
}

std::int16_t CostVoxelGrid::at(const Voxel & voxel) const {return data_.at(index(voxel));}
void CostVoxelGrid::set(const Voxel & voxel, std::int16_t cost) {data_.at(index(voxel)) = cost;}

bool CostVoxelGrid::traversable(const Voxel & voxel) const
{
  if (!in_bounds(voxel)) {
    return false;
  }
  const auto cost = at(voxel);
  return cost >= 0 && cost < VOXEL_LETHAL_COST;
}

Point3 CostVoxelGrid::voxel_center(const Voxel & voxel) const
{
  if (!in_bounds(voxel)) {
    throw std::out_of_range("voxel outside cost grid");
  }
  return {
    origin_.x + (static_cast<double>(voxel.x) + 0.5) * resolution_,
    origin_.y + (static_cast<double>(voxel.y) + 0.5) * resolution_,
    origin_.z + (static_cast<double>(voxel.z) + 0.5) * resolution_};
}

std::optional<Voxel> CostVoxelGrid::world_to_voxel(const Point3 & point) const
{
  Voxel voxel{
    static_cast<int>(std::floor((point.x - origin_.x) / resolution_)),
    static_cast<int>(std::floor((point.y - origin_.y) / resolution_)),
    static_cast<int>(std::floor((point.z - origin_.z) / resolution_))};
  return in_bounds(voxel) ? std::optional<Voxel>(voxel) : std::nullopt;
}

CostVoxelGrid inflate_voxels(
  const VoxelGrid & grid, double lethal_radius, double inflation_radius,
  double cost_scaling)
{
  if (!std::isfinite(lethal_radius) || lethal_radius < 0.0 ||
    !std::isfinite(inflation_radius) || inflation_radius < lethal_radius ||
    !std::isfinite(cost_scaling) || cost_scaling < 0.0)
  {
    throw std::invalid_argument("invalid 3D inflation parameters");
  }
  CostVoxelGrid result(grid);
  // Multi-source 26-neighbour distance transform. Chebyshev distance is a
  // lower bound on Euclidean distance, so using it for clearance is
  // conservative. Crucially, unknown is a distance source as well as occupied:
  // a spherical envelope may not overlap an unobserved voxel.
  const auto unset = std::numeric_limits<int>::max();
  std::vector<int> distance_steps(grid.size(), unset);
  std::deque<Voxel> queue;
  for (int z = 0; z < static_cast<int>(grid.depth()); ++z) {
    for (int y = 0; y < static_cast<int>(grid.height()); ++y) {
      for (int x = 0; x < static_cast<int>(grid.width()); ++x) {
        const Voxel voxel{x, y, z};
        if (grid.at(voxel) != VoxelState::Free) {
          distance_steps[grid.index(voxel)] = 0;
          queue.push_back(voxel);
        }
      }
    }
  }
  const auto max_steps = static_cast<int>(
    std::ceil((inflation_radius + std::sqrt(3.0) * grid.resolution()) /
    grid.resolution()));
  while (!queue.empty()) {
    const auto current = queue.front();
    queue.pop_front();
    const auto next_distance = distance_steps[grid.index(current)] + 1;
    if (next_distance > max_steps) {continue;}
    for (int dz = -1; dz <= 1; ++dz) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0 && dz == 0) {continue;}
          const Voxel next{current.x + dx, current.y + dy, current.z + dz};
          if (!grid.in_bounds(next) || distance_steps[grid.index(next)] <= next_distance) {
            continue;
          }
          distance_steps[grid.index(next)] = next_distance;
          queue.push_back(next);
        }
      }
    }
  }

  const auto half_diagonal = std::sqrt(3.0) * grid.resolution() * 0.5;
  const auto upper = grid.upper_bound();
  for (int z = 0; z < static_cast<int>(grid.depth()); ++z) {
    for (int y = 0; y < static_cast<int>(grid.height()); ++y) {
      for (int x = 0; x < static_cast<int>(grid.width()); ++x) {
        const Voxel voxel{x, y, z};
        const auto state = grid.at(voxel);
        if (state == VoxelState::Unknown) {result.set(voxel, VOXEL_UNKNOWN_COST); continue;}
        if (state == VoxelState::Occupied) {result.set(voxel, VOXEL_LETHAL_COST); continue;}
        const auto center = grid.voxel_center(voxel);
        const auto boundary_clearance = std::min({
            center.x - grid.origin().x, upper.x - center.x,
            center.y - grid.origin().y, upper.y - center.y,
            center.z - grid.origin().z, upper.z - center.z});
        double obstacle_clearance = std::numeric_limits<double>::infinity();
        const auto steps = distance_steps[grid.index(voxel)];
        if (steps != unset) {
          obstacle_clearance = std::max(
            0.0, static_cast<double>(steps) * grid.resolution() - half_diagonal);
        }
        const auto clearance = std::min(boundary_clearance, obstacle_clearance);
        if (clearance <= lethal_radius + 1.0e-12) {
          result.set(voxel, VOXEL_LETHAL_COST);
        } else if (clearance <= inflation_radius + 1.0e-12) {
          const auto span = std::max(grid.resolution(), inflation_radius - lethal_radius);
          const auto decay = std::exp(-cost_scaling * (clearance - lethal_radius) / span);
          result.set(voxel, static_cast<std::int16_t>(std::clamp(
              static_cast<int>(std::nearbyint((VOXEL_LETHAL_COST - 1) * decay)),
              1, static_cast<int>(VOXEL_LETHAL_COST - 1))));
        } else {
          result.set(voxel, 0);
        }
      }
    }
  }
  return result;
}

std::vector<std::pair<Voxel, double>> traversable_neighbors_3d(
  const CostVoxelGrid & grid, const Voxel & voxel)
{
  std::vector<std::pair<Voxel, double>> neighbors;
  neighbors.reserve(26);
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        const Voxel next{voxel.x + dx, voxel.y + dy, voxel.z + dz};
        if (!grid.traversable(next)) {
          continue;
        }
        const std::array<int, 3> deltas{dx, dy, dz};
        std::array<int, 3> changed{};
        int changed_count = 0;
        for (int axis = 0; axis < 3; ++axis) {
          if (deltas[axis] != 0) {
            changed[changed_count++] = axis;
          }
        }
        bool clear = true;
        // Every proper non-empty subset is a face/edge voxel crossed by this
        // diagonal. Requiring all of them is deliberately conservative.
        for (int mask = 1; mask < (1 << changed_count) - 1 && clear; ++mask) {
          Voxel intermediate = voxel;
          for (int bit = 0; bit < changed_count; ++bit) {
            if ((mask & (1 << bit)) == 0) {
              continue;
            }
            if (changed[bit] == 0) {intermediate.x += dx;}
            if (changed[bit] == 1) {intermediate.y += dy;}
            if (changed[bit] == 2) {intermediate.z += dz;}
          }
          clear = grid.traversable(intermediate);
        }
        if (clear) {
          neighbors.emplace_back(next, std::sqrt(static_cast<double>(changed_count)));
        }
      }
    }
  }
  return neighbors;
}

std::optional<Voxel> recover_start_3d(
  const VoxelGrid & raw, const CostVoxelGrid & inflated, const Voxel & start,
  double max_radius)
{
  if (!raw.in_bounds(start) || raw.at(start) != VoxelState::Free) {
    return std::nullopt;
  }
  if (inflated.traversable(start)) {
    return start;
  }
  const auto limit = static_cast<int>(std::floor(max_radius / raw.resolution()));
  if (limit < 1) {
    return std::nullopt;
  }
  std::deque<Voxel> queue{start};
  std::vector<std::uint8_t> visited(raw.size(), 0);
  visited[raw.index(start)] = 1;
  while (!queue.empty()) {
    const auto current = queue.front();
    queue.pop_front();
    const auto current_cost = inflated.at(current);
    for (int dz = -1; dz <= 1; ++dz) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0 && dz == 0) {continue;}
          Voxel next{current.x + dx, current.y + dy, current.z + dz};
          if (!raw.in_bounds(next) || visited[raw.index(next)] ||
            raw.at(next) != VoxelState::Free || voxel_distance(start, next) > limit + 1.0e-12)
          {
            continue;
          }
          // Inflation cost is monotonic with obstacle clearance. Never permit
          // a recovery step that becomes less safe or slides on a plateau.
          if (inflated.at(next) >= current_cost) {
            continue;
          }
          visited[raw.index(next)] = 1;
          if (inflated.traversable(next)) {
            return next;
          }
          queue.push_back(next);
        }
      }
    }
  }
  return std::nullopt;
}

std::optional<GoalSelection3D> closest_reachable_goal_3d(
  const VoxelGrid & raw, const CostVoxelGrid & inflated, const Voxel & start,
  const Point3 & requested, double timeout_ms)
{
  if (!inflated.traversable(start)) {
    return std::nullopt;
  }
  const auto requested_voxel = raw.world_to_voxel(requested);
  const bool known_blocked = requested_voxel && raw.at(*requested_voxel) == VoxelState::Occupied;
  const bool requested_exact = requested_voxel && inflated.traversable(*requested_voxel);
  std::deque<Voxel> queue{start};
  std::vector<std::uint8_t> visited(raw.size(), 0);
  visited[raw.index(start)] = 1;
  Voxel best = start;
  double best_distance = distance(raw.voxel_center(start), requested);
  std::size_t reachable = 0;
  const auto begin = std::chrono::steady_clock::now();
  while (!queue.empty()) {
    if (std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count() > timeout_ms)
    {
      break;
    }
    const auto current = queue.front();
    queue.pop_front();
    ++reachable;
    const auto current_distance = distance(raw.voxel_center(current), requested);
    if (current_distance < best_distance - 1.0e-12 ||
      (std::abs(current_distance - best_distance) <= 1.0e-12 &&
      raw.index(current) < raw.index(best)))
    {
      best = current;
      best_distance = current_distance;
    }
    for (const auto & item : traversable_neighbors_3d(inflated, current)) {
      const auto & next = item.first;
      if (!visited[raw.index(next)]) {
        visited[raw.index(next)] = 1;
        queue.push_back(next);
      }
    }
  }
  if (requested_exact && visited[raw.index(*requested_voxel)]) {
    return GoalSelection3D{*requested_voxel, true, true, 0.0, reachable};
  }
  return GoalSelection3D{best, false, known_blocked, best_distance, reachable};
}

SearchResult3D astar_3d(
  const CostVoxelGrid & grid, const Voxel & start, const Voxel & goal,
  double heuristic_weight, double cost_weight, double timeout_ms)
{
  const auto begin = std::chrono::steady_clock::now();
  const auto elapsed = [&]() {
      return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count();
    };
  SearchResult3D result;
  if (!grid.traversable(start) || !grid.traversable(goal)) {
    result.reason = "START_OR_GOAL_BLOCKED";
    return result;
  }
  struct QueueItem {double f; double h; std::size_t serial; Voxel voxel;};
  const auto compare = [](const QueueItem & a, const QueueItem & b) {
      return std::tie(a.f, a.h, a.serial) > std::tie(b.f, b.h, b.serial);
    };
  std::priority_queue<QueueItem, std::vector<QueueItem>, decltype(compare)> open(compare);
  std::vector<double> g(grid.data().size(), std::numeric_limits<double>::infinity());
  std::vector<std::int64_t> parent(grid.data().size(), -1);
  std::vector<std::uint8_t> closed(grid.data().size(), 0);
  const auto start_index = grid.index(start);
  const auto goal_index = grid.index(goal);
  g[start_index] = 0.0;
  std::size_t serial = 0;
  open.push({heuristic_weight * voxel_distance(start, goal), voxel_distance(start, goal), serial++,
      start});
  while (!open.empty()) {
    if (elapsed() > timeout_ms) {
      result.reason = "TIMEOUT";
      result.elapsed_ms = elapsed();
      return result;
    }
    const auto item = open.top();
    open.pop();
    const auto current_index = grid.index(item.voxel);
    if (closed[current_index]) {continue;}
    closed[current_index] = 1;
    ++result.expanded;
    if (current_index == goal_index) {
      std::vector<Voxel> reversed;
      auto cursor = static_cast<std::int64_t>(goal_index);
      while (cursor >= 0) {
        const auto flat = static_cast<std::size_t>(cursor);
        const auto x = static_cast<int>(flat % grid.width());
        const auto yz = flat / grid.width();
        const auto y = static_cast<int>(yz % grid.height());
        const auto z = static_cast<int>(yz / grid.height());
        reversed.push_back({x, y, z});
        cursor = parent[flat];
      }
      result.voxels.assign(reversed.rbegin(), reversed.rend());
      result.cost = g[goal_index];
      result.elapsed_ms = elapsed();
      result.reason = "PATH_FOUND";
      return result;
    }
    for (const auto & edge : traversable_neighbors_3d(grid, item.voxel)) {
      const auto & next = edge.first;
      const auto next_index = grid.index(next);
      if (closed[next_index]) {continue;}
      const auto normalized_cost = static_cast<double>(grid.at(next)) /
        static_cast<double>(VOXEL_LETHAL_COST - 1);
      const auto tentative = g[current_index] + edge.second * (1.0 + cost_weight * normalized_cost);
      if (tentative + 1.0e-12 >= g[next_index]) {continue;}
      g[next_index] = tentative;
      parent[next_index] = static_cast<std::int64_t>(current_index);
      const auto heuristic = voxel_distance(next, goal);
      open.push({tentative + heuristic_weight * heuristic, heuristic, serial++, next});
    }
  }
  result.elapsed_ms = elapsed();
  result.reason = "NO_PATH";
  return result;
}

bool swept_sphere_clear(
  const VoxelGrid & raw, const Point3 & start, const Point3 & end, double radius)
{
  if (!std::isfinite(radius) || radius < 0.0) {return false;}
  const auto upper = raw.upper_bound();
  const auto inside = [&](const Point3 & point) {
      return point.x >= raw.origin().x + radius && point.y >= raw.origin().y + radius &&
             point.z >= raw.origin().z + radius && point.x <= upper.x - radius &&
             point.y <= upper.y - radius && point.z <= upper.z - radius;
    };
  // The shrunken map AABB is convex, so endpoint membership proves the whole
  // chord remains inside the hard planning bounds.
  if (!inside(start) || !inside(end)) {return false;}
  const auto resolution = raw.resolution();
  const auto clamp_index = [](int value, int limit) {return std::clamp(value, 0, limit - 1);};
  const auto low_x = clamp_index(
    static_cast<int>(std::floor((std::min(start.x, end.x) - radius - raw.origin().x) / resolution)),
    static_cast<int>(raw.width()));
  const auto high_x = clamp_index(
    static_cast<int>(std::floor((std::max(start.x, end.x) + radius - raw.origin().x) / resolution)),
    static_cast<int>(raw.width()));
  const auto low_y = clamp_index(
    static_cast<int>(std::floor((std::min(start.y, end.y) - radius - raw.origin().y) / resolution)),
    static_cast<int>(raw.height()));
  const auto high_y = clamp_index(
    static_cast<int>(std::floor((std::max(start.y, end.y) + radius - raw.origin().y) / resolution)),
    static_cast<int>(raw.height()));
  const auto low_z = clamp_index(
    static_cast<int>(std::floor((std::min(start.z, end.z) - radius - raw.origin().z) / resolution)),
    static_cast<int>(raw.depth()));
  const auto high_z = clamp_index(
    static_cast<int>(std::floor((std::max(start.z, end.z) + radius - raw.origin().z) / resolution)),
    static_cast<int>(raw.depth()));
  for (int z = low_z; z <= high_z; ++z) {
    for (int y = low_y; y <= high_y; ++y) {
      for (int x = low_x; x <= high_x; ++x) {
        const Voxel voxel{x, y, z};
        if (raw.at(voxel) == VoxelState::Free) {continue;}
        const Point3 lower{
          raw.origin().x + x * resolution - radius,
          raw.origin().y + y * resolution - radius,
          raw.origin().z + z * resolution - radius};
        const Point3 box_upper{lower.x + resolution + 2.0 * radius,
          lower.y + resolution + 2.0 * radius,
          lower.z + resolution + 2.0 * radius};
        if (segment_intersects_box(start, end, lower, box_upper)) {return false;}
      }
    }
  }
  return true;
}

std::vector<Point3> simplify_path_3d(
  const VoxelGrid & raw, const std::vector<Point3> & path, double radius)
{
  if (path.size() < 3) {return path;}
  std::vector<Point3> simplified{path.front()};
  std::size_t anchor = 0;
  while (anchor + 1 < path.size()) {
    std::size_t selected = anchor + 1;
    for (std::size_t candidate = path.size() - 1; candidate > anchor + 1; --candidate) {
      if (swept_sphere_clear(raw, path[anchor], path[candidate], radius)) {
        selected = candidate;
        break;
      }
    }
    // Even an adjacent A* centre-to-centre edge must pass continuous geometry.
    if (!swept_sphere_clear(raw, path[anchor], path[selected], radius)) {
      return {};
    }
    simplified.push_back(path[selected]);
    anchor = selected;
  }
  return simplified;
}

double path_length_3d(const std::vector<Point3> & path)
{
  double total = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    total += distance(path[i - 1], path[i]);
  }
  return total;
}

PlanResult3D plan_path_3d(
  const VoxelGrid & raw, const Point3 & start, const Point3 & requested_goal,
  const Planner3DConfig & config)
{
  PlanResult3D result;
  if (config.lethal_radius <= 0.0 || config.inflation_radius < config.lethal_radius) {
    result.reason = "INVALID_CONFIG";
    return result;
  }
  const auto start_voxel = raw.world_to_voxel(start);
  if (!start_voxel) {result.reason = "START_OUTSIDE_MAP"; return result;}
  auto inflated = inflate_voxels(
    raw, config.lethal_radius, config.inflation_radius, config.cost_scaling);
  const auto recovered = recover_start_3d(
    raw, inflated, *start_voxel, config.start_recovery_radius);
  if (!recovered) {result.reason = "START_BLOCKED"; return result;}
  result.recovered_start = recovered;
  // Recovery from an already-invalid envelope is a distinct controller state,
  // not an ordinary route segment. This planner identifies the monotonically
  // safer recovery target but deliberately does not publish it as a flyable
  // A* path.
  if (*recovered != *start_voxel) {
    result.reason = "RECOVERY_REQUIRED";
    return result;
  }
  // The common case is an exact known-free goal. Search it directly instead
  // of flood-filling the entire connected planning volume merely to prove it
  // reachable; on the initial 6x6x2 m crop that distinction is load-bearing.
  const auto requested_voxel = raw.world_to_voxel(requested_goal);
  if (requested_voxel && inflated.traversable(*requested_voxel)) {
    result.search = astar_3d(
      inflated, *recovered, *requested_voxel, config.heuristic_weight,
      config.cost_weight, config.timeout_ms);
    if (result.search.found()) {
      result.goal = GoalSelection3D{
        *requested_voxel, true, true, 0.0, result.search.expanded};
    }
  }
  if (!result.goal) {
    result.goal = closest_reachable_goal_3d(
      raw, inflated, *recovered, requested_goal, config.timeout_ms);
    if (!result.goal) {result.reason = "NO_REACHABLE_GOAL"; return result;}
    result.search = astar_3d(
      inflated, *recovered, result.goal->voxel, config.heuristic_weight,
      config.cost_weight, config.timeout_ms);
  }
  if (!result.search.found()) {result.reason = result.search.reason; return result;}
  std::vector<Point3> centres;
  centres.reserve(result.search.voxels.size());
  for (const auto & voxel : result.search.voxels) {
    centres.push_back(raw.voxel_center(voxel));
  }
  result.path = simplify_path_3d(raw, centres, config.lethal_radius);
  if (result.path.empty()) {result.reason = "CONTINUOUS_CLEARANCE_FAILED"; return result;}
  if (!swept_sphere_clear(raw, start, result.path.front(), config.lethal_radius)) {
    result.path.clear();
    result.reason = "START_ENVELOPE_BLOCKED";
    return result;
  }
  if (distance(start, result.path.front()) > 1.0e-9) {
    result.path.insert(result.path.begin(), start);
  }
  if (result.goal->exact) {
    if (!swept_sphere_clear(raw, result.path.back(), requested_goal, config.lethal_radius)) {
      result.path.clear();
      result.reason = "GOAL_ENVELOPE_BLOCKED";
      return result;
    }
    if (distance(result.path.back(), requested_goal) > 1.0e-9) {
      result.path.push_back(requested_goal);
    }
  }
  result.reason = result.goal->exact ? "PATH_VALID" :
    (result.goal->terminal ? "SAFE_APPROACH" : "EXPLORING");
  return result;
}

}  // namespace px4_vio_bridge
