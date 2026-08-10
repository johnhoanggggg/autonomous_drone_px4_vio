"""ROS-free 2-D costmap inflation, A* search, and path simplification."""

from collections import deque
from dataclasses import dataclass
import heapq
import math
import time

import numpy as np

UNKNOWN = -1
LETHAL = 255
HARD_INFLATION_DISPLAY = 70


@dataclass(frozen=True)
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: tuple

    def __post_init__(self):
        if self.width < 1 or self.height < 1 or self.resolution <= 0.0:
            raise ValueError("invalid grid geometry")
        if len(self.data) != self.width * self.height:
            raise ValueError("grid data length does not match geometry")

    def index(self, cell):
        x, y = cell
        return y * self.width + x

    def in_bounds(self, cell):
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def value(self, cell):
        return self.data[self.index(cell)]

    def world_to_cell(self, point):
        x, y = point
        cell = (
            int(math.floor((x - self.origin_x) / self.resolution)),
            int(math.floor((y - self.origin_y) / self.resolution)),
        )
        return cell if self.in_bounds(cell) else None

    def cell_center(self, cell):
        x, y = cell
        return (
            self.origin_x + (x + 0.5) * self.resolution,
            self.origin_y + (y + 0.5) * self.resolution,
        )


@dataclass(frozen=True)
class SearchResult:
    cells: tuple
    cost: float
    expanded: int
    elapsed_ms: float
    reason: str

    @property
    def found(self):
        return bool(self.cells)


@dataclass(frozen=True)
class GoalSelection:
    cell: tuple
    distance: float
    reachable_cells: int


@dataclass(frozen=True)
class StartRecovery:
    cell: tuple
    distance: float


def inflation_offsets(resolution, lethal_radius, inflation_radius, cost_scaling):
    """Return `(dx, dy, cost)` for every cell offset inside the inflation disc."""
    cells = int(math.ceil(inflation_radius / resolution))
    offsets = []
    for dy in range(-cells, cells + 1):
        for dx in range(-cells, cells + 1):
            distance = math.hypot(dx, dy) * resolution
            if distance <= inflation_radius + 1e-12:
                if distance <= lethal_radius + 1e-12:
                    cost = LETHAL
                else:
                    span = max(resolution, inflation_radius - lethal_radius)
                    decay = math.exp(-cost_scaling * (distance - lethal_radius) / span)
                    cost = max(1, min(LETHAL - 1, int(round((LETHAL - 1) * decay))))
                offsets.append((dx, dy, cost))
    return tuple(offsets)


def inflate_occupancy(
    grid,
    *,
    occupied_threshold=65,
    lethal_radius=0.40,
    inflation_radius=0.60,
    cost_scaling=3.0,
):
    """Return UNKNOWN/0..LETHAL costs while preserving unknown cells.

    Each cell takes the highest cost any obstacle within `inflation_radius`
    projects onto it.  That maximum is computed by dilating the obstacle mask
    once per kernel offset rather than by walking the kernel around every
    obstacle: the loop runs ~450 times (the offsets) instead of ~1M times
    (obstacles x offsets), which is the difference between 1.3 s and 27 ms for
    a flight-sized map on a Pi.  Both forms take the same per-offset maximum,
    so the costs are identical.
    """
    if lethal_radius < 0.0 or inflation_radius < lethal_radius:
        raise ValueError("inflation radius must be at least the lethal radius")
    source = np.asarray(grid.data, dtype=np.int16).reshape(grid.height, grid.width)
    obstacles = source >= occupied_threshold
    costs = np.zeros_like(source, dtype=np.int16)
    height, width = grid.height, grid.width
    for dx, dy, cost in inflation_offsets(
        grid.resolution, lethal_radius, inflation_radius, cost_scaling
    ):
        # Cells at (x, y) inherit `cost` from an obstacle at (x - dx, y - dy);
        # clip both windows to the map instead of bounds-checking per cell.
        y0, y1 = max(0, dy), min(height, height + dy)
        x0, x1 = max(0, dx), min(width, width + dx)
        if y0 >= y1 or x0 >= x1:
            continue
        target = costs[y0:y1, x0:x1]
        sources = obstacles[y0 - dy:y1 - dy, x0 - dx:x1 - dx]
        np.maximum(target, np.where(sources, np.int16(cost), np.int16(0)), out=target)
    # Unknown space is never inflated into: it is already untraversable, and
    # overwriting it would hide the frontier the goal selector flies towards.
    costs[source < 0] = UNKNOWN
    return GridMap(
        grid.width,
        grid.height,
        grid.resolution,
        grid.origin_x,
        grid.origin_y,
        tuple(costs.reshape(-1).tolist()),
    )


def inflation_display_data(source_grid, inflated_grid, *, occupied_threshold=65):
    """Encode a costmap for OccupancyGrid without hiding real obstacles.

    The planner uses 255 for both source obstacles and the hard vehicle
    envelope around them.  Publishing both as occupancy 100 makes Foxglove
    render both black.  Keep source obstacles black, render hard inflation as
    dark grey, and scale the traversable outer cost band below that.  This is
    deliberately display-only; A* continues to consume ``inflated_grid``.
    """
    if (
        source_grid.width != inflated_grid.width
        or source_grid.height != inflated_grid.height
        or len(source_grid.data) != len(inflated_grid.data)
    ):
        raise ValueError("source and inflated grids must have matching geometry")

    display = []
    for source, cost in zip(source_grid.data, inflated_grid.data):
        if source < 0:
            display.append(UNKNOWN)
        elif source >= occupied_threshold:
            display.append(100)
        elif cost >= LETHAL:
            display.append(HARD_INFLATION_DISPLAY)
        elif cost > 0:
            display.append(
                max(
                    1,
                    min(
                        HARD_INFLATION_DISPLAY - 1,
                        int(
                            round(
                                cost
                                * (HARD_INFLATION_DISPLAY - 1)
                                / (LETHAL - 1)
                            )
                        ),
                    ),
                )
            )
        else:
            display.append(0)
    return display


def traversable(grid, cell):
    return grid.in_bounds(cell) and 0 <= grid.value(cell) < LETHAL


def _heuristic(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)


MOVES = (
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
)


def traversable_neighbors(grid, cell):
    """Yield safe eight-connected neighbours without diagonal corner cutting."""
    cx, cy = cell
    for dx, dy, distance in MOVES:
        nxt = cx + dx, cy + dy
        if not traversable(grid, nxt):
            continue
        if dx and dy:
            if not traversable(grid, (cx + dx, cy)) or not traversable(
                grid, (cx, cy + dy)
            ):
                continue
        yield nxt, distance


def recover_start(source_grid, inflated_grid, start, *, occupied_threshold=65,
                  max_radius=0.0):
    """Return the closest cell A* can start from when `start` sits in inflation.

    A vehicle that passes within `lethal_radius` of a mapped obstacle marks its
    own cell LETHAL, so A* refuses to plan at all — the route dies because the
    aircraft is *near* an obstacle, not because it is stuck.  Flood outward from
    the start over cells that are neither mapped obstacles nor unknown, and
    return the first traversable cell found.  Routing the flood around real
    obstacles (rather than taking the nearest traversable cell by distance)
    keeps the recovered start on the vehicle's own side of a thin wall.

    Returns None when `start` is itself an obstacle or unknown, or when nothing
    traversable lies within `max_radius` metres.
    """
    if traversable(inflated_grid, start):
        return StartRecovery(start, 0.0)
    if max_radius <= 0.0:
        return None
    limit = int(math.floor(max_radius / inflated_grid.resolution))
    if limit < 1:
        return None

    def escapable(cell):
        if not source_grid.in_bounds(cell):
            return False
        value = source_grid.value(cell)
        return 0 <= value < occupied_threshold

    if not escapable(start):
        return None
    queue = deque([start])
    visited = {start}
    while queue:
        current = queue.popleft()
        for dx, dy, _ in MOVES:
            nxt = (current[0] + dx, current[1] + dy)
            if nxt in visited or not escapable(nxt):
                continue
            offset = math.hypot(nxt[0] - start[0], nxt[1] - start[1])
            if offset > limit:
                continue
            visited.add(nxt)
            if traversable(inflated_grid, nxt):
                return StartRecovery(nxt, offset * inflated_grid.resolution)
            queue.append(nxt)
    return None


def closest_reachable_goal(grid, start, requested_world):
    """Return the reachable safe cell closest to an arbitrary world goal.

    Unknown, lethal, outside-map, and disconnected requested cells are never
    made traversable. Instead, this floods the start's safe connected component
    and returns its closest cell to the requested point.
    """
    if not traversable(grid, start):
        return None
    requested = tuple(float(value) for value in requested_world)
    if not all(math.isfinite(value) for value in requested):
        raise ValueError("requested goal must be finite")
    queue = deque([start])
    visited = {start}
    best = start
    best_key = (
        math.dist(grid.cell_center(start), requested),
        0.0,
        start,
    )
    while queue:
        current = queue.popleft()
        key = (
            math.dist(grid.cell_center(current), requested),
            _heuristic(start, current),
            current,
        )
        if key < best_key:
            best, best_key = current, key
        for nxt, _ in traversable_neighbors(grid, current):
            if nxt not in visited:
                visited.add(nxt)
                queue.append(nxt)
    return GoalSelection(best, best_key[0], len(visited))


def classify_goal(source_grid, inflated_grid, requested_world, selected_cell):
    """Return `(exact, terminal)` for a selected safe approach endpoint.

    A known but unsafe requested cell (an obstacle or inside its lethal
    inflation) has a terminal safe-approach endpoint. Unknown, outside-map, or
    disconnected safe goals remain non-terminal so mapping can advance them.
    """
    requested_cell = source_grid.world_to_cell(requested_world)
    exact = requested_cell == selected_cell
    known = requested_cell is not None and source_grid.value(requested_cell) >= 0
    safe = requested_cell is not None and traversable(inflated_grid, requested_cell)
    return exact, exact or (known and not safe)


def astar(grid, start, goal, *, heuristic_weight=1.0, cost_weight=2.0, timeout_ms=100.0):
    """Cost-aware eight-connected A*, forbidding unknown and corner cutting."""
    begun = time.monotonic()
    if not traversable(grid, start):
        return SearchResult((), math.inf, 0, 0.0, "START_BLOCKED")
    if not traversable(grid, goal):
        return SearchResult((), math.inf, 0, 0.0, "GOAL_BLOCKED")
    if heuristic_weight < 1.0:
        raise ValueError("heuristic_weight must be >= 1")
    frontier = [(heuristic_weight * _heuristic(start, goal), 0.0, start)]
    came_from = {}
    best = {start: 0.0}
    expanded = 0
    while frontier:
        elapsed_ms = (time.monotonic() - begun) * 1000.0
        if timeout_ms > 0.0 and elapsed_ms > timeout_ms:
            return SearchResult((), math.inf, expanded, elapsed_ms, "TIMEOUT")
        _, current_cost, current = heapq.heappop(frontier)
        if current_cost > best.get(current, math.inf) + 1e-12:
            continue
        expanded += 1
        if current == goal:
            path = [current]
            while current != start:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return SearchResult(
                tuple(path), current_cost, expanded,
                (time.monotonic() - begun) * 1000.0, "PATH_VALID"
            )
        for nxt, distance in traversable_neighbors(grid, current):
            cell_cost = grid.value(nxt) / float(LETHAL - 1)
            candidate = current_cost + distance * (1.0 + cost_weight * cell_cost)
            if candidate + 1e-12 >= best.get(nxt, math.inf):
                continue
            best[nxt] = candidate
            came_from[nxt] = current
            priority = candidate + heuristic_weight * _heuristic(nxt, goal)
            heapq.heappush(frontier, (priority, candidate, nxt))
    return SearchResult(
        (), math.inf, expanded, (time.monotonic() - begun) * 1000.0, "NO_KNOWN_PATH"
    )


def line_is_clear(grid, start, end):
    """Conservative grid line check with diagonal corner checks."""
    x0, y0 = start
    x1, y1 = end
    dx, dy = x1 - x0, y1 - y0
    steps = max(abs(dx), abs(dy)) * 2 + 1
    previous = start
    for i in range(steps + 1):
        t = i / max(1, steps)
        cell = (int(round(x0 + dx * t)), int(round(y0 + dy * t)))
        if not traversable(grid, cell):
            return False
        sx, sy = cell[0] - previous[0], cell[1] - previous[1]
        if sx and sy:
            if not traversable(grid, (previous[0] + sx, previous[1])):
                return False
            if not traversable(grid, (previous[0], previous[1] + sy)):
                return False
        previous = cell
    return True


def simplify_path(grid, cells):
    if len(cells) <= 2:
        return tuple(cells)
    simplified = [cells[0]]
    anchor = 0
    while anchor < len(cells) - 1:
        candidate = len(cells) - 1
        while candidate > anchor + 1 and not line_is_clear(grid, cells[anchor], cells[candidate]):
            candidate -= 1
        simplified.append(cells[candidate])
        anchor = candidate
    return tuple(simplified)


def path_length(points):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))
