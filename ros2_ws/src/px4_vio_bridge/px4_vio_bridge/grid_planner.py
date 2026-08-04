"""ROS-free 2-D costmap inflation, A* search, and path simplification."""

from dataclasses import dataclass
import heapq
import math
import time

import numpy as np

UNKNOWN = -1
LETHAL = 255


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


def inflate_occupancy(
    grid,
    *,
    occupied_threshold=65,
    lethal_radius=0.40,
    inflation_radius=0.60,
    cost_scaling=3.0,
):
    """Return UNKNOWN/0..LETHAL costs while preserving unknown cells."""
    if lethal_radius < 0.0 or inflation_radius < lethal_radius:
        raise ValueError("inflation radius must be at least the lethal radius")
    source = np.asarray(grid.data, dtype=np.int16).reshape(grid.height, grid.width)
    costs = np.zeros_like(source, dtype=np.int16)
    costs[source < 0] = UNKNOWN
    obstacles = np.argwhere(source >= occupied_threshold)
    costs[source >= occupied_threshold] = LETHAL
    cells = int(math.ceil(inflation_radius / grid.resolution))
    offsets = []
    for dy in range(-cells, cells + 1):
        for dx in range(-cells, cells + 1):
            distance = math.hypot(dx, dy) * grid.resolution
            if distance <= inflation_radius + 1e-12:
                if distance <= lethal_radius + 1e-12:
                    cost = LETHAL
                else:
                    span = max(grid.resolution, inflation_radius - lethal_radius)
                    decay = math.exp(-cost_scaling * (distance - lethal_radius) / span)
                    cost = max(1, min(LETHAL - 1, int(round((LETHAL - 1) * decay))))
                offsets.append((dx, dy, cost))
    for oy, ox in obstacles:
        for dx, dy, cost in offsets:
            x, y = ox + dx, oy + dy
            if 0 <= x < grid.width and 0 <= y < grid.height and costs[y, x] != UNKNOWN:
                costs[y, x] = max(costs[y, x], cost)
    return GridMap(
        grid.width,
        grid.height,
        grid.resolution,
        grid.origin_x,
        grid.origin_y,
        tuple(int(v) for v in costs.reshape(-1)),
    )


def traversable(grid, cell):
    return grid.in_bounds(cell) and 0 <= grid.value(cell) < LETHAL


def _heuristic(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)


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
    moves = (
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
    )
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
        cx, cy = current
        for dx, dy, distance in moves:
            nxt = (cx + dx, cy + dy)
            if not traversable(grid, nxt):
                continue
            if dx and dy:
                if not traversable(grid, (cx + dx, cy)) or not traversable(grid, (cx, cy + dy)):
                    continue
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
