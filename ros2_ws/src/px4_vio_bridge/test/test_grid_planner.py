import math

from px4_vio_bridge.grid_planner import (
    GridMap,
    LETHAL,
    astar,
    inflate_occupancy,
    line_is_clear,
    path_length,
    simplify_path,
)


def grid(rows, resolution=0.1):
    height = len(rows)
    width = len(rows[0])
    values = tuple(LETHAL if v == 100 else v for row in rows for v in row)
    return GridMap(width, height, resolution, 0.0, 0.0, values)


def test_open_grid_finds_diagonal_path():
    result = astar(grid([[0] * 6 for _ in range(6)]), (0, 0), (5, 5))
    assert result.found
    assert result.cells[0] == (0, 0)
    assert result.cells[-1] == (5, 5)
    assert math.isclose(result.cost, 5 * math.sqrt(2.0))


def test_unknown_is_not_traversable():
    rows = [[0, -1, 0], [0, -1, 0], [0, -1, 0]]
    result = astar(grid(rows), (0, 1), (2, 1))
    assert not result.found
    assert result.reason == "NO_KNOWN_PATH"


def test_search_routes_through_known_gap_and_out_of_dead_end():
    rows = [[0] * 9 for _ in range(9)]
    for y in range(8):
        rows[y][4] = 100
    rows[6][4] = 0
    result = astar(grid(rows), (1, 2), (7, 2))
    assert result.found
    assert (4, 6) in result.cells


def test_diagonal_corner_cutting_is_forbidden():
    rows = [[0, 100], [100, 0]]
    result = astar(grid(rows), (0, 0), (1, 1))
    assert not result.found


def test_start_and_goal_failures_are_distinct():
    blocked = grid([[100, 0, 100]])
    assert astar(blocked, (0, 0), (1, 0)).reason == "START_BLOCKED"
    assert astar(blocked, (1, 0), (2, 0)).reason == "GOAL_BLOCKED"


def test_inflation_preserves_unknown_and_enforces_clearance():
    source = grid([
        [-1, -1, -1, -1, -1],
        [0, 0, 0, 0, 0],
        [0, 0, 100, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
    ], resolution=0.1)
    costs = inflate_occupancy(source, lethal_radius=0.15, inflation_radius=0.30)
    assert costs.value((0, 0)) == -1
    assert costs.value((2, 2)) == LETHAL
    assert costs.value((3, 2)) == LETHAL
    assert 0 < costs.value((4, 2)) < LETHAL


def test_narrow_gap_is_closed_by_vehicle_inflation():
    rows = [[0] * 15 for _ in range(11)]
    for y in range(11):
        if y not in (4, 5, 6):
            rows[y][7] = 100
    source = grid(rows, resolution=0.2)
    costs = inflate_occupancy(source, lethal_radius=0.4, inflation_radius=0.4)
    assert not astar(costs, (2, 5), (12, 5)).found


def test_simplification_keeps_required_corner_but_removes_collinear_cells():
    rows = [[0] * 7 for _ in range(7)]
    rows[3][3] = 100
    world = grid(rows)
    cells = ((0, 3), (1, 3), (2, 3), (2, 4), (3, 4), (4, 4), (5, 4), (6, 4))
    simple = simplify_path(world, cells)
    assert simple[0] == cells[0] and simple[-1] == cells[-1]
    assert len(simple) < len(cells)
    assert all(line_is_clear(world, a, b) for a, b in zip(simple, simple[1:]))


def test_world_cell_round_trip_uses_cell_centres():
    world = GridMap(10, 10, 0.2, -1.0, -2.0, (0,) * 100)
    cell = world.world_to_cell((-0.31, -0.71))
    assert cell == (3, 6)
    assert world.world_to_cell(world.cell_center(cell)) == cell


def test_path_length():
    assert path_length(((0.0, 0.0), (3.0, 4.0))) == 5.0
