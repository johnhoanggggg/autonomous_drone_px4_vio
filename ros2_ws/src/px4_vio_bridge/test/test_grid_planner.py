import math

from px4_vio_bridge.grid_planner import (
    GridMap,
    LETHAL,
    UNKNOWN,
    astar,
    classify_goal,
    closest_reachable_goal,
    inflate_occupancy,
    inflation_display_data,
    inflation_offsets,
    line_is_clear,
    path_length,
    recover_start,
    simplify_path,
    traversable,
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


def test_inflation_display_distinguishes_obstacles_from_safety_envelope():
    source = grid([
        [-1, 0, 0, 0, 0],
        [0, 0, 100, 0, 0],
        [0, 0, 0, 0, 0],
    ], resolution=0.1)
    costs = inflate_occupancy(
        source, lethal_radius=0.10, inflation_radius=0.20
    )
    display = inflation_display_data(source, costs)

    assert display[source.index((0, 0))] == -1
    assert display[source.index((2, 1))] == 100
    assert display[source.index((1, 1))] == 70
    assert 0 < display[source.index((0, 1))] < 70
    assert display[source.index((4, 2))] == 0


def reference_inflation(source, **kwargs):
    """The per-obstacle kernel walk `inflate_occupancy` replaced, kept as an oracle."""
    offsets = inflation_offsets(
        source.resolution,
        kwargs["lethal_radius"],
        kwargs["inflation_radius"],
        kwargs.get("cost_scaling", 3.0),
    )
    threshold = kwargs.get("occupied_threshold", 65)
    costs = [UNKNOWN if v < 0 else 0 for v in source.data]
    for index, value in enumerate(source.data):
        if value < threshold:
            continue
        ox, oy = index % source.width, index // source.width
        for dx, dy, cost in offsets:
            x, y = ox + dx, oy + dy
            if not source.in_bounds((x, y)):
                continue
            at = y * source.width + x
            if costs[at] != UNKNOWN:
                costs[at] = max(costs[at], cost)
    return tuple(costs)


def test_vectorised_inflation_matches_the_per_obstacle_kernel_walk():
    rows = [[0] * 17 for _ in range(13)]
    for y in range(13):
        rows[y][0] = -1
    rows[2][4] = rows[3][4] = rows[9][11] = rows[10][12] = 100
    rows[6][8] = 100
    rows[6][9] = -1
    source = grid(rows, resolution=0.05)
    settings = dict(lethal_radius=0.10, inflation_radius=0.25, cost_scaling=3.0)
    assert (
        inflate_occupancy(source, **settings).data
        == reference_inflation(source, **settings)
    )


def test_narrow_gap_is_closed_by_vehicle_inflation():
    rows = [[0] * 15 for _ in range(11)]
    for y in range(11):
        if y not in (4, 5, 6):
            rows[y][7] = 100
    source = grid(rows, resolution=0.2)
    costs = inflate_occupancy(source, lethal_radius=0.4, inflation_radius=0.4)
    assert not astar(costs, (2, 5), (12, 5)).found


def test_start_inside_lethal_inflation_recovers_instead_of_blocking():
    # offboard_global_props_on_22: the vehicle passed 0.36m from a mapped
    # obstacle with a 0.40m envelope, so its own free cell went LETHAL and the
    # route was dropped three times.
    source = grid([
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 100, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
    ])
    inflated = inflate_occupancy(source, lethal_radius=0.15, inflation_radius=0.15)
    assert not traversable(inflated, (2, 1))

    recovery = recover_start(source, inflated, (2, 1), max_radius=0.30)
    assert recovery is not None
    assert traversable(inflated, recovery.cell)
    assert recovery.distance <= 0.30
    assert astar(inflated, recovery.cell, (0, 1)).found


def test_traversable_start_is_returned_unmoved():
    world = grid([[0, 0, 0]])
    recovery = recover_start(world, world, (1, 0), max_radius=0.30)
    assert (recovery.cell, recovery.distance) == ((1, 0), 0.0)


def test_start_recovery_stays_on_the_vehicle_side_of_a_wall():
    # Free space one cell away through the wall must not win over free space
    # further away on this side, or the planner would hand the follower a path
    # starting beyond an obstacle the vehicle cannot cross.
    source = grid([
        [0, 0, 100, 0],
        [0, 0, 100, 0],
        [0, 0, 100, 0],
    ])
    inflated = inflate_occupancy(source, lethal_radius=0.15, inflation_radius=0.15)
    recovery = recover_start(source, inflated, (1, 1), max_radius=0.50)
    assert recovery is not None
    assert recovery.cell[0] < 2


def test_start_recovery_gives_up_beyond_its_radius():
    rows = [[0] * 9 for _ in range(9)]
    for y in range(9):
        rows[y][0] = 100
        rows[y][8] = 100
    source = grid(rows)
    inflated = inflate_occupancy(source, lethal_radius=0.45, inflation_radius=0.45)
    assert recover_start(source, inflated, (4, 4), max_radius=0.10) is None
    assert recover_start(source, inflated, (4, 4), max_radius=0.0) is None


def test_start_recovery_refuses_an_obstacle_or_unknown_cell():
    source = grid([[100, 0, -1]])
    inflated = inflate_occupancy(source, lethal_radius=0.05, inflation_radius=0.05)
    assert recover_start(source, inflated, (0, 0), max_radius=0.50) is None
    assert recover_start(source, inflated, (2, 0), max_radius=0.50) is None


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


def test_unknown_goal_selects_nearest_known_reachable_frontier():
    world = grid([[0, 0, 0, -1, -1]])
    selection = closest_reachable_goal(world, (0, 0), (0.45, 0.05))
    assert selection.cell == (2, 0)
    assert not classify_goal(world, world, (0.45, 0.05), selection.cell)[1]
    assert all(world.value(cell) >= 0 for cell in astar(world, (0, 0), selection.cell).cells)


def test_outside_map_goal_selects_reachable_map_edge_and_is_not_terminal():
    world = grid([[0, 0, 0, 0, 0]])
    selection = closest_reachable_goal(world, (0, 0), (3.0, 0.05))
    assert selection.cell == (4, 0)
    assert classify_goal(world, world, (3.0, 0.05), selection.cell) == (
        False,
        False,
    )


def test_obstacle_goal_stops_outside_lethal_inflation_and_is_terminal():
    source = grid([[0, 0, 0, 0, 0, 100, 0, 0, 0]])
    inflated = inflate_occupancy(
        source, lethal_radius=0.15, inflation_radius=0.15
    )
    selection = closest_reachable_goal(inflated, (0, 0), (0.55, 0.05))
    assert selection.cell == (3, 0)
    assert classify_goal(source, inflated, (0.55, 0.05), selection.cell) == (
        False,
        True,
    )
    assert all(
        0 <= inflated.value(cell) < LETHAL
        for cell in astar(inflated, (0, 0), selection.cell).cells
    )


def test_disconnected_safe_goal_approaches_but_waits_for_more_map():
    world = grid([[0, 0, 100, 0, 0]])
    selection = closest_reachable_goal(world, (0, 0), (0.45, 0.05))
    assert selection.cell == (1, 0)
    assert classify_goal(world, world, (0.45, 0.05), selection.cell) == (
        False,
        False,
    )


def test_frontier_endpoint_advances_as_unknown_space_becomes_free():
    first = grid([[0, 0, -1, -1, -1]])
    second = grid([[0, 0, 0, 0, -1]])
    requested = (0.45, 0.05)
    assert closest_reachable_goal(first, (0, 0), requested).cell == (1, 0)
    assert closest_reachable_goal(second, (0, 0), requested).cell == (3, 0)


def test_exact_reachable_goal_is_exact_and_terminal():
    world = grid([[0, 0, 0]])
    selection = closest_reachable_goal(world, (0, 0), (0.25, 0.05))
    assert selection.cell == (2, 0)
    assert classify_goal(world, world, (0.25, 0.05), selection.cell) == (
        True,
        True,
    )
