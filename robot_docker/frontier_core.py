"""Pure occupancy-grid frontier detection used by autonomous mapping."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence


@dataclass(frozen=True)
class GridSpec:
    width: int
    height: int
    resolution: float
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw: float = 0.0


@dataclass(frozen=True)
class FrontierCandidate:
    grid_x: int
    grid_y: int
    x: float
    y: float
    cell_count: int
    distance: float
    score: float


def _neighbors4(index: int, width: int, height: int) -> Iterable[int]:
    x = index % width
    y = index // width
    if x > 0:
        yield index - 1
    if x + 1 < width:
        yield index + 1
    if y > 0:
        yield index - width
    if y + 1 < height:
        yield index + width


def _neighbors8(index: int, width: int, height: int) -> Iterable[int]:
    x = index % width
    y = index // width
    for offset_y in (-1, 0, 1):
        for offset_x in (-1, 0, 1):
            if offset_x == 0 and offset_y == 0:
                continue
            neighbor_x = x + offset_x
            neighbor_y = y + offset_y
            if 0 <= neighbor_x < width and 0 <= neighbor_y < height:
                yield neighbor_y * width + neighbor_x


def frontier_cell_indices(
    data: Sequence[int], spec: GridSpec, *, free_max: int = 20
) -> set[int]:
    """Return known-free cells that touch unknown space."""

    expected = spec.width * spec.height
    if spec.width <= 0 or spec.height <= 0 or len(data) != expected:
        raise ValueError(f"grid data has {len(data)} cells; expected {expected}")

    frontiers: set[int] = set()
    for index, value in enumerate(data):
        if value < 0 or value > free_max:
            continue
        if any(data[neighbor] < 0 for neighbor in _neighbors4(
            index, spec.width, spec.height
        )):
            frontiers.add(index)
    return frontiers


def cluster_frontiers(
    frontier_indices: Iterable[int], spec: GridSpec
) -> list[tuple[int, ...]]:
    """Group frontier cells using eight-way connectivity."""

    remaining = set(frontier_indices)
    clusters: list[tuple[int, ...]] = []
    while remaining:
        seed = remaining.pop()
        cluster = [seed]
        pending = [seed]
        while pending:
            current = pending.pop()
            for neighbor in _neighbors8(current, spec.width, spec.height):
                if neighbor not in remaining:
                    continue
                remaining.remove(neighbor)
                cluster.append(neighbor)
                pending.append(neighbor)
        clusters.append(tuple(cluster))
    return clusters


def grid_cell_to_world(grid_x: int, grid_y: int, spec: GridSpec) -> tuple[float, float]:
    local_x = (grid_x + 0.5) * spec.resolution
    local_y = (grid_y + 0.5) * spec.resolution
    cosine = math.cos(spec.origin_yaw)
    sine = math.sin(spec.origin_yaw)
    return (
        spec.origin_x + cosine * local_x - sine * local_y,
        spec.origin_y + sine * local_x + cosine * local_y,
    )


def world_to_grid_cell(x: float, y: float, spec: GridSpec) -> tuple[int, int]:
    """Convert a world position into a grid cell, including a rotated origin."""

    delta_x = x - spec.origin_x
    delta_y = y - spec.origin_y
    cosine = math.cos(spec.origin_yaw)
    sine = math.sin(spec.origin_yaw)
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    return math.floor(local_x / spec.resolution), math.floor(
        local_y / spec.resolution
    )


def reachable_free_cell_indices(
    data: Sequence[int],
    spec: GridSpec,
    *,
    robot_x: float,
    robot_y: float,
    free_max: int = 20,
    maximum_seed_distance: float = 0.0,
) -> set[int]:
    """Return the known-free component connected to the robot.

    A frontier can be close in straight-line distance while sitting across an
    unknown strip, fence, or tree row. Nav2 cannot plan to such a cell when
    ``allow_unknown`` is disabled, so those cells must not become goals.
    """

    reachable, _parents = _reachable_free_tree(
        data,
        spec,
        robot_x=robot_x,
        robot_y=robot_y,
        free_max=free_max,
        maximum_seed_distance=maximum_seed_distance,
    )
    return reachable


def _reachable_free_tree(
    data: Sequence[int],
    spec: GridSpec,
    *,
    robot_x: float,
    robot_y: float,
    free_max: int = 20,
    maximum_seed_distance: float = 0.0,
    minimum_path_obstacle_clearance: float = 0.0,
) -> tuple[set[int], dict[int, int | None]]:
    """Return robot-traversable free cells and a shortest-path parent tree.

    Raw free-cell connectivity is insufficient for a physical robot: a
    one-cell gap can connect two rooms in the occupancy grid even though the
    footprint cannot pass through it.  Optional path clearance removes those
    cells before reachability is evaluated.
    """

    expected = spec.width * spec.height
    if spec.width <= 0 or spec.height <= 0 or len(data) != expected:
        raise ValueError(f"grid data has {len(data)} cells; expected {expected}")

    robot_grid_x, robot_grid_y = world_to_grid_cell(robot_x, robot_y, spec)
    free_cells = {
        index for index, value in enumerate(data) if 0 <= value <= free_max
    }
    if minimum_path_obstacle_clearance > 0.0:
        free_cells = {
            index
            for index in free_cells
            if cell_has_obstacle_clearance(
                data,
                spec,
                index,
                clearance_m=minimum_path_obstacle_clearance,
                free_max=free_max,
            )
        }
    if not free_cells:
        return set(), {}

    if 0 <= robot_grid_x < spec.width and 0 <= robot_grid_y < spec.height:
        seed = robot_grid_y * spec.width + robot_grid_x
    else:
        seed = -1
    if seed not in free_cells:
        # SLAM can place the base cell just outside the free raster because of
        # rounding or footprint clearing. Use the nearest known-free seed.
        seed = min(
            free_cells,
            key=lambda index: (
                (index % spec.width - robot_grid_x) ** 2
                + (index // spec.width - robot_grid_y) ** 2
            ),
        )
        if maximum_seed_distance > 0.0:
            seed_x = seed % spec.width
            seed_y = seed // spec.width
            seed_distance = math.hypot(
                seed_x - robot_grid_x,
                seed_y - robot_grid_y,
            ) * spec.resolution
            if seed_distance > maximum_seed_distance:
                return set(), {}

    reachable = {seed}
    parents: dict[int, int | None] = {seed: None}
    pending = [seed]
    pending_index = 0
    while pending_index < len(pending):
        current = pending[pending_index]
        pending_index += 1
        for neighbor in _neighbors4(current, spec.width, spec.height):
            if neighbor not in free_cells or neighbor in reachable:
                continue
            reachable.add(neighbor)
            parents[neighbor] = current
            pending.append(neighbor)
    return reachable, parents


def _staged_path_cell(
    target: int,
    parents: dict[int, int | None],
    *,
    resolution: float,
    maximum_step_distance: float,
) -> int:
    """Pull a far target back along its known-free shortest path."""

    if maximum_step_distance <= 0.0 or target not in parents:
        return target
    reverse_path = [target]
    while parents[reverse_path[-1]] is not None:
        reverse_path.append(parents[reverse_path[-1]])
    path = list(reversed(reverse_path))
    maximum_steps = max(1, math.floor(maximum_step_distance / resolution))
    return path[min(maximum_steps, len(path) - 1)]


def _path_step_count(target: int, parents: dict[int, int | None]) -> int:
    """Return shortest known-free path length in grid steps."""

    if target not in parents:
        return 0
    steps = 0
    current = target
    while parents[current] is not None:
        current = parents[current]
        steps += 1
    return steps


def _path_standoff_cell(
    target: int,
    parents: dict[int, int | None],
    *,
    resolution: float,
    standoff_distance: float,
) -> int:
    """Move a frontier goal inward along its reachable free-space path."""

    if standoff_distance <= 0.0 or target not in parents:
        return target
    steps = max(1, math.ceil(standoff_distance / resolution))
    current = target
    for _ in range(steps):
        parent = parents.get(current)
        if parent is None:
            break
        current = parent
    return current


def cell_has_obstacle_clearance(
    data: Sequence[int],
    spec: GridSpec,
    index: int,
    *,
    clearance_m: float,
    free_max: int = 20,
) -> bool:
    """Whether a goal cell is clear of occupied map cells by a safe radius.

    Unknown cells are intentionally handled by frontier standoff instead of
    this function. Treating every unknown cell as an obstacle here would leave
    no valid goal during the first seconds of exploration.
    """

    if clearance_m <= 0.0:
        return True
    center_x = index % spec.width
    center_y = index // spec.width
    cell_radius = math.ceil(clearance_m / spec.resolution)
    clearance_squared = clearance_m * clearance_m
    for offset_y in range(-cell_radius, cell_radius + 1):
        for offset_x in range(-cell_radius, cell_radius + 1):
            if (
                (offset_x * spec.resolution) ** 2
                + (offset_y * spec.resolution) ** 2
                > clearance_squared
            ):
                continue
            grid_x = center_x + offset_x
            grid_y = center_y + offset_y
            if not (0 <= grid_x < spec.width and 0 <= grid_y < spec.height):
                continue
            value = data[grid_y * spec.width + grid_x]
            if value > free_max:
                return False
    return True


def frontier_candidates(
    data: Sequence[int],
    spec: GridSpec,
    *,
    robot_x: float,
    robot_y: float,
    min_cells: int = 4,
    min_distance: float = 0.35,
    max_distance: float = 8.0,
    blacklisted: Sequence[tuple[float, float]] = (),
    blacklist_radius: float = 0.6,
    gain_weight: float = 1.0,
    distance_weight: float = 0.45,
    goal_standoff: float = 0.0,
    maximum_goal_step_distance: float = 0.0,
    minimum_obstacle_clearance: float = 0.0,
    maximum_robot_free_seed_distance: float = 0.0,
) -> list[FrontierCandidate]:
    """Return scored, reachable frontier approach goals, best first.

    ``goal_standoff`` pulls each goal back from the changing unknown boundary
    into stable known-free space. This prevents a SLAM map update from turning
    the active Nav2 goal into an unknown cell while the robot is driving.
    """

    frontiers = frontier_cell_indices(data, spec)
    clusters = cluster_frontiers(frontiers, spec)
    reachable, parents = _reachable_free_tree(
        data,
        spec,
        robot_x=robot_x,
        robot_y=robot_y,
        maximum_seed_distance=maximum_robot_free_seed_distance,
        minimum_path_obstacle_clearance=minimum_obstacle_clearance,
    )
    candidates: list[FrontierCandidate] = []

    for cluster in clusters:
        if len(cluster) < max(1, min_cells):
            continue
        centroid_x = sum(index % spec.width for index in cluster) / len(cluster)
        centroid_y = sum(index // spec.width for index in cluster) / len(cluster)
        usable_cells: list[tuple[int, int, float, float, float, float]] = []
        for index in cluster:
            if index not in reachable:
                continue
            approach_index = _path_standoff_cell(
                index,
                parents,
                resolution=spec.resolution,
                standoff_distance=goal_standoff,
            )
            if not cell_has_obstacle_clearance(
                data,
                spec,
                approach_index,
                clearance_m=minimum_obstacle_clearance,
            ):
                continue

            target_distance = (
                _path_step_count(approach_index, parents) * spec.resolution
            )
            if target_distance < min_distance:
                continue
            staged_index = _staged_path_cell(
                approach_index,
                parents,
                resolution=spec.resolution,
                maximum_step_distance=maximum_goal_step_distance,
            )
            grid_x = staged_index % spec.width
            grid_y = staged_index // spec.width
            world_x, world_y = grid_cell_to_world(grid_x, grid_y, spec)
            distance = _path_step_count(staged_index, parents) * spec.resolution
            # A distant frontier is still actionable when it can be reached
            # through bounded intermediate goals.  Applying max_distance to
            # the final frontier before staging incorrectly discarded exactly
            # the large unexplored regions that staging is intended to reach.
            if max_distance > 0.0 and distance > max_distance:
                continue
            if not cell_has_obstacle_clearance(
                data,
                spec,
                staged_index,
                clearance_m=minimum_obstacle_clearance,
            ):
                continue
            if any(
                math.hypot(world_x - blocked_x, world_y - blocked_y)
                <= blacklist_radius
                for blocked_x, blocked_y in blacklisted
            ):
                continue
            usable_cells.append(
                (
                    index,
                    staged_index,
                    world_x,
                    world_y,
                    distance,
                    target_distance,
                )
            )
        if not usable_cells:
            continue

        representative, approach, world_x, world_y, distance, target_distance = min(
            usable_cells,
            key=lambda item: (
                (item[0] % spec.width - centroid_x) ** 2
                + (item[0] // spec.width - centroid_y) ** 2,
                item[4],
            ),
        )
        grid_x = approach % spec.width
        grid_y = approach // spec.width

        frontier_length = len(cluster) * spec.resolution
        score = gain_weight * frontier_length - distance_weight * target_distance
        candidates.append(
            FrontierCandidate(
                grid_x=grid_x,
                grid_y=grid_y,
                x=world_x,
                y=world_y,
                cell_count=len(cluster),
                distance=distance,
                score=score,
            )
        )

    return sorted(candidates, key=lambda candidate: candidate.score, reverse=True)
