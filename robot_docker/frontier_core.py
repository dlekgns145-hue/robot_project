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
    )
    return reachable


def _reachable_free_tree(
    data: Sequence[int],
    spec: GridSpec,
    *,
    robot_x: float,
    robot_y: float,
    free_max: int = 20,
) -> tuple[set[int], dict[int, int | None]]:
    """Return the robot-connected free cells and a shortest-path parent tree."""

    expected = spec.width * spec.height
    if spec.width <= 0 or spec.height <= 0 or len(data) != expected:
        raise ValueError(f"grid data has {len(data)} cells; expected {expected}")

    robot_grid_x, robot_grid_y = world_to_grid_cell(robot_x, robot_y, spec)
    free_cells = {
        index for index, value in enumerate(data) if 0 <= value <= free_max
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
            frontier_grid_x = index % spec.width
            frontier_grid_y = index // spec.width
            frontier_x, frontier_y = grid_cell_to_world(
                frontier_grid_x, frontier_grid_y, spec
            )
            approach_index = index
            if goal_standoff > 0.0:
                frontier_distance = math.hypot(
                    frontier_x - robot_x, frontier_y - robot_y
                )
                if frontier_distance > 1e-9:
                    ratio = min(goal_standoff, frontier_distance) / frontier_distance
                    desired_x = frontier_x - (frontier_x - robot_x) * ratio
                    desired_y = frontier_y - (frontier_y - robot_y) * ratio
                    desired_grid_x, desired_grid_y = world_to_grid_cell(
                        desired_x, desired_y, spec
                    )
                    search_radius = max(
                        1, math.ceil(goal_standoff / spec.resolution) + 1
                    )
                    nearby = []
                    for offset_y in range(-search_radius, search_radius + 1):
                        for offset_x in range(-search_radius, search_radius + 1):
                            candidate_x = desired_grid_x + offset_x
                            candidate_y = desired_grid_y + offset_y
                            if not (
                                0 <= candidate_x < spec.width
                                and 0 <= candidate_y < spec.height
                            ):
                                continue
                            candidate_index = candidate_y * spec.width + candidate_x
                            if candidate_index not in reachable:
                                continue
                            # Prefer a stable interior cell. Fall back to the
                            # frontier itself only when the known corridor is
                            # too narrow to provide one.
                            if candidate_index in frontiers:
                                continue
                            candidate_world = grid_cell_to_world(
                                candidate_x, candidate_y, spec
                            )
                            nearby.append(
                                (
                                    (candidate_world[0] - desired_x) ** 2
                                    + (candidate_world[1] - desired_y) ** 2,
                                    candidate_index,
                                )
                            )
                    if nearby:
                        approach_index = min(nearby)[1]

            target_grid_x = approach_index % spec.width
            target_grid_y = approach_index // spec.width
            target_x, target_y = grid_cell_to_world(
                target_grid_x, target_grid_y, spec
            )
            target_distance = math.hypot(target_x - robot_x, target_y - robot_y)
            if target_distance < min_distance or target_distance > max_distance:
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
            distance = math.hypot(world_x - robot_x, world_y - robot_y)
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
