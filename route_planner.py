import heapq
import math
from itertools import count

from obstacle import Obstacle

Point = tuple[float, float]


def is_segment_clear(
        start: Point,
        end: Point,
        obstacles: list[Obstacle],
        clearance_m: float,
) -> bool:
    if clearance_m < 0.0:
        raise ValueError("Clearance cannot be negative.")

    start_x_m, start_y_m = start
    end_x_m, end_y_m = end

    segment_x_m = end_x_m - start_x_m
    segment_y_m = end_y_m - start_y_m

    segment_length_squared_m2 = (
        segment_x_m**2 + segment_y_m**2
    )

    for obstacle in obstacles:
        inflated_radius_m = (
            obstacle.radius_m + clearance_m
        )

        if segment_length_squared_m2 == 0.0:
            closest_x_m = start_x_m
            closest_y_m = start_y_m
        else:
            projection_fraction = (
                (
                    (obstacle.x_m - start_x_m)
                    * segment_x_m
                    + (obstacle.y_m - start_y_m)
                    * segment_y_m
                )
                / segment_length_squared_m2
            )

            projection_fraction = max(
                0.0,
                min(1.0, projection_fraction),
            )

            closest_x_m = (
                start_x_m
                + projection_fraction * segment_x_m
            )
            closest_y_m = (
                start_y_m
                + projection_fraction * segment_y_m
            )

        distance_squared_m2 = (
            (closest_x_m - obstacle.x_m) ** 2
            + (closest_y_m - obstacle.y_m) ** 2
        )

        if distance_squared_m2 <= inflated_radius_m**2:
            return False

    return True

def _distance(first: Point, second: Point) -> float:
    return math.hypot(
        second[0] - first[0],
        second[1] - first[1],
    )


def _generate_candidate_points(
        obstacles: list[Obstacle],
        clearance_m: float,
        samples_per_obstacle: int,
        candidate_offset_m: float,
) -> list[Point]:
    candidates = []

    angle_step_rad = (
        2.0 * math.pi / samples_per_obstacle
    )

    for obstacle in obstacles:
        inflated_radius_m = (
            obstacle.radius_m + clearance_m
        )

        sampling_radius_m = (
            inflated_radius_m
            / math.cos(math.pi / samples_per_obstacle)
            + candidate_offset_m
        )

        for sample_index in range(samples_per_obstacle):
            angle_rad = sample_index * angle_step_rad

            candidate = (
                obstacle.x_m
                + sampling_radius_m * math.cos(angle_rad),
                obstacle.y_m
                + sampling_radius_m * math.sin(angle_rad),
            )

            if is_segment_clear(
                    start=candidate,
                    end=candidate,
                    obstacles=obstacles,
                    clearance_m=clearance_m,
            ):
                candidates.append(candidate)

    return candidates


def find_route(
        start: Point,
        destination: Point,
        obstacles: list[Obstacle],
        clearance_m: float,
        samples_per_obstacle: int = 24,
        candidate_offset_m: float = 1.0,
) -> list[Point] | None:
    if clearance_m < 0.0:
        raise ValueError("Clearance cannot be negative.")

    if samples_per_obstacle < 8:
        raise ValueError(
            "At least eight samples per obstacle are required."
        )

    if candidate_offset_m <= 0.0:
        raise ValueError(
            "Candidate offset must be positive."
        )

    if not is_segment_clear(
            start=start,
            end=start,
            obstacles=obstacles,
            clearance_m=clearance_m,
    ):
        return None

    if not is_segment_clear(
            start=destination,
            end=destination,
            obstacles=obstacles,
            clearance_m=clearance_m,
    ):
        return None

    nodes = [
        start,
        destination,
        *_generate_candidate_points(
            obstacles=obstacles,
            clearance_m=clearance_m,
            samples_per_obstacle=samples_per_obstacle,
            candidate_offset_m=candidate_offset_m,
        ),
    ]

    neighbors: list[list[tuple[int, float]]] = [
        [] for _ in nodes
    ]

    for first_index in range(len(nodes)):
        for second_index in range(
                first_index + 1,
                len(nodes),
        ):
            if not is_segment_clear(
                    start=nodes[first_index],
                    end=nodes[second_index],
                    obstacles=obstacles,
                    clearance_m=clearance_m,
            ):
                continue

            edge_length_m = _distance(
                nodes[first_index],
                nodes[second_index],
            )

            neighbors[first_index].append(
                (second_index, edge_length_m)
            )
            neighbors[second_index].append(
                (first_index, edge_length_m)
            )

    start_index = 0
    destination_index = 1

    lowest_costs = [math.inf] * len(nodes)
    lowest_costs[start_index] = 0.0

    previous_nodes: dict[int, int] = {}
    insertion_order = count()

    open_nodes = [
        (
            _distance(start, destination),
            0.0,
            next(insertion_order),
            start_index,
        )
    ]

    while open_nodes:
        (
            _,
            current_cost_m,
            _,
            current_index,
        ) = heapq.heappop(open_nodes)

        if current_cost_m > lowest_costs[current_index]:
            continue

        if current_index == destination_index:
            route_indices = [destination_index]

            while route_indices[-1] != start_index:
                route_indices.append(
                    previous_nodes[route_indices[-1]]
                )

            route_indices.reverse()

            return [
                nodes[node_index]
                for node_index in route_indices[1:]
            ]

        for neighbor_index, edge_length_m in (
                neighbors[current_index]
        ):
            candidate_cost_m = (
                current_cost_m + edge_length_m
            )

            if (
                    candidate_cost_m
                    >= lowest_costs[neighbor_index]
            ):
                continue

            lowest_costs[neighbor_index] = candidate_cost_m
            previous_nodes[neighbor_index] = current_index

            estimated_total_cost_m = (
                candidate_cost_m
                + _distance(
                    nodes[neighbor_index],
                    destination,
                )
            )

            heapq.heappush(
                open_nodes,
                (
                    estimated_total_cost_m,
                    candidate_cost_m,
                    next(insertion_order),
                    neighbor_index,
                ),
            )

    return None