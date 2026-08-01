import math
import random

from obstacle import Obstacle


def generate_obstacles(
        *,
        seed: int,
        obstacle_count: int,
        world_width_m: float,
        world_height_m: float,
        min_radius_m: float,
        max_radius_m: float,
        edge_clearance_m: float,
        obstacle_spacing_m: float,
        protected_points: list[tuple[float, float, float]],
        max_attempts: int,
) -> list[Obstacle]:
    if obstacle_count < 0:
        raise ValueError("Obstacle count cannot be negative.")

    if world_width_m <= 0.0 or world_height_m <= 0.0:
        raise ValueError("World dimensions must be positive.")

    if min_radius_m <= 0.0:
        raise ValueError("Minimum obstacle radius must be positive.")

    if max_radius_m < min_radius_m:
        raise ValueError(
            "Maximum obstacle radius cannot be smaller "
            "than the minimum radius."
        )

    if edge_clearance_m < 0.0:
        raise ValueError("Edge clearance cannot be negative.")

    if obstacle_spacing_m < 0.0:
        raise ValueError("Obstacle spacing cannot be negative.")

    if max_attempts <= 0:
        raise ValueError("Maximum attempts must be positive.")

    random_generator = random.Random(seed)
    obstacles: list[Obstacle] = []

    half_world_width_m = world_width_m / 2.0
    half_world_height_m = world_height_m / 2.0

    for _ in range(max_attempts):
        if len(obstacles) == obstacle_count:
            return obstacles

        radius_m = random_generator.uniform(
            min_radius_m,
            max_radius_m,
        )

        maximum_x_m = (
            half_world_width_m
            - edge_clearance_m
            - radius_m
        )
        maximum_y_m = (
            half_world_height_m
            - edge_clearance_m
            - radius_m
        )

        if maximum_x_m < 0.0 or maximum_y_m < 0.0:
            continue

        candidate = Obstacle(
            x_m=random_generator.uniform(
                -maximum_x_m,
                maximum_x_m,
            ),
            y_m=random_generator.uniform(
                -maximum_y_m,
                maximum_y_m,
            ),
            radius_m=radius_m,
        )

        if _overlaps_protected_point(
                candidate,
                protected_points,
        ):
            continue

        if _overlaps_existing_obstacle(
                candidate,
                obstacles,
                obstacle_spacing_m,
        ):
            continue

        obstacles.append(candidate)

    raise RuntimeError(
        "Unable to generate a valid obstacle layout "
        "within the maximum number of attempts."
    )


def _overlaps_protected_point(
        candidate: Obstacle,
        protected_points: list[tuple[float, float, float]],
) -> bool:
    for point_x_m, point_y_m, clearance_m in protected_points:
        if clearance_m < 0.0:
            raise ValueError(
                "Protected-point clearance cannot be negative."
            )

        distance_m = math.hypot(
            candidate.x_m - point_x_m,
            candidate.y_m - point_y_m,
        )

        if distance_m <= candidate.radius_m + clearance_m:
            return True

    return False


def _overlaps_existing_obstacle(
        candidate: Obstacle,
        obstacles: list[Obstacle],
        obstacle_spacing_m: float,
) -> bool:
    for obstacle in obstacles:
        distance_m = math.hypot(
            candidate.x_m - obstacle.x_m,
            candidate.y_m - obstacle.y_m,
        )

        required_distance_m = (
            candidate.radius_m
            + obstacle.radius_m
            + obstacle_spacing_m
        )

        if distance_m <= required_distance_m:
            return True

    return False