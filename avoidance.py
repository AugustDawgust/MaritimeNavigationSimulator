import math

from obstacle import Obstacle


def obstacle_blocks_route(
    start_x_m: float,
    start_y_m: float,
    destination_x_m: float,
    destination_y_m: float,
    obstacle: Obstacle,
    clearance_m: float,
) -> bool:
    route_x_m = destination_x_m - start_x_m
    route_y_m = destination_y_m - start_y_m
    route_length_squared = route_x_m**2 + route_y_m**2

    if route_length_squared == 0.0:
        return False

    obstacle_x_m = obstacle.x_m - start_x_m
    obstacle_y_m = obstacle.y_m - start_y_m

    projection = (
        obstacle_x_m * route_x_m
        + obstacle_y_m * route_y_m
    ) / route_length_squared

    if projection < 0.0 or projection > 1.0:
        return False

    closest_x_m = start_x_m + projection * route_x_m
    closest_y_m = start_y_m + projection * route_y_m

    distance_to_route_m = math.hypot(
        obstacle.x_m - closest_x_m,
        obstacle.y_m - closest_y_m,
    )

    required_clearance_m = obstacle.radius_m + clearance_m

    return distance_to_route_m <= required_clearance_m

def find_nearest_blocking_obstacle(
    start_x_m: float,
    start_y_m: float,
    destination_x_m: float,
    destination_y_m: float,
    obstacles: list[Obstacle],
    clearance_m: float,
) -> Obstacle | None:
    blocking_obstacles = [
        obstacle
        for obstacle in obstacles
        if obstacle_blocks_route(
            start_x_m,
            start_y_m,
            destination_x_m,
            destination_y_m,
            obstacle,
            clearance_m,
        )
    ]

    if not blocking_obstacles:
        return None

    return min(
        blocking_obstacles,
        key=lambda obstacle: math.hypot(
            obstacle.x_m - start_x_m,
            obstacle.y_m - start_y_m,
        ),
    )
def calculate_avoidance_waypoint(
    start_x_m: float,
    start_y_m: float,
    destination_x_m: float,
    destination_y_m: float,
    obstacle: Obstacle,
    clearance_m: float,
    extra_offset_m: float,
) -> tuple[float, float]:
    route_x_m = destination_x_m - start_x_m
    route_y_m = destination_y_m - start_y_m
    route_length_m = math.hypot(route_x_m, route_y_m)

    if route_length_m == 0.0:
        raise ValueError(
            "Cannot calculate a waypoint for a zero-length route."
        )

    perpendicular_x = -route_y_m / route_length_m
    perpendicular_y = route_x_m / route_length_m

    waypoint_offset_m = (
        obstacle.radius_m
        + clearance_m
        + extra_offset_m
    )

    waypoint_x_m = (
        obstacle.x_m
        + perpendicular_x * waypoint_offset_m
    )
    waypoint_y_m = (
        obstacle.y_m
        + perpendicular_y * waypoint_offset_m
    )

    return waypoint_x_m, waypoint_y_m
def find_safe_avoidance_waypoint(
    start_x_m: float,
    start_y_m: float,
    destination_x_m: float,
    destination_y_m: float,
    blocking_obstacle: Obstacle,
    obstacles: list[Obstacle],
    clearance_m: float,
    extra_offset_m: float,
) -> tuple[float, float] | None:
    preferred_waypoint = calculate_avoidance_waypoint(
        start_x_m,
        start_y_m,
        destination_x_m,
        destination_y_m,
        blocking_obstacle,
        clearance_m,
        extra_offset_m,
    )

    preferred_x_m, preferred_y_m = preferred_waypoint

    opposite_waypoint = (
        2.0 * blocking_obstacle.x_m - preferred_x_m,
        2.0 * blocking_obstacle.y_m - preferred_y_m,
    )

    for waypoint_x_m, waypoint_y_m in (
        preferred_waypoint,
        opposite_waypoint,
    ):
        waypoint_is_safe = all(
            not obstacle_blocks_route(
                start_x_m,
                start_y_m,
                waypoint_x_m,
                waypoint_y_m,
                obstacle,
                clearance_m,
            )
            for obstacle in obstacles
        )

        if waypoint_is_safe:
            return waypoint_x_m, waypoint_y_m

    return None