import math

from navigation import (
    calculate_desired_heading,
    turn_toward_heading,
)
from obstacle import Obstacle
from vessel import Vessel


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


def predict_vessel_trajectory(
    vessel: Vessel,
    target_x_m: float,
    target_y_m: float,
    max_turn_rate_deg_s: float,
    prediction_horizon_s: float,
    prediction_step_s: float,
) -> list[tuple[float, float]]:
    if max_turn_rate_deg_s < 0.0:
        raise ValueError(
            "Maximum turn rate cannot be negative."
        )

    if prediction_horizon_s < 0.0:
        raise ValueError(
            "Prediction horizon cannot be negative."
        )

    if prediction_step_s <= 0.0:
        raise ValueError(
            "Prediction step must be greater than zero."
        )

    predicted_x_m = vessel.x_m
    predicted_y_m = vessel.y_m
    predicted_heading_deg = vessel.heading_deg
    elapsed_time_s = 0.0

    trajectory_points = [
        (predicted_x_m, predicted_y_m)
    ]

    while elapsed_time_s < prediction_horizon_s:
        dt_s = min(
            prediction_step_s,
            prediction_horizon_s - elapsed_time_s,
        )

        desired_heading_deg = calculate_desired_heading(
            predicted_x_m,
            predicted_y_m,
            target_x_m,
            target_y_m,
        )

        predicted_heading_deg = turn_toward_heading(
            predicted_heading_deg,
            desired_heading_deg,
            max_turn_rate_deg_s,
            dt_s,
        )

        heading_rad = math.radians(predicted_heading_deg)

        predicted_x_m += (
            vessel.speed_mps
            * math.sin(heading_rad)
            * dt_s
        )
        predicted_y_m += (
            vessel.speed_mps
            * math.cos(heading_rad)
            * dt_s
        )

        trajectory_points.append(
            (predicted_x_m, predicted_y_m)
        )

        elapsed_time_s += dt_s

    return trajectory_points


def trajectory_collides_with_obstacle(
    trajectory_points: list[tuple[float, float]],
    obstacle: Obstacle,
    clearance_m: float,
) -> bool:
    required_clearance_m = obstacle.radius_m + clearance_m

    for point_x_m, point_y_m in trajectory_points:
        distance_to_obstacle_m = math.hypot(
            obstacle.x_m - point_x_m,
            obstacle.y_m - point_y_m,
        )

        if distance_to_obstacle_m <= required_clearance_m:
            return True

    for start_point, end_point in zip(
        trajectory_points,
        trajectory_points[1:],
    ):
        if obstacle_blocks_route(
            start_point[0],
            start_point[1],
            end_point[0],
            end_point[1],
            obstacle,
            clearance_m,
        ):
            return True

    return False