import math

def calculate_desired_heading(
    current_x_m: float,
    current_y_m: float,
    destination_x_m: float,
    destination_y_m: float,
) -> float:
    delta_x = destination_x_m - current_x_m
    delta_y = destination_y_m - current_y_m

    heading_deg = math.degrees(math.atan2(delta_x, delta_y))
    return heading_deg % 360.0

def calculate_heading_error(
    current_heading_deg: float,
    desired_heading_deg: float,
) -> float:
    return (
        desired_heading_deg - current_heading_deg + 180.0
    ) % 360.0 - 180.0

def turn_toward_heading(
    current_heading_deg: float,
    desired_heading_deg: float,
    max_turn_rate_deg_s: float,
    dt_s: float,
) -> float:
    heading_error = calculate_heading_error(
        current_heading_deg,
        desired_heading_deg,
    )
    maximum_turn = max_turn_rate_deg_s * dt_s

    limited_turn = max(
        -maximum_turn,
        min(heading_error, maximum_turn),
    )
    return (current_heading_deg + limited_turn) % 360.0

def calculate_distance(
    current_x_m: float,
    current_y_m: float,
    destination_x_m: float,
    destination_y_m: float,
) -> float:
    delta_x = destination_x_m - current_x_m
    delta_y = destination_y_m - current_y_m

    return math.hypot(delta_x, delta_y)
def calculate_guidance_speed(
        current_heading_deg: float,
        desired_heading_deg: float,
        distance_to_target_m: float,
        cruise_speed_mps: float,
        max_turn_rate_deg_s: float,
        minimum_speed_mps: float,
        turn_radius_factor: float,
        minimum_turn_demand: float,
) -> float:
    if distance_to_target_m <= 0.0:
        return 0.0

    heading_error_deg = abs(
        (
            desired_heading_deg
            - current_heading_deg
            + 180.0
        )
        % 360.0
        - 180.0
    )

    turn_demand = max(
        heading_error_deg / 90.0,
        minimum_turn_demand,
    )

    max_turn_rate_rad_s = math.radians(
        max_turn_rate_deg_s
    )

    allowed_turn_radius_m = (
        distance_to_target_m
        * turn_radius_factor
        / turn_demand
    )

    turn_limited_speed_mps = (
        max_turn_rate_rad_s
        * allowed_turn_radius_m
    )

    return min(
        cruise_speed_mps,
        max(
            minimum_speed_mps,
            turn_limited_speed_mps,
        ),
    )
def has_reached_or_passed_waypoint(
        current_x_m: float,
        current_y_m: float,
        waypoint_x_m: float,
        waypoint_y_m: float,
        onward_target_x_m: float,
        onward_target_y_m: float,
        waypoint_radius_m: float,
) -> bool:
    if waypoint_radius_m < 0.0:
        raise ValueError("Waypoint radius cannot be negative.")

    distance_to_waypoint_m = calculate_distance(
        current_x_m,
        current_y_m,
        waypoint_x_m,
        waypoint_y_m,
    )

    if distance_to_waypoint_m <= waypoint_radius_m:
        return True

    onward_x_m = onward_target_x_m - waypoint_x_m
    onward_y_m = onward_target_y_m - waypoint_y_m

    if onward_x_m == 0.0 and onward_y_m == 0.0:
        return False

    position_from_waypoint_x_m = (
        current_x_m - waypoint_x_m
    )
    position_from_waypoint_y_m = (
        current_y_m - waypoint_y_m
    )

    onward_progress_m2 = (
        position_from_waypoint_x_m * onward_x_m
        + position_from_waypoint_y_m * onward_y_m
    )

    return onward_progress_m2 > 0.0