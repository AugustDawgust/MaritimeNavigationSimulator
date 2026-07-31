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