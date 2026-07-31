import math

from obstacle import Obstacle
from vessel import Vessel


def vessel_collides_with_obstacle(
    vessel: Vessel,
    vessel_collision_radius_m: float,
    obstacle: Obstacle,
) -> bool:
    center_distance_m = math.hypot(
        obstacle.x_m - vessel.x_m,
        obstacle.y_m - vessel.y_m,
    )

    combined_radius_m = (
        vessel_collision_radius_m + obstacle.radius_m
    )

    return center_distance_m <= combined_radius_m

def vessel_collides_with_any_obstacle(
    vessel: Vessel,
    vessel_collision_radius_m: float,
    obstacles: list[Obstacle],
) -> bool:
    return any(
        vessel_collides_with_obstacle(
            vessel,
            vessel_collision_radius_m,
            obstacle,
        )
        for obstacle in obstacles
    )