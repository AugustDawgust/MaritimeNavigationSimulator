import config
from navigation import (
    calculate_desired_heading,
    calculate_distance,
    turn_toward_heading,
)
from vessel import Vessel
from collision import vessel_collides_with_any_obstacle
from obstacle import Obstacle

class Simulation:
    def __init__(self) -> None:
        self.vessel = Vessel(
            x_m=config.INITIAL_VESSEL_X_M,
            y_m=config.INITIAL_VESSEL_Y_M,
            heading_deg=config.INITIAL_VESSEL_HEADING_DEG,
            speed_mps=config.INITIAL_VESSEL_SPEED_MPS,
        )

        self.destination_x_m = config.DESTINATION_X_M
        self.destination_y_m = config.DESTINATION_Y_M
        self.arrived = False
        self.collided = False

        self.obstacles = [
            Obstacle(
                x_m=x_m,
                y_m=y_m,
                radius_m=radius_m,
            )
            for x_m, y_m, radius_m in config.OBSTACLES
        ]

        self.trail_points = [
            (self.vessel.x_m, self.vessel.y_m)
        ]

    @property
    def distance_to_destination_m(self) -> float:
        return calculate_distance(
            self.vessel.x_m,
            self.vessel.y_m,
            self.destination_x_m,
            self.destination_y_m,
        )

    def update(self, dt_s: float) -> None:
        distance_to_destination = self.distance_to_destination_m

        if self.arrived or self.collided:
            return

        if distance_to_destination <= config.ARRIVAL_RADIUS_M:
            self.arrived = True
            self.vessel.speed_mps = 0.0
            return

        desired_heading_deg = calculate_desired_heading(
            self.vessel.x_m,
            self.vessel.y_m,
            self.destination_x_m,
            self.destination_y_m,
        )

        self.vessel.heading_deg = turn_toward_heading(
            self.vessel.heading_deg,
            desired_heading_deg,
            config.MAX_TURN_RATE_DEG_S,
            dt_s,
        )

        self.vessel.update(dt_s)
        last_x_m, last_y_m = self.trail_points[-1]

        distance_from_last_point = calculate_distance(
            last_x_m,
            last_y_m,
            self.vessel.x_m,
            self.vessel.y_m,
        )

        if distance_from_last_point >= config.TRAIL_POINT_SPACING_M:
            self.trail_points.append(
                (self.vessel.x_m, self.vessel.y_m)
            )

        if vessel_collides_with_any_obstacle(
            self.vessel,
            config.VESSEL_COLLISION_RADIUS_M,
            self.obstacles,
        ):
            self.collided = True
            self.vessel.speed_mps = 0.0