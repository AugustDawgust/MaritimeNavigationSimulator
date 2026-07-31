import config
from avoidance import (
    calculate_avoidance_waypoint,
    find_nearest_blocking_obstacle,
)
from collision import vessel_collides_with_any_obstacle
from navigation import (
    calculate_desired_heading,
    calculate_distance,
    turn_toward_heading,
)
from obstacle import Obstacle
from vessel import Vessel


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

        self.avoidance_waypoint: tuple[float, float] | None = None

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
        if self.arrived or self.collided:
            return

        if (
            self.distance_to_destination_m
            <= config.ARRIVAL_RADIUS_M
        ):
            self.arrived = True
            self.vessel.speed_mps = 0.0
            return

        if self.avoidance_waypoint is not None:
            waypoint_x_m, waypoint_y_m = self.avoidance_waypoint

            distance_to_waypoint_m = calculate_distance(
                self.vessel.x_m,
                self.vessel.y_m,
                waypoint_x_m,
                waypoint_y_m,
            )

            if (
                distance_to_waypoint_m
                <= config.AVOIDANCE_WAYPOINT_RADIUS_M
            ):
                self.avoidance_waypoint = None

        if self.avoidance_waypoint is None:
            blocking_obstacle = find_nearest_blocking_obstacle(
                self.vessel.x_m,
                self.vessel.y_m,
                self.destination_x_m,
                self.destination_y_m,
                self.obstacles,
                config.AVOIDANCE_CLEARANCE_M,
            )

            if blocking_obstacle is not None:
                self.avoidance_waypoint = (
                    calculate_avoidance_waypoint(
                        self.vessel.x_m,
                        self.vessel.y_m,
                        self.destination_x_m,
                        self.destination_y_m,
                        blocking_obstacle,
                        config.AVOIDANCE_CLEARANCE_M,
                        config.AVOIDANCE_EXTRA_OFFSET_M,
                    )
                )

        if self.avoidance_waypoint is None:
            target_x_m = self.destination_x_m
            target_y_m = self.destination_y_m
        else:
            target_x_m, target_y_m = self.avoidance_waypoint

        desired_heading_deg = calculate_desired_heading(
            self.vessel.x_m,
            self.vessel.y_m,
            target_x_m,
            target_y_m,
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

        if (
            distance_from_last_point
            >= config.TRAIL_POINT_SPACING_M
        ):
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