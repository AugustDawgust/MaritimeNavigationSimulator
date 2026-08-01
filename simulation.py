import config
from avoidance import (
    find_nearest_blocking_obstacle,
    find_safe_avoidance_waypoint,
)
from collision import vessel_collides_with_any_obstacle
from navigation import (
    calculate_desired_heading,
    calculate_distance,
    turn_toward_heading,
)
from obstacle import Obstacle
from vessel import Vessel
from mission_metrics import MissionMetrics

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
        self.navigation_blocked = False
        self.metrics = MissionMetrics(
            base_fuel_burn_rate_lph=(
                config.BASE_FUEL_BURN_RATE_LPH
            ),
            speed_cubed_coefficient=(
                config.SPEED_CUBED_COEFFICIENT
            ),
        )

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

    @property
    def active_target(self) -> tuple[float, float]:
        if self.avoidance_waypoint is not None:
            return self.avoidance_waypoint

        return (
            self.destination_x_m,
            self.destination_y_m,
        )
    def update(self, dt_s: float) -> None:
        if (
                self.arrived
                or self.collided
                or self.navigation_blocked
        ):
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
                safe_waypoint = find_safe_avoidance_waypoint(
                    self.vessel.x_m,
                    self.vessel.y_m,
                    self.destination_x_m,
                    self.destination_y_m,
                    blocking_obstacle,
                    self.obstacles,
                    config.AVOIDANCE_CLEARANCE_M,
                    config.AVOIDANCE_EXTRA_OFFSET_M,
                )

                if safe_waypoint is None:
                    self.navigation_blocked = True
                    self.vessel.speed_mps = 0.0
                    return

                self.avoidance_waypoint = safe_waypoint

        target_x_m, target_y_m = self.active_target

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

        previous_x_m = self.vessel.x_m
        previous_y_m = self.vessel.y_m
        movement_speed_mps = self.vessel.speed_mps

        self.vessel.update(dt_s)

        distance_traveled_m = calculate_distance(
            previous_x_m,
            previous_y_m,
            self.vessel.x_m,
            self.vessel.y_m,
        )

        self.metrics.update(
            distance_traveled_m=distance_traveled_m,
            speed_mps=movement_speed_mps,
            dt_s=dt_s,
        )

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