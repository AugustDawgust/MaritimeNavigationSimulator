import config
from avoidance import (
    calculate_avoidance_waypoint,
    obstacle_blocks_route,
    predict_vessel_trajectory,
    trajectory_collides_with_obstacle,
)
from collision import vessel_collides_with_any_obstacle
from mission_metrics import MissionMetrics
from mission_route import MissionRoute
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

        self.route = MissionRoute(
            waypoints=config.MISSION_WAYPOINTS
        )

        initial_waypoint = self.route.current_waypoint

        if initial_waypoint is None:
            raise RuntimeError(
                "The configured mission has no initial waypoint."
            )

        self.destination_x_m, self.destination_y_m = (
            initial_waypoint
        )

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

        self.avoidance_waypoint: (
            tuple[float, float] | None
        ) = None

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

    def _should_advance_mission_waypoint(self) -> bool:
        if (
                self.distance_to_destination_m
                <= config.ARRIVAL_RADIUS_M
        ):
            return True

        if self.avoidance_waypoint is not None:
            return False

        if self.route.next_waypoint is None:
            return False

        return (
                self.distance_to_destination_m
                <= config.WAYPOINT_TURN_ANTICIPATION_RADIUS_M
        )
    def _prediction_horizon_to_target(
        self,
        target_x_m: float,
        target_y_m: float,
    ) -> float:
        if self.vessel.speed_mps <= 0.0:
            return config.TRAJECTORY_PREDICTION_HORIZON_S

        distance_to_target_m = calculate_distance(
            self.vessel.x_m,
            self.vessel.y_m,
            target_x_m,
            target_y_m,
        )

        straight_travel_time_s = (
            distance_to_target_m
            / self.vessel.speed_mps
        )

        return min(
            config.TRAJECTORY_PREDICTION_HORIZON_S,
            max(
                config.TRAJECTORY_PREDICTION_STEP_S,
                straight_travel_time_s,
            ),
        )

    def _predict_trajectory_to(
        self,
        target_x_m: float,
        target_y_m: float,
    ) -> list[tuple[float, float]]:
        return predict_vessel_trajectory(
            self.vessel,
            target_x_m,
            target_y_m,
            config.MAX_TURN_RATE_DEG_S,
            self._prediction_horizon_to_target(
                target_x_m,
                target_y_m,
            ),
            config.TRAJECTORY_PREDICTION_STEP_S,
        )

    def _trajectory_is_clear_to(
        self,
        target_x_m: float,
        target_y_m: float,
    ) -> bool:
        trajectory_points = self._predict_trajectory_to(
            target_x_m,
            target_y_m,
        )

        for obstacle in self.obstacles:
            if trajectory_collides_with_obstacle(
                trajectory_points,
                obstacle,
                config.AVOIDANCE_CLEARANCE_M,
            ):
                return False

        final_predicted_x_m, final_predicted_y_m = (
            trajectory_points[-1]
        )

        for obstacle in self.obstacles:
            if obstacle_blocks_route(
                final_predicted_x_m,
                final_predicted_y_m,
                target_x_m,
                target_y_m,
                obstacle,
                config.AVOIDANCE_CLEARANCE_M,
            ):
                return False

        return True

    def _find_nearest_threatening_obstacle(
        self,
        target_x_m: float,
        target_y_m: float,
    ) -> Obstacle | None:
        trajectory_points = self._predict_trajectory_to(
            target_x_m,
            target_y_m,
        )

        threatening_obstacles = [
            obstacle
            for obstacle in self.obstacles
            if (
                obstacle_blocks_route(
                    self.vessel.x_m,
                    self.vessel.y_m,
                    target_x_m,
                    target_y_m,
                    obstacle,
                    config.AVOIDANCE_CLEARANCE_M,
                )
                or trajectory_collides_with_obstacle(
                    trajectory_points,
                    obstacle,
                    config.AVOIDANCE_CLEARANCE_M,
                )
            )
        ]

        if not threatening_obstacles:
            return None

        return min(
            threatening_obstacles,
            key=lambda obstacle: calculate_distance(
                self.vessel.x_m,
                self.vessel.y_m,
                obstacle.x_m,
                obstacle.y_m,
            ),
        )

    @staticmethod
    def _heading_change_deg(
        current_heading_deg: float,
        desired_heading_deg: float,
    ) -> float:
        return abs(
            (
                desired_heading_deg
                - current_heading_deg
                + 180.0
            )
            % 360.0
            - 180.0
        )

    def _select_safe_avoidance_waypoint(
        self,
        obstacle: Obstacle,
    ) -> tuple[float, float] | None:
        first_waypoint = calculate_avoidance_waypoint(
            self.vessel.x_m,
            self.vessel.y_m,
            self.destination_x_m,
            self.destination_y_m,
            obstacle,
            config.AVOIDANCE_CLEARANCE_M,
            config.AVOIDANCE_EXTRA_OFFSET_M,
        )

        opposite_waypoint = (
            2.0 * obstacle.x_m - first_waypoint[0],
            2.0 * obstacle.y_m - first_waypoint[1],
        )

        safe_waypoints = [
            waypoint
            for waypoint in (
                first_waypoint,
                opposite_waypoint,
            )
            if self._trajectory_is_clear_to(*waypoint)
        ]

        if not safe_waypoints:
            return None

        return min(
            safe_waypoints,
            key=lambda waypoint: (
                self._heading_change_deg(
                    self.vessel.heading_deg,
                    calculate_desired_heading(
                        self.vessel.x_m,
                        self.vessel.y_m,
                        waypoint[0],
                        waypoint[1],
                    ),
                ),
                calculate_distance(
                    self.vessel.x_m,
                    self.vessel.y_m,
                    waypoint[0],
                    waypoint[1],
                ),
            ),
        )

    def update(self, dt_s: float) -> None:
        if self.arrived or self.collided:
            return

        if vessel_collides_with_any_obstacle(
                self.vessel,
                config.VESSEL_COLLISION_RADIUS_M,
                self.obstacles,
        ):
            self.collided = True
            self.vessel.speed_mps = 0.0
            return

        if self.navigation_blocked:
            return

        if self._should_advance_mission_waypoint():
            self.route.advance()
            self.avoidance_waypoint = None

            next_waypoint = self.route.current_waypoint

            if next_waypoint is None:
                self.arrived = True
                self.vessel.speed_mps = 0.0
                return

            (
                self.destination_x_m,
                self.destination_y_m,
            ) = next_waypoint

        if self.avoidance_waypoint is not None:
            waypoint_x_m, waypoint_y_m = (
                self.avoidance_waypoint
            )

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
            threatening_obstacle = (
                self._find_nearest_threatening_obstacle(
                    self.destination_x_m,
                    self.destination_y_m,
                )
            )

            if threatening_obstacle is not None:
                selected_waypoint = (
                    self._select_safe_avoidance_waypoint(
                        threatening_obstacle
                    )
                )

                if selected_waypoint is None:
                    self.navigation_blocked = True
                    self.vessel.speed_mps = 0.0
                    return

                self.avoidance_waypoint = selected_waypoint

        if self.avoidance_waypoint is None:
            target_x_m = self.destination_x_m
            target_y_m = self.destination_y_m
        else:
            target_x_m, target_y_m = (
                self.avoidance_waypoint
            )

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