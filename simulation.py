import config
import math

from collision import vessel_collides_with_any_obstacle
from navigation import (
    calculate_desired_heading,
    calculate_distance,
    calculate_guidance_speed,
    turn_toward_heading,
)
from obstacle import Obstacle
from vessel import Vessel
from mission_metrics import MissionMetrics
from scenario import generate_obstacles
from route_planner import find_route

class Simulation:
    def __init__(self) -> None:
        self.vessel = Vessel(
            x_m=config.INITIAL_VESSEL_X_M,
            y_m=config.INITIAL_VESSEL_Y_M,
            heading_deg=config.INITIAL_VESSEL_HEADING_DEG,
            speed_mps=config.INITIAL_VESSEL_SPEED_MPS,
        )
        self.route_waypoints: list[
            tuple[float, float]
        ] = []
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
        self.scenario_seed: int | None = None

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
        if self.route_waypoints:
            return self.route_waypoints[0]

        return (
            self.destination_x_m,
            self.destination_y_m,
        )
    def is_destination_valid(
        self,
        destination_x_m: float,
        destination_y_m: float,
    ) -> bool:
        for obstacle in self.obstacles:
            distance_to_obstacle_m = calculate_distance(
                destination_x_m,
                destination_y_m,
                obstacle.x_m,
                obstacle.y_m,
            )

            required_clearance_m = (
                obstacle.radius_m
                + config.ROUTE_CLEARANCE_M
            )

            if distance_to_obstacle_m <= required_clearance_m:
                return False

        return True

    def set_destination(
            self,
            destination_x_m: float,
            destination_y_m: float,
    ) -> bool:
        if not self.is_destination_valid(
                destination_x_m,
                destination_y_m,
        ):
            return False

        self.destination_x_m = destination_x_m
        self.destination_y_m = destination_y_m

        self.arrived = False
        self.collided = False
        self.navigation_blocked = False

        self.vessel.speed_mps = config.INITIAL_VESSEL_SPEED_MPS

        self.trail_points = [
            (
                self.vessel.x_m,
                self.vessel.y_m,
            )
        ]

        self.metrics = MissionMetrics(
            base_fuel_burn_rate_lph=(
                config.BASE_FUEL_BURN_RATE_LPH
            ),
            speed_cubed_coefficient=(
                config.SPEED_CUBED_COEFFICIENT
            ),
        )
        self.route_waypoints = []

        return self.plan_route()
    def reset_scenario(self, seed: int) -> None:
        new_obstacles = generate_obstacles(
            seed=seed,
            obstacle_count=config.SCENARIO_OBSTACLE_COUNT,
            world_width_m=(
                config.WINDOW_WIDTH
                / config.PIXELS_PER_METER
            ),
            world_height_m=(
                config.WINDOW_HEIGHT
                / config.PIXELS_PER_METER
            ),
            min_radius_m=config.SCENARIO_MIN_RADIUS_M,
            max_radius_m=config.SCENARIO_MAX_RADIUS_M,
            edge_clearance_m=(
                config.SCENARIO_EDGE_CLEARANCE_M
            ),
            obstacle_spacing_m=(
                config.SCENARIO_OBSTACLE_SPACING_M
            ),
            protected_points=[
                (
                    config.INITIAL_VESSEL_X_M,
                    config.INITIAL_VESSEL_Y_M,
                    config.SCENARIO_START_CLEARANCE_M,
                ),
                (
                    config.DESTINATION_X_M,
                    config.DESTINATION_Y_M,
                    config.SCENARIO_DESTINATION_CLEARANCE_M,
                ),
            ],
            max_attempts=(
                config.SCENARIO_MAX_GENERATION_ATTEMPTS
            ),
        )

        self.vessel = Vessel(
            x_m=config.INITIAL_VESSEL_X_M,
            y_m=config.INITIAL_VESSEL_Y_M,
            heading_deg=config.INITIAL_VESSEL_HEADING_DEG,
            speed_mps=config.INITIAL_VESSEL_SPEED_MPS,
        )

        self.route_waypoints = []

        self.destination_x_m = config.DESTINATION_X_M
        self.destination_y_m = config.DESTINATION_Y_M

        self.arrived = False
        self.collided = False
        self.navigation_blocked = False

        self.obstacles = new_obstacles
        self.scenario_seed = seed

        self.trail_points = [
            (
                self.vessel.x_m,
                self.vessel.y_m,
            )
        ]

        self.metrics = MissionMetrics(
            base_fuel_burn_rate_lph=(
                config.BASE_FUEL_BURN_RATE_LPH
            ),
            speed_cubed_coefficient=(
                config.SPEED_CUBED_COEFFICIENT
            ),
        )
    def plan_route(self) -> bool:

        planned_route = find_route(
            start=(
                self.vessel.x_m,
                self.vessel.y_m,
            ),
            destination=(
                self.destination_x_m,
                self.destination_y_m,
            ),
            obstacles=self.obstacles,
            clearance_m=config.ROUTE_CLEARANCE_M,
        )

        if planned_route is None:
            self.route_waypoints = []
            self.navigation_blocked = True
            self.vessel.speed_mps = 0.0
            return False

        self.route_waypoints = planned_route
        self.navigation_blocked = False
        return True
    def _advance_reached_route_waypoints(self) -> None:
        while self.route_waypoints:
            waypoint_x_m, waypoint_y_m = (
                self.route_waypoints[0]
            )

            distance_to_waypoint_m = math.hypot(
                waypoint_x_m - self.vessel.x_m,
                waypoint_y_m - self.vessel.y_m,
            )

            if (
                    distance_to_waypoint_m
                    > config.ARRIVAL_RADIUS_M
            ):
                break

            self.route_waypoints.pop(0)

    def update(self, dt_s: float) -> None:
        self._advance_reached_route_waypoints()

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

        if (
                self.distance_to_destination_m
                <= config.ARRIVAL_RADIUS_M
        ):
            self.arrived = True
            self.vessel.speed_mps = 0.0
            return

        if not self.route_waypoints:
            if not self.plan_route():
                return

        target_x_m, target_y_m = self.active_target

        desired_heading_deg = calculate_desired_heading(
            self.vessel.x_m,
            self.vessel.y_m,
            target_x_m,
            target_y_m,
        )
        distance_to_active_target_m = calculate_distance(
            self.vessel.x_m,
            self.vessel.y_m,
            target_x_m,
            target_y_m,
        )

        self.vessel.speed_mps = calculate_guidance_speed(
            current_heading_deg=self.vessel.heading_deg,
            desired_heading_deg=desired_heading_deg,
            distance_to_target_m=distance_to_active_target_m,
            cruise_speed_mps=(
                config.INITIAL_VESSEL_SPEED_MPS
            ),
            max_turn_rate_deg_s=(
                config.MAX_TURN_RATE_DEG_S
            ),
            minimum_speed_mps=(
                config.MIN_GUIDANCE_SPEED_MPS
            ),
            turn_radius_factor=(
                config.GUIDANCE_TURN_RADIUS_FACTOR
            ),
            minimum_turn_demand=(
                config.GUIDANCE_MIN_TURN_DEMAND
            ),
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