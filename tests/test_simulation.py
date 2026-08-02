import unittest

from obstacle import Obstacle
from simulation import Simulation
from navigation import calculate_distance
import config


class TestSimulation(unittest.TestCase):
    def test_collision_stops_vessel(self) -> None:
        simulation = Simulation()
        simulation.obstacles = [
            Obstacle(
                x_m=0.0,
                y_m=8.0,
                radius_m=2.0,
            )
        ]

        simulation.update(0.1)

        self.assertTrue(simulation.collided)
        self.assertEqual(simulation.vessel.speed_mps, 0.0)

    def test_vessel_remains_stopped_after_collision(self) -> None:
        simulation = Simulation()
        simulation.obstacles = [
            Obstacle(
                x_m=0.0,
                y_m=8.0,
                radius_m=2.0,
            )
        ]

        simulation.update(0.1)

        collision_x_m = simulation.vessel.x_m
        collision_y_m = simulation.vessel.y_m

        simulation.update(1.0)

        self.assertAlmostEqual(
            simulation.vessel.x_m,
            collision_x_m,
        )
        self.assertAlmostEqual(
            simulation.vessel.y_m,
            collision_y_m,
        )

    def test_no_collision_when_obstacle_list_is_empty(self) -> None:
        simulation = Simulation()
        simulation.obstacles = []

        simulation.update(1.0)

        self.assertFalse(simulation.collided)
        self.assertEqual(
            simulation.vessel.speed_mps,
            config.INITIAL_VESSEL_SPEED_MPS,
        )

    def test_update_plans_route_around_blocking_obstacle(
            self,
    ) -> None:
        simulation = Simulation()
        simulation.vessel.x_m = 0.0
        simulation.vessel.y_m = 0.0
        simulation.destination_x_m = 100.0
        simulation.destination_y_m = 0.0
        simulation.obstacles = [
            Obstacle(
                x_m=50.0,
                y_m=0.0,
                radius_m=5.0,
            )
        ]
        simulation.route_waypoints = []

        simulation.update(0.0)

        self.assertGreater(
            len(simulation.route_waypoints),
            1,
        )
        self.assertEqual(
            simulation.route_waypoints[-1],
            (100.0, 0.0),
        )
        self.assertFalse(simulation.navigation_blocked)

    def test_update_plans_direct_route_when_clear(
            self,
    ) -> None:
        simulation = Simulation()
        simulation.vessel.x_m = 0.0
        simulation.vessel.y_m = 0.0
        simulation.destination_x_m = 100.0
        simulation.destination_y_m = 0.0
        simulation.obstacles = []
        simulation.route_waypoints = []

        simulation.update(0.0)

        self.assertEqual(
            simulation.route_waypoints,
            [(100.0, 0.0)],
        )
        self.assertFalse(simulation.navigation_blocked)

    def test_navigates_around_obstacle_and_arrives(self) -> None:
        simulation = Simulation()
        simulation.vessel.x_m = 0.0
        simulation.vessel.y_m = 0.0
        simulation.vessel.heading_deg = 90.0
        simulation.destination_x_m = 100.0
        simulation.destination_y_m = 0.0
        simulation.obstacles = [
            Obstacle(
                x_m=50.0,
                y_m=0.0,
                radius_m=5.0,
            )
        ]
        simulation.trail_points = [(0.0, 0.0)]

        for _ in range(2000):
            simulation.update(0.05)

            if simulation.arrived or simulation.collided:
                break

        self.assertTrue(simulation.arrived)
        self.assertFalse(simulation.collided)
        self.assertEqual(simulation.vessel.speed_mps, 0.0)

    def test_navigates_around_multiple_obstacles_and_arrives(
            self,
    ) -> None:
        simulation = Simulation()
        simulation.vessel.x_m = 0.0
        simulation.vessel.y_m = 0.0
        simulation.vessel.heading_deg = 90.0
        simulation.destination_x_m = 120.0
        simulation.destination_y_m = 0.0
        simulation.obstacles = [
            Obstacle(
                x_m=40.0,
                y_m=0.0,
                radius_m=5.0,
            ),
            Obstacle(
                x_m=80.0,
                y_m=0.0,
                radius_m=5.0,
            ),
        ]
        simulation.route_waypoints = []
        simulation.trail_points = [(0.0, 0.0)]

        self.assertTrue(simulation.plan_route())
        self.assertGreater(
            len(simulation.route_waypoints),
            1,
        )

        for _ in range(2500):
            simulation.update(0.05)

            if simulation.arrived or simulation.collided:
                break

        self.assertTrue(simulation.arrived)
        self.assertFalse(simulation.collided)
        self.assertFalse(simulation.navigation_blocked)
        self.assertEqual(simulation.vessel.speed_mps, 0.0)

    def test_navigates_around_staggered_obstacles_and_arrives(
            self,
    ) -> None:
        simulation = Simulation()
        simulation.vessel.x_m = 0.0
        simulation.vessel.y_m = 0.0
        simulation.vessel.heading_deg = 90.0
        simulation.destination_x_m = 180.0
        simulation.destination_y_m = 0.0
        simulation.obstacles = [
            Obstacle(
                x_m=50.0,
                y_m=-5.0,
                radius_m=5.0,
            ),
            Obstacle(
                x_m=120.0,
                y_m=5.0,
                radius_m=5.0,
            ),
        ]
        simulation.route_waypoints = []
        simulation.trail_points = [(0.0, 0.0)]

        self.assertTrue(simulation.plan_route())
        self.assertGreater(
            len(simulation.route_waypoints),
            1,
        )

        for _ in range(4000):
            simulation.update(0.05)

            if simulation.arrived or simulation.collided:
                break

        self.assertTrue(simulation.arrived)
        self.assertFalse(simulation.collided)
        self.assertFalse(simulation.navigation_blocked)
        self.assertEqual(simulation.vessel.speed_mps, 0.0)

    def test_tracks_metrics_during_vessel_movement(self) -> None:
        simulation = Simulation()
        simulation.obstacles = []

        initial_x_m = simulation.vessel.x_m
        initial_y_m = simulation.vessel.y_m
        initial_speed_mps = simulation.vessel.speed_mps

        simulation.update(1.0)

        expected_distance_m = calculate_distance(
            initial_x_m,
            initial_y_m,
            simulation.vessel.x_m,
            simulation.vessel.y_m,
        )
        expected_burn_rate_lph = (
            config.BASE_FUEL_BURN_RATE_LPH
            + config.SPEED_CUBED_COEFFICIENT
            * initial_speed_mps**3
        )

        self.assertAlmostEqual(
            simulation.metrics.elapsed_time_s,
            1.0,
        )
        self.assertAlmostEqual(
            simulation.metrics.distance_traveled_m,
            expected_distance_m,
        )
        self.assertAlmostEqual(
            simulation.metrics.fuel_used_l,
            expected_burn_rate_lph / 3600.0,
        )

    def test_metrics_do_not_change_after_arrival(self) -> None:
        simulation = Simulation()
        simulation.arrived = True
        simulation.vessel.speed_mps = 0.0

        simulation.update(10.0)

        self.assertEqual(
            simulation.metrics.elapsed_time_s,
            0.0,
        )
        self.assertEqual(
            simulation.metrics.distance_traveled_m,
            0.0,
        )
        self.assertEqual(
            simulation.metrics.fuel_used_l,
            0.0,
        )

    def test_metrics_do_not_change_after_collision(self) -> None:
        simulation = Simulation()
        simulation.collided = True
        simulation.vessel.speed_mps = 0.0

        simulation.update(10.0)

        self.assertEqual(
            simulation.metrics.elapsed_time_s,
            0.0,
        )
        self.assertEqual(
            simulation.metrics.distance_traveled_m,
            0.0,
        )
        self.assertEqual(
            simulation.metrics.fuel_used_l,
            0.0,
        )

    def test_completed_mission_metrics_are_consistent(
            self,
    ) -> None:
        simulation = Simulation()
        simulation.vessel.x_m = 0.0
        simulation.vessel.y_m = 0.0
        simulation.vessel.heading_deg = 90.0
        simulation.destination_x_m = 100.0
        simulation.destination_y_m = 0.0
        simulation.obstacles = [
            Obstacle(
                x_m=50.0,
                y_m=0.0,
                radius_m=5.0,
            )
        ]
        simulation.trail_points = [(0.0, 0.0)]

        expected_route_distance_m = 0.0
        expected_fuel_used_l = 0.0

        for _ in range(2000):
            previous_x_m = simulation.vessel.x_m
            previous_y_m = simulation.vessel.y_m
            previous_elapsed_time_s = (
                simulation.metrics.elapsed_time_s
            )

            simulation.update(0.05)

            step_distance_m = calculate_distance(
                previous_x_m,
                previous_y_m,
                simulation.vessel.x_m,
                simulation.vessel.y_m,
            )
            step_elapsed_time_s = (
                    simulation.metrics.elapsed_time_s
                    - previous_elapsed_time_s
            )

            expected_route_distance_m += step_distance_m

            if step_elapsed_time_s > 0.0:
                step_speed_mps = (
                        step_distance_m
                        / step_elapsed_time_s
                )

                if step_speed_mps == 0.0:
                    step_fuel_burn_rate_lph = 0.0
                else:
                    step_fuel_burn_rate_lph = (
                            config.BASE_FUEL_BURN_RATE_LPH
                            + config.SPEED_CUBED_COEFFICIENT
                            * step_speed_mps ** 3
                    )

                expected_fuel_used_l += (
                        step_fuel_burn_rate_lph
                        * step_elapsed_time_s
                        / 3600.0
                )

            if simulation.arrived or simulation.collided:
                break

        self.assertTrue(simulation.arrived)
        self.assertFalse(simulation.collided)
        self.assertGreater(
            simulation.metrics.elapsed_time_s,
            0.0,
        )
        self.assertAlmostEqual(
            simulation.metrics.distance_traveled_m,
            expected_route_distance_m,
        )
        self.assertAlmostEqual(
            simulation.metrics.fuel_used_l,
            expected_fuel_used_l,
        )

    def test_destination_is_active_target_without_avoidance(
            self,
    ) -> None:
        simulation = Simulation()

        self.assertEqual(
            simulation.active_target,
            (
                simulation.destination_x_m,
                simulation.destination_y_m,
            ),
        )

    def test_update_stops_when_global_route_cannot_be_planned(
            self,
    ) -> None:
        simulation = Simulation()
        simulation.vessel.x_m = 0.0
        simulation.vessel.y_m = 0.0
        simulation.destination_x_m = 20.0
        simulation.destination_y_m = 0.0
        simulation.obstacles = [
            Obstacle(
                x_m=20.0,
                y_m=0.0,
                radius_m=5.0,
            )
        ]
        simulation.route_waypoints = []

        simulation.update(0.05)

        self.assertTrue(simulation.navigation_blocked)
        self.assertFalse(simulation.arrived)
        self.assertFalse(simulation.collided)
        self.assertEqual(simulation.route_waypoints, [])
        self.assertEqual(simulation.vessel.speed_mps, 0.0)

    def test_set_destination_resets_mission(self) -> None:
        simulation = Simulation()

        simulation.vessel.x_m = 12.0
        simulation.vessel.y_m = 18.0
        simulation.vessel.heading_deg = 135.0
        simulation.vessel.speed_mps = 0.0

        simulation.arrived = True
        simulation.collided = True
        simulation.navigation_blocked = True
        simulation.trail_points = [
            (0.0, 0.0),
            (12.0, 18.0),
        ]

        simulation.metrics.update(
            distance_traveled_m=20.0,
            speed_mps=4.0,
            dt_s=5.0,
        )

        original_obstacles = simulation.obstacles

        simulation.set_destination(90.0, 65.0)

        self.assertEqual(simulation.destination_x_m, 90.0)
        self.assertEqual(simulation.destination_y_m, 65.0)

        self.assertEqual(simulation.vessel.x_m, 12.0)
        self.assertEqual(simulation.vessel.y_m, 18.0)
        self.assertEqual(simulation.vessel.heading_deg, 135.0)
        self.assertEqual(
            simulation.vessel.speed_mps,
            config.INITIAL_VESSEL_SPEED_MPS,
        )

        self.assertFalse(simulation.arrived)
        self.assertFalse(simulation.collided)
        self.assertFalse(simulation.navigation_blocked)

        self.assertEqual(
            simulation.trail_points,
            [(12.0, 18.0)],
        )
        self.assertEqual(
            simulation.metrics.elapsed_time_s,
            0.0,
        )
        self.assertEqual(
            simulation.metrics.distance_traveled_m,
            0.0,
        )
        self.assertEqual(
            simulation.metrics.fuel_used_l,
            0.0,
        )

        self.assertIs(simulation.obstacles, original_obstacles)

    def test_destination_is_valid_outside_obstacle_clearance(
        self,
    ) -> None:
        simulation = Simulation()
        simulation.obstacles = [
            Obstacle(
                x_m=20.0,
                y_m=30.0,
                radius_m=5.0,
            )
        ]

        is_valid = simulation.is_destination_valid(
            20.0,
            30.0
            + 5.0
            + config.ROUTE_CLEARANCE_M
            + 0.1,
        )

        self.assertTrue(is_valid)

    def test_destination_is_invalid_inside_obstacle_clearance(
        self,
    ) -> None:
        simulation = Simulation()
        simulation.obstacles = [
            Obstacle(
                x_m=20.0,
                y_m=30.0,
                radius_m=5.0,
            )
        ]

        is_valid = simulation.is_destination_valid(
            20.0,
            30.0,
        )

        self.assertFalse(is_valid)

    def test_destination_is_invalid_on_clearance_boundary(
        self,
    ) -> None:
        simulation = Simulation()
        simulation.obstacles = [
            Obstacle(
                x_m=20.0,
                y_m=30.0,
                radius_m=5.0,
            )
        ]

        is_valid = simulation.is_destination_valid(
            20.0,
            30.0
            + 5.0
            + config.ROUTE_CLEARANCE_M,
        )

        self.assertFalse(is_valid)

    def test_invalid_destination_does_not_change_mission(
        self,
    ) -> None:
        simulation = Simulation()
        simulation.obstacles = [
            Obstacle(
                x_m=50.0,
                y_m=25.0,
                radius_m=5.0,
            )
        ]

        simulation.trail_points = [
            (0.0, 0.0),
            (8.0, 4.0),
        ]
        simulation.metrics.update(
            distance_traveled_m=8.0,
            speed_mps=4.0,
            dt_s=2.0,
        )

        original_destination = (
            simulation.destination_x_m,
            simulation.destination_y_m,
        )
        original_speed_mps = simulation.vessel.speed_mps
        original_trail = simulation.trail_points.copy()
        original_metrics = simulation.metrics

        destination_was_set = simulation.set_destination(
            50.0,
            25.0,
        )

        self.assertFalse(destination_was_set)
        self.assertEqual(
            (
                simulation.destination_x_m,
                simulation.destination_y_m,
            ),
            original_destination,
        )
        self.assertEqual(
            simulation.vessel.speed_mps,
            original_speed_mps,
        )
        self.assertEqual(
            simulation.trail_points,
            original_trail,
        )
        self.assertIs(simulation.metrics, original_metrics)

    def test_scenario_reset_records_seed_and_obstacles(
            self,
    ) -> None:
        simulation = Simulation()

        simulation.reset_scenario(12345)

        self.assertEqual(simulation.scenario_seed, 12345)
        self.assertEqual(
            len(simulation.obstacles),
            config.SCENARIO_OBSTACLE_COUNT,
        )

    def test_same_seed_reproduces_same_simulation_layout(
            self,
    ) -> None:
        first_simulation = Simulation()
        second_simulation = Simulation()

        first_simulation.reset_scenario(12345)
        second_simulation.reset_scenario(12345)

        self.assertEqual(
            first_simulation.obstacles,
            second_simulation.obstacles,
        )

    def test_scenario_reset_restores_initial_state(self) -> None:
        simulation = Simulation()

        simulation.vessel.x_m = 30.0
        simulation.vessel.y_m = 20.0
        simulation.vessel.heading_deg = 145.0
        simulation.vessel.speed_mps = 0.0
        simulation.destination_x_m = -40.0
        simulation.destination_y_m = -20.0
        simulation.arrived = True
        simulation.collided = True
        simulation.navigation_blocked = True
        simulation.trail_points = [
            (0.0, 0.0),
            (30.0, 20.0),
        ]

        original_metrics = simulation.metrics

        simulation.reset_scenario(12345)

        self.assertEqual(
            simulation.vessel.x_m,
            config.INITIAL_VESSEL_X_M,
        )
        self.assertEqual(
            simulation.vessel.y_m,
            config.INITIAL_VESSEL_Y_M,
        )
        self.assertEqual(
            simulation.vessel.heading_deg,
            config.INITIAL_VESSEL_HEADING_DEG,
        )
        self.assertEqual(
            simulation.vessel.speed_mps,
            config.INITIAL_VESSEL_SPEED_MPS,
        )
        self.assertEqual(
            simulation.destination_x_m,
            config.DESTINATION_X_M,
        )
        self.assertEqual(
            simulation.destination_y_m,
            config.DESTINATION_Y_M,
        )
        self.assertFalse(simulation.arrived)
        self.assertFalse(simulation.collided)
        self.assertFalse(simulation.navigation_blocked)
        self.assertEqual(
            simulation.trail_points,
            [
                (
                    config.INITIAL_VESSEL_X_M,
                    config.INITIAL_VESSEL_Y_M,
                )
            ],
        )
        self.assertIsNot(simulation.metrics, original_metrics)

    def test_reaches_destination_inside_full_speed_turning_radius(
            self,
    ) -> None:
        simulation = Simulation()
        simulation.vessel.x_m = 0.0
        simulation.vessel.y_m = 0.0
        simulation.vessel.heading_deg = 0.0
        simulation.vessel.speed_mps = (
            config.INITIAL_VESSEL_SPEED_MPS
        )

        simulation.destination_x_m = -10.0
        simulation.destination_y_m = -10.0
        simulation.obstacles = []
        simulation.trail_points = [(0.0, 0.0)]

        for _ in range(1000):
            simulation.update(0.05)

            if simulation.arrived:
                break

        self.assertTrue(simulation.arrived)
        self.assertFalse(simulation.collided)
        self.assertFalse(simulation.navigation_blocked)
        self.assertEqual(simulation.vessel.speed_mps, 0.0)

    def test_global_route_reaches_offset_destination(
            self,
    ) -> None:
        simulation = Simulation()
        simulation.vessel.x_m = 0.0
        simulation.vessel.y_m = 0.0
        simulation.vessel.heading_deg = 90.0
        simulation.destination_x_m = 120.0
        simulation.destination_y_m = 40.0
        simulation.obstacles = [
            Obstacle(
                x_m=40.0,
                y_m=0.0,
                radius_m=5.0,
            )
        ]

        simulation.trail_points = [(0.0, 0.0)]

        self.assertTrue(simulation.plan_route())

        for _ in range(4000):
            simulation.update(0.05)

            if (
                    simulation.arrived
                    or simulation.collided
                    or simulation.navigation_blocked
            ):
                break

        self.assertTrue(simulation.arrived)
        self.assertFalse(simulation.collided)
        self.assertFalse(simulation.navigation_blocked)
        self.assertEqual(simulation.vessel.speed_mps, 0.0)

    def test_destination_at_collision_boundary_is_invalid(
            self,
    ) -> None:
        simulation = Simulation()
        simulation.obstacles = [
            Obstacle(
                x_m=20.0,
                y_m=10.0,
                radius_m=5.0,
            )
        ]

        required_clearance_m = (
            5.0 + config.VESSEL_COLLISION_RADIUS_M
        )

        self.assertFalse(
            simulation.is_destination_valid(
                destination_x_m=(
                    20.0 + required_clearance_m
                ),
                destination_y_m=10.0,
            )
        )

    def test_destination_outside_collision_boundary_is_valid(
            self,
    ) -> None:
        simulation = Simulation()
        simulation.obstacles = [
            Obstacle(
                x_m=20.0,
                y_m=10.0,
                radius_m=5.0,
            )
        ]

        required_clearance_m = (
            5.0 + config.VESSEL_COLLISION_RADIUS_M
        )

        self.assertTrue(
            simulation.is_destination_valid(
                destination_x_m=(
                    20.0
                    + required_clearance_m
                    + 0.000001
                ),
                destination_y_m=10.0,
            )
        )

    def test_plan_route_stores_wide_route_around_cluster(
            self,
    ) -> None:
        simulation = Simulation()
        simulation.vessel.x_m = 0.0
        simulation.vessel.y_m = 0.0
        simulation.destination_x_m = 100.0
        simulation.destination_y_m = 0.0
        simulation.obstacles = [
            Obstacle(
                x_m=50.0,
                y_m=-12.0,
                radius_m=5.0,
            ),
            Obstacle(
                x_m=50.0,
                y_m=0.0,
                radius_m=5.0,
            ),
            Obstacle(
                x_m=50.0,
                y_m=12.0,
                radius_m=5.0,
            ),
        ]

        route_found = simulation.plan_route()

        self.assertTrue(route_found)
        self.assertFalse(simulation.navigation_blocked)
        self.assertGreater(
            len(simulation.route_waypoints),
            1,
        )
        self.assertEqual(
            simulation.route_waypoints[-1],
            (100.0, 0.0),
        )
        self.assertTrue(
            any(
                abs(point_y_m) > 20.0
                for _, point_y_m
                in simulation.route_waypoints
            )
        )
    def test_active_target_is_first_route_waypoint(
            self,
    ) -> None:
        simulation = Simulation()
        simulation.destination_x_m = 100.0
        simulation.destination_y_m = 0.0
        simulation.route_waypoints = [
            (30.0, 20.0),
            (70.0, 20.0),
            (100.0, 0.0),
        ]

        self.assertEqual(
            simulation.active_target,
            (30.0, 20.0),
        )
    def test_reaching_route_waypoint_advances_to_next_target(
            self,
    ) -> None:
        simulation = Simulation()
        simulation.vessel.x_m = 30.0
        simulation.vessel.y_m = 20.0
        simulation.destination_x_m = 100.0
        simulation.destination_y_m = 0.0
        simulation.obstacles = []
        simulation.route_waypoints = [
            (30.0, 20.0),
            (70.0, 20.0),
            (100.0, 0.0),
        ]

        simulation.update(0.05)

        self.assertEqual(
            simulation.route_waypoints,
            [
                (70.0, 20.0),
                (100.0, 0.0),
            ],
        )
        self.assertEqual(
            simulation.active_target,
            (70.0, 20.0),
        )
        self.assertFalse(simulation.arrived)
        self.assertFalse(simulation.navigation_blocked)

    def test_set_destination_automatically_plans_global_route(
            self,
    ) -> None:
        simulation = Simulation()
        simulation.vessel.x_m = 0.0
        simulation.vessel.y_m = 0.0
        simulation.obstacles = [
            Obstacle(
                x_m=50.0,
                y_m=-12.0,
                radius_m=5.0,
            ),
            Obstacle(
                x_m=50.0,
                y_m=0.0,
                radius_m=5.0,
            ),
            Obstacle(
                x_m=50.0,
                y_m=12.0,
                radius_m=5.0,
            ),
        ]

        destination_accepted = simulation.set_destination(
            destination_x_m=100.0,
            destination_y_m=0.0,
        )

        self.assertTrue(destination_accepted)
        self.assertFalse(simulation.navigation_blocked)
        self.assertGreater(
            len(simulation.route_waypoints),
            1,
        )
        self.assertEqual(
            simulation.route_waypoints[-1],
            (100.0, 0.0),
        )
        self.assertTrue(
            any(
                abs(waypoint_y_m) > 20.0
                for _, waypoint_y_m
                in simulation.route_waypoints
            )
        )

    def test_reset_scenario_clears_global_route(
            self,
    ) -> None:
        simulation = Simulation()
        simulation.route_waypoints = [
            (30.0, 20.0),
            (100.0, 0.0),
        ]

        simulation.reset_scenario(seed=0)

        self.assertEqual(
            simulation.route_waypoints,
            [],
        )

if __name__ == "__main__":
    unittest.main()