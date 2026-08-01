import unittest

from obstacle import Obstacle
from simulation import Simulation
from navigation import calculate_distance
from mission_route import MissionRoute
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
    def test_creates_waypoint_for_blocking_obstacle(self) -> None:
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

        simulation.update(0.0)

        self.assertIsNotNone(simulation.avoidance_waypoint)
        self.assertFalse(simulation.collided)

    def test_does_not_create_waypoint_for_clear_route(
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
                y_m=30.0,
                radius_m=5.0,
            )
        ]

        simulation.update(0.0)

        self.assertIsNone(simulation.avoidance_waypoint)

    def test_creates_waypoint_for_curved_trajectory_threat(
        self,
    ) -> None:
        simulation = Simulation()
        simulation.vessel.x_m = 0.0
        simulation.vessel.y_m = 0.0
        simulation.vessel.heading_deg = 0.0
        simulation.destination_x_m = 100.0
        simulation.destination_y_m = 0.0
        simulation.obstacles = [
            Obstacle(
                x_m=15.0,
                y_m=22.0,
                radius_m=2.0,
            )
        ]

        simulation.update(0.0)

        self.assertIsNotNone(
            simulation.avoidance_waypoint
        )

        waypoint_x_m, waypoint_y_m = (
            simulation.avoidance_waypoint
        )

        self.assertAlmostEqual(waypoint_x_m, 15.0)
        self.assertLess(waypoint_y_m, 22.0)

    def test_selects_safe_side_of_blocking_obstacle(
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
            ),
            Obstacle(
                x_m=50.0,
                y_m=15.0,
                radius_m=2.0,
            ),
        ]

        simulation.update(0.0)

        self.assertIsNotNone(
            simulation.avoidance_waypoint
        )

        waypoint_x_m, waypoint_y_m = (
            simulation.avoidance_waypoint
        )

        self.assertAlmostEqual(waypoint_x_m, 50.0)
        self.assertLess(waypoint_y_m, 0.0)

    def test_clears_reached_avoidance_waypoint(self) -> None:
        simulation = Simulation()
        simulation.obstacles = []
        simulation.avoidance_waypoint = (
            simulation.vessel.x_m,
            simulation.vessel.y_m,
        )

        simulation.update(0.0)

        self.assertIsNone(simulation.avoidance_waypoint)
    def test_navigates_around_obstacle_and_arrives(self) -> None:
        simulation = Simulation()
        simulation.route = MissionRoute(
            waypoints=[(100.0, 0.0)]
        )
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
        simulation.avoidance_waypoint = None
        simulation.trail_points = [(0.0, 0.0)]

        for _ in range(2000):
            simulation.update(0.05)

            if simulation.arrived or simulation.collided:
                break

        for _ in range(2000):
            simulation.update(0.05)

            if simulation.arrived or simulation.collided:
                break

        self.assertTrue(simulation.arrived)
        self.assertTrue(simulation.arrived)
        self.assertFalse(simulation.collided)
        self.assertEqual(simulation.vessel.speed_mps, 0.0)

    def test_resumes_destination_navigation_after_waypoint(
        self,
    ) -> None:
        simulation = Simulation()
        simulation.vessel.x_m = 0.0
        simulation.vessel.y_m = 0.0
        simulation.vessel.heading_deg = 90.0
        simulation.destination_x_m = 100.0
        simulation.destination_y_m = 0.0
        simulation.obstacles = []
        simulation.avoidance_waypoint = (0.0, 0.0)
        simulation.trail_points = [(0.0, 0.0)]

        simulation.update(0.1)

        self.assertIsNone(simulation.avoidance_waypoint)
        self.assertGreater(simulation.vessel.x_m, 0.0)
        self.assertAlmostEqual(simulation.vessel.y_m, 0.0)

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
        simulation.route = MissionRoute(
            waypoints=[(100.0, 0.0)]
        )
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
        simulation.avoidance_waypoint = None
        simulation.trail_points = [(0.0, 0.0)]

        movement_speed_mps = simulation.vessel.speed_mps

        for _ in range(2000):
            simulation.update(0.05)

            if simulation.arrived or simulation.collided:
                break

        expected_fuel_burn_rate_lph = (
            config.BASE_FUEL_BURN_RATE_LPH
            + config.SPEED_CUBED_COEFFICIENT
            * movement_speed_mps**3
        )
        expected_fuel_used_l = (
            expected_fuel_burn_rate_lph
            * simulation.metrics.elapsed_time_s
            / 3600.0
        )
        expected_route_distance_m = (
            movement_speed_mps
            * simulation.metrics.elapsed_time_s
        )

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

    def test_initializes_route_from_config(self) -> None:
        simulation = Simulation()

        self.assertEqual(
            simulation.route.waypoints,
            config.MISSION_WAYPOINTS,
        )
        self.assertEqual(
            simulation.route.current_waypoint,
            config.MISSION_WAYPOINTS[0],
        )
        self.assertEqual(
            (
                simulation.destination_x_m,
                simulation.destination_y_m,
            ),
            config.MISSION_WAYPOINTS[0],
        )
        self.assertEqual(
            simulation.route.total_waypoint_count,
            len(config.MISSION_WAYPOINTS),
        )
        self.assertFalse(simulation.route.is_complete)

    def test_advances_through_configured_mission_waypoints(
            self,
    ) -> None:
        simulation = Simulation()

        for waypoint_index, waypoint in enumerate(
                config.MISSION_WAYPOINTS
        ):
            simulation.vessel.x_m = waypoint[0]
            simulation.vessel.y_m = waypoint[1]

            simulation.update(0.0)

            is_final_waypoint = (
                    waypoint_index
                    == len(config.MISSION_WAYPOINTS) - 1
            )

            if is_final_waypoint:
                self.assertTrue(simulation.arrived)
                self.assertTrue(simulation.route.is_complete)
                self.assertIsNone(
                    simulation.route.current_waypoint
                )
                self.assertEqual(
                    simulation.vessel.speed_mps,
                    0.0,
                )
            else:
                next_waypoint = config.MISSION_WAYPOINTS[
                    waypoint_index + 1
                    ]

                self.assertFalse(simulation.arrived)
                self.assertEqual(
                    simulation.route.current_waypoint,
                    next_waypoint,
                )
                self.assertEqual(
                    (
                        simulation.destination_x_m,
                        simulation.destination_y_m,
                    ),
                    next_waypoint,
                )

    def test_stops_when_threat_has_no_safe_avoidance_side(
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
            ),
            Obstacle(
                x_m=50.0,
                y_m=15.0,
                radius_m=2.0,
            ),
            Obstacle(
                x_m=50.0,
                y_m=-15.0,
                radius_m=2.0,
            ),
        ]

        initial_position = (
            simulation.vessel.x_m,
            simulation.vessel.y_m,
        )

        simulation.update(0.1)

        self.assertTrue(simulation.navigation_blocked)
        self.assertEqual(simulation.vessel.speed_mps, 0.0)
        self.assertEqual(
            (
                simulation.vessel.x_m,
                simulation.vessel.y_m,
            ),
            initial_position,
        )
        self.assertIsNone(simulation.avoidance_waypoint)
        self.assertFalse(simulation.collided)

    def test_anticipates_intermediate_waypoint_turn(
            self,
    ) -> None:
        simulation = Simulation()
        simulation.obstacles = []

        simulation.vessel.x_m = 6.0
        simulation.vessel.y_m = 15.0

        simulation.update(0.0)

        self.assertEqual(
            simulation.route.current_waypoint_index,
            1,
        )
        self.assertEqual(
            (
                simulation.destination_x_m,
                simulation.destination_y_m,
            ),
            (55.0, 60.0),
        )
        self.assertFalse(simulation.arrived)

if __name__ == "__main__":
    unittest.main()