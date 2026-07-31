import unittest

from obstacle import Obstacle
from simulation import Simulation
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

    def test_does_not_create_waypoint_for_clear_route(self) -> None:
        simulation = Simulation()
        simulation.vessel.x_m = 0.0
        simulation.vessel.y_m = 0.0
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

if __name__ == "__main__":
    unittest.main()