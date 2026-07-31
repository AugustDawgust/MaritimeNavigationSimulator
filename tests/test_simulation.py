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


if __name__ == "__main__":
    unittest.main()