import unittest

from collision import (
    vessel_collides_with_any_obstacle,
    vessel_collides_with_obstacle,
)
from obstacle import Obstacle
from vessel import Vessel


class TestCollision(unittest.TestCase):
    def setUp(self) -> None:
        self.vessel = Vessel(
            x_m=0.0,
            y_m=0.0,
            heading_deg=0.0,
            speed_mps=8.0,
        )

    def test_detects_overlapping_boundaries(self) -> None:
        obstacle = Obstacle(
            x_m=8.0,
            y_m=0.0,
            radius_m=3.0,
        )

        self.assertTrue(
            vessel_collides_with_obstacle(
                self.vessel,
                vessel_collision_radius_m=6.0,
                obstacle=obstacle,
            )
        )

    def test_detects_separate_boundaries(self) -> None:
        obstacle = Obstacle(
            x_m=12.0,
            y_m=0.0,
            radius_m=3.0,
        )

        self.assertFalse(
            vessel_collides_with_obstacle(
                self.vessel,
                vessel_collision_radius_m=6.0,
                obstacle=obstacle,
            )
        )

    def test_touching_boundaries_count_as_collision(self) -> None:
        obstacle = Obstacle(
            x_m=9.0,
            y_m=0.0,
            radius_m=3.0,
        )

        self.assertTrue(
            vessel_collides_with_obstacle(
                self.vessel,
                vessel_collision_radius_m=6.0,
                obstacle=obstacle,
            )
        )

    def test_detects_collision_in_obstacle_list(self) -> None:
        obstacles = [
            Obstacle(x_m=20.0, y_m=0.0, radius_m=3.0),
            Obstacle(x_m=8.0, y_m=0.0, radius_m=3.0),
            Obstacle(x_m=-20.0, y_m=0.0, radius_m=3.0),
        ]

        self.assertTrue(
            vessel_collides_with_any_obstacle(
                self.vessel,
                vessel_collision_radius_m=6.0,
                obstacles=obstacles,
            )
        )

    def test_empty_obstacle_list_has_no_collision(self) -> None:
        self.assertFalse(
            vessel_collides_with_any_obstacle(
                self.vessel,
                vessel_collision_radius_m=6.0,
                obstacles=[],
            )
        )


if __name__ == "__main__":
    unittest.main()