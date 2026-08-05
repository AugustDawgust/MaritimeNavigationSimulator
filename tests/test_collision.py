import unittest

from collision import (
    vessel_collides_with_any_obstacle,
    vessel_collides_with_obstacle,
)
from obstacle import Obstacle
from vessel import Vessel
import config

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
            x_m=0.0,
            y_m=8.0,
            radius_m=3.0,
        )

        self.assertTrue(
            vessel_collides_with_obstacle(
                self.vessel,
                vessel_bounding_radius_m=(
                    config.VESSEL_BOUNDING_RADIUS_M
                ),
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
                vessel_bounding_radius_m=(
                    config.VESSEL_BOUNDING_RADIUS_M
                ),
                obstacle=obstacle,
            )
        )

    def test_touching_boundaries_count_as_collision(self) -> None:
        obstacle = Obstacle(
            x_m=0.0,
            y_m=9.0,
            radius_m=3.0,
        )

        self.assertTrue(
            vessel_collides_with_obstacle(
                self.vessel,
                vessel_bounding_radius_m=(
                    config.VESSEL_BOUNDING_RADIUS_M
                ),
                obstacle=obstacle,
            )
        )

    def test_detects_collision_in_obstacle_list(self) -> None:
        obstacles = [
            Obstacle(x_m=20.0, y_m=0.0, radius_m=3.0),
            Obstacle(x_m=0.0, y_m=8.0, radius_m=3.0),
            Obstacle(x_m=-20.0, y_m=0.0, radius_m=3.0),
        ]

        self.assertTrue(
            vessel_collides_with_any_obstacle(
                self.vessel,
                vessel_bounding_radius_m=(
                    config.VESSEL_BOUNDING_RADIUS_M
                ),
                obstacles=obstacles,
            )
        )

    def test_empty_obstacle_list_has_no_collision(self) -> None:
        self.assertFalse(
            vessel_collides_with_any_obstacle(
                self.vessel,
                vessel_bounding_radius_m=(
                    config.VESSEL_BOUNDING_RADIUS_M
                ),
                obstacles=[],
            )
        )

    def test_collision_occurs_exactly_at_bow_contact(
            self,
    ) -> None:
        vessel = Vessel(
            x_m=0.0,
            y_m=0.0,
            heading_deg=0.0,
            speed_mps=0.0,
        )
        obstacle = Obstacle(
            x_m=0.0,
            y_m=11.0,
            radius_m=5.0,
        )

        self.assertTrue(
            vessel_collides_with_obstacle(
                vessel,
                vessel_bounding_radius_m=(
                    config.VESSEL_BOUNDING_RADIUS_M
                ),
                obstacle=obstacle,
            )
        )

    def test_no_collision_immediately_beyond_bow_contact(
            self,
    ) -> None:
        vessel = Vessel(
            x_m=0.0,
            y_m=0.0,
            heading_deg=0.0,
            speed_mps=0.0,
        )
        obstacle = Obstacle(
            x_m=0.0,
            y_m=11.000001,
            radius_m=5.0,
        )

        self.assertFalse(
            vessel_collides_with_obstacle(
                vessel,
                vessel_bounding_radius_m=(
                    config.VESSEL_BOUNDING_RADIUS_M
                ),
                obstacle=obstacle,
            )
        )


if __name__ == "__main__":
    unittest.main()