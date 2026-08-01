import unittest

from avoidance import (
    calculate_avoidance_waypoint,
    find_nearest_blocking_obstacle,
    obstacle_blocks_route,
)
from obstacle import Obstacle


class TestAvoidance(unittest.TestCase):
    def test_detects_obstacle_directly_on_route(self) -> None:
        obstacle = Obstacle(
            x_m=50.0,
            y_m=0.0,
            radius_m=5.0,
        )

        self.assertTrue(
            obstacle_blocks_route(
                0.0,
                0.0,
                100.0,
                0.0,
                obstacle,
                clearance_m=6.0,
            )
        )

    def test_ignores_obstacle_far_from_route(self) -> None:
        obstacle = Obstacle(
            x_m=50.0,
            y_m=20.0,
            radius_m=5.0,
        )

        self.assertFalse(
            obstacle_blocks_route(
                0.0,
                0.0,
                100.0,
                0.0,
                obstacle,
                clearance_m=6.0,
            )
        )

    def test_touching_clearance_boundary_blocks_route(self) -> None:
        obstacle = Obstacle(
            x_m=50.0,
            y_m=11.0,
            radius_m=5.0,
        )

        self.assertTrue(
            obstacle_blocks_route(
                0.0,
                0.0,
                100.0,
                0.0,
                obstacle,
                clearance_m=6.0,
            )
        )

    def test_ignores_obstacle_behind_start(self) -> None:
        obstacle = Obstacle(
            x_m=-20.0,
            y_m=0.0,
            radius_m=5.0,
        )

        self.assertFalse(
            obstacle_blocks_route(
                0.0,
                0.0,
                100.0,
                0.0,
                obstacle,
                clearance_m=6.0,
            )
        )

    def test_ignores_obstacle_beyond_destination(self) -> None:
        obstacle = Obstacle(
            x_m=120.0,
            y_m=0.0,
            radius_m=5.0,
        )

        self.assertFalse(
            obstacle_blocks_route(
                0.0,
                0.0,
                100.0,
                0.0,
                obstacle,
                clearance_m=6.0,
            )
        )

    def test_finds_blocking_obstacle_in_list(self) -> None:
        blocking_obstacle = Obstacle(
            x_m=50.0,
            y_m=0.0,
            radius_m=5.0,
        )
        obstacles = [
            Obstacle(x_m=30.0, y_m=30.0, radius_m=5.0),
            blocking_obstacle,
            Obstacle(x_m=70.0, y_m=-30.0, radius_m=5.0),
        ]

        result = find_nearest_blocking_obstacle(
            0.0,
            0.0,
            100.0,
            0.0,
            obstacles,
            clearance_m=6.0,
        )

        self.assertEqual(result, blocking_obstacle)

    def test_returns_none_when_route_is_clear(self) -> None:
        obstacles = [
            Obstacle(x_m=30.0, y_m=30.0, radius_m=5.0),
            Obstacle(x_m=70.0, y_m=-30.0, radius_m=5.0),
        ]

        result = find_nearest_blocking_obstacle(
            0.0,
            0.0,
            100.0,
            0.0,
            obstacles,
            clearance_m=6.0,
        )

        self.assertIsNone(result)

    def test_selects_nearest_blocking_obstacle(self) -> None:
        nearest_obstacle = Obstacle(
            x_m=30.0,
            y_m=0.0,
            radius_m=5.0,
        )
        obstacles = [
            Obstacle(x_m=70.0, y_m=0.0, radius_m=5.0),
            nearest_obstacle,
        ]

        result = find_nearest_blocking_obstacle(
            0.0,
            0.0,
            100.0,
            0.0,
            obstacles,
            clearance_m=6.0,
        )

        self.assertEqual(result, nearest_obstacle)

    def test_calculates_waypoint_beside_horizontal_route(
        self,
    ) -> None:
        obstacle = Obstacle(
            x_m=50.0,
            y_m=0.0,
            radius_m=5.0,
        )

        waypoint = calculate_avoidance_waypoint(
            0.0,
            0.0,
            100.0,
            0.0,
            obstacle,
            clearance_m=6.0,
            extra_offset_m=4.0,
        )

        self.assertAlmostEqual(waypoint[0], 50.0)
        self.assertAlmostEqual(waypoint[1], 15.0)

    def test_calculates_waypoint_beside_vertical_route(
        self,
    ) -> None:
        obstacle = Obstacle(
            x_m=0.0,
            y_m=50.0,
            radius_m=5.0,
        )

        waypoint = calculate_avoidance_waypoint(
            0.0,
            0.0,
            0.0,
            100.0,
            obstacle,
            clearance_m=6.0,
            extra_offset_m=4.0,
        )

        self.assertAlmostEqual(waypoint[0], -15.0)
        self.assertAlmostEqual(waypoint[1], 50.0)

    def test_rejects_zero_length_route(self) -> None:
        obstacle = Obstacle(
            x_m=50.0,
            y_m=0.0,
            radius_m=5.0,
        )

        with self.assertRaises(ValueError):
            calculate_avoidance_waypoint(
                0.0,
                0.0,
                0.0,
                0.0,
                obstacle,
                clearance_m=6.0,
                extra_offset_m=4.0,
            )
    def test_generated_route_clears_obstacle(self) -> None:
        obstacle = Obstacle(
            x_m=50.0,
            y_m=0.0,
            radius_m=5.0,
        )

        waypoint_x_m, waypoint_y_m = (
            calculate_avoidance_waypoint(
                0.0,
                0.0,
                100.0,
                0.0,
                obstacle,
                clearance_m=6.0,
                extra_offset_m=4.0,
            )
        )

        first_leg_blocked = obstacle_blocks_route(
            0.0,
            0.0,
            waypoint_x_m,
            waypoint_y_m,
            obstacle,
            clearance_m=6.0,
        )
        second_leg_blocked = obstacle_blocks_route(
            waypoint_x_m,
            waypoint_y_m,
            100.0,
            0.0,
            obstacle,
            clearance_m=6.0,
        )

        self.assertFalse(first_leg_blocked)
        self.assertFalse(second_leg_blocked)

if __name__ == "__main__":
    unittest.main()