import unittest

from obstacle import Obstacle
from route_planner import find_route, is_segment_clear


class TestRoutePlannerGeometry(unittest.TestCase):

    def setUp(self) -> None:
        self.obstacle = Obstacle(
            x_m=10.0,
            y_m=0.0,
            radius_m=2.0,
        )

    def test_segment_is_clear_without_obstacles(self) -> None:
        self.assertTrue(
            is_segment_clear(
                start=(0.0, 0.0),
                end=(20.0, 0.0),
                obstacles=[],
                clearance_m=3.0,
            )
        )

    def test_segment_through_inflated_obstacle_is_blocked(
            self,
    ) -> None:
        self.assertFalse(
            is_segment_clear(
                start=(0.0, 0.0),
                end=(20.0, 0.0),
                obstacles=[self.obstacle],
                clearance_m=3.0,
            )
        )

    def test_segment_tangent_to_inflated_obstacle_is_blocked(
            self,
    ) -> None:
        self.assertFalse(
            is_segment_clear(
                start=(0.0, 5.0),
                end=(20.0, 5.0),
                obstacles=[self.obstacle],
                clearance_m=3.0,
            )
        )

    def test_segment_immediately_outside_inflated_obstacle_is_clear(
            self,
    ) -> None:
        self.assertTrue(
            is_segment_clear(
                start=(0.0, 5.000001),
                end=(20.0, 5.000001),
                obstacles=[self.obstacle],
                clearance_m=3.0,
            )
        )

    def test_zero_length_segment_inside_obstacle_is_blocked(
            self,
    ) -> None:
        self.assertFalse(
            is_segment_clear(
                start=(10.0, 0.0),
                end=(10.0, 0.0),
                obstacles=[self.obstacle],
                clearance_m=3.0,
            )
        )

    def test_zero_length_segment_outside_obstacle_is_clear(
            self,
    ) -> None:
        self.assertTrue(
            is_segment_clear(
                start=(20.0, 0.0),
                end=(20.0, 0.0),
                obstacles=[self.obstacle],
                clearance_m=3.0,
            )
        )


if __name__ == "__main__":
    unittest.main()

class TestGlobalRoutePlanner(unittest.TestCase):

    def assert_route_is_clear(
            self,
            start: tuple[float, float],
            route: list[tuple[float, float]],
            obstacles: list[Obstacle],
            clearance_m: float,
    ) -> None:
        previous_point = start

        for route_point in route:
            self.assertTrue(
                is_segment_clear(
                    start=previous_point,
                    end=route_point,
                    obstacles=obstacles,
                    clearance_m=clearance_m,
                )
            )
            previous_point = route_point

    def test_direct_route_contains_only_destination(
            self,
    ) -> None:
        route = find_route(
            start=(0.0, 0.0),
            destination=(20.0, 0.0),
            obstacles=[],
            clearance_m=3.0,
        )

        self.assertEqual(route, [(20.0, 0.0)])

    def test_route_is_found_around_single_obstacle(
            self,
    ) -> None:
        obstacles = [
            Obstacle(
                x_m=10.0,
                y_m=0.0,
                radius_m=2.0,
            )
        ]

        route = find_route(
            start=(0.0, 0.0),
            destination=(20.0, 0.0),
            obstacles=obstacles,
            clearance_m=3.0,
        )

        self.assertIsNotNone(route)
        assert route is not None

        self.assertGreater(len(route), 1)
        self.assertEqual(route[-1], (20.0, 0.0))

        self.assert_route_is_clear(
            start=(0.0, 0.0),
            route=route,
            obstacles=obstacles,
            clearance_m=3.0,
        )

    def test_wide_route_is_found_around_obstacle_cluster(
            self,
    ) -> None:
        obstacles = [
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

        route = find_route(
            start=(0.0, 0.0),
            destination=(100.0, 0.0),
            obstacles=obstacles,
            clearance_m=3.0,
        )

        self.assertIsNotNone(route)
        assert route is not None

        self.assertEqual(route[-1], (100.0, 0.0))
        self.assertTrue(
            any(
                abs(point_y_m) > 20.0
                for _, point_y_m in route
            )
        )

        self.assert_route_is_clear(
            start=(0.0, 0.0),
            route=route,
            obstacles=obstacles,
            clearance_m=3.0,
        )

    def test_no_route_when_destination_is_inside_obstacle(
            self,
    ) -> None:
        obstacles = [
            Obstacle(
                x_m=20.0,
                y_m=0.0,
                radius_m=5.0,
            )
        ]

        route = find_route(
            start=(0.0, 0.0),
            destination=(20.0, 0.0),
            obstacles=obstacles,
            clearance_m=3.0,
        )

        self.assertIsNone(route)