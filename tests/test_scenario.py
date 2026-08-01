import math
import unittest

from scenario import generate_obstacles


class TestScenarioGeneration(unittest.TestCase):
    def generate_standard_layout(
            self,
            seed: int,
    ):
        return generate_obstacles(
            seed=seed,
            obstacle_count=5,
            world_width_m=200.0,
            world_height_m=120.0,
            min_radius_m=3.0,
            max_radius_m=7.0,
            edge_clearance_m=2.0,
            obstacle_spacing_m=3.0,
            protected_points=[
                (-50.0, 0.0, 10.0),
                (50.0, 0.0, 10.0),
            ],
            max_attempts=10_000,
        )

    def test_same_seed_reproduces_same_layout(self) -> None:
        first_layout = self.generate_standard_layout(12345)
        second_layout = self.generate_standard_layout(12345)

        self.assertEqual(first_layout, second_layout)

    def test_different_seeds_produce_different_layouts(
            self,
    ) -> None:
        first_layout = self.generate_standard_layout(12345)
        second_layout = self.generate_standard_layout(54321)

        self.assertNotEqual(first_layout, second_layout)

    def test_obstacles_remain_inside_world_boundaries(
            self,
    ) -> None:
        obstacles = self.generate_standard_layout(12345)

        half_width_m = 100.0
        half_height_m = 60.0
        edge_clearance_m = 2.0

        for obstacle in obstacles:
            self.assertLessEqual(
                abs(obstacle.x_m)
                + obstacle.radius_m
                + edge_clearance_m,
                half_width_m,
            )
            self.assertLessEqual(
                abs(obstacle.y_m)
                + obstacle.radius_m
                + edge_clearance_m,
                half_height_m,
            )

    def test_obstacles_do_not_overlap(self) -> None:
        obstacles = self.generate_standard_layout(12345)
        required_spacing_m = 3.0

        for index, first_obstacle in enumerate(obstacles):
            for second_obstacle in obstacles[index + 1:]:
                center_distance_m = math.hypot(
                    first_obstacle.x_m - second_obstacle.x_m,
                    first_obstacle.y_m - second_obstacle.y_m,
                )

                required_distance_m = (
                    first_obstacle.radius_m
                    + second_obstacle.radius_m
                    + required_spacing_m
                )

                self.assertGreater(
                    center_distance_m,
                    required_distance_m,
                )

    def test_obstacles_avoid_protected_points(self) -> None:
        obstacles = self.generate_standard_layout(12345)

        protected_points = [
            (-50.0, 0.0, 10.0),
            (50.0, 0.0, 10.0),
        ]

        for obstacle in obstacles:
            for point_x_m, point_y_m, clearance_m in (
                    protected_points
            ):
                distance_m = math.hypot(
                    obstacle.x_m - point_x_m,
                    obstacle.y_m - point_y_m,
                )

                self.assertGreater(
                    distance_m,
                    obstacle.radius_m + clearance_m,
                )

    def test_impossible_layout_fails_safely(self) -> None:
        with self.assertRaises(RuntimeError):
            generate_obstacles(
                seed=12345,
                obstacle_count=1,
                world_width_m=20.0,
                world_height_m=20.0,
                min_radius_m=2.0,
                max_radius_m=2.0,
                edge_clearance_m=0.0,
                obstacle_spacing_m=0.0,
                protected_points=[
                    (0.0, 0.0, 100.0),
                ],
                max_attempts=10,
            )


if __name__ == "__main__":
    unittest.main()