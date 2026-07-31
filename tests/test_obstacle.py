import unittest

from obstacle import Obstacle


class TestObstacle(unittest.TestCase):
    def test_obstacle_stores_position_and_radius(self) -> None:
        obstacle = Obstacle(
            x_m=40.0,
            y_m=25.0,
            radius_m=6.0,
        )

        self.assertEqual(obstacle.x_m, 40.0)
        self.assertEqual(obstacle.y_m, 25.0)
        self.assertEqual(obstacle.radius_m, 6.0)

    def test_rejects_zero_radius(self) -> None:
        with self.assertRaises(ValueError):
            Obstacle(
                x_m=40.0,
                y_m=25.0,
                radius_m=0.0,
            )

    def test_rejects_negative_radius(self) -> None:
        with self.assertRaises(ValueError):
            Obstacle(
                x_m=40.0,
                y_m=25.0,
                radius_m=-2.0,
            )

if __name__ == "__main__":
    unittest.main()