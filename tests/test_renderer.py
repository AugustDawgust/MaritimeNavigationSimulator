import unittest
from unittest.mock import Mock, patch

import config
from obstacle import Obstacle
from renderer import Renderer


class TestRenderer(unittest.TestCase):

    def test_screen_to_world_converts_coordinates(self) -> None:
        screen_x_px = (
            config.WINDOW_WIDTH / 2
            + 50.0 * config.PIXELS_PER_METER
        )
        screen_y_px = (
            config.WINDOW_HEIGHT / 2
            - 25.0 * config.PIXELS_PER_METER
        )

        world_x_m, world_y_m = Renderer.screen_to_world(
            screen_x_px,
            screen_y_px,
        )

        self.assertAlmostEqual(world_x_m, 50.0)
        self.assertAlmostEqual(world_y_m, 25.0)

    @patch("renderer.pygame.draw.circle")
    def test_obstacle_radius_is_converted_from_meters_to_pixels(
            self,
            mock_draw_circle: Mock,
    ) -> None:
        renderer = Renderer.__new__(Renderer)
        renderer.screen = Mock()

        obstacle = Obstacle(
            x_m=10.0,
            y_m=5.0,
            radius_m=7.0,
        )

        renderer._draw_obstacles([obstacle])

        expected_radius_px = round(
            obstacle.radius_m * config.PIXELS_PER_METER
        )

        self.assertEqual(mock_draw_circle.call_count, 2)
        self.assertEqual(
            mock_draw_circle.call_args_list[0].args[3],
            expected_radius_px,
        )
        self.assertEqual(
            mock_draw_circle.call_args_list[1].args[3],
            expected_radius_px,
        )


if __name__ == "__main__":
    unittest.main()