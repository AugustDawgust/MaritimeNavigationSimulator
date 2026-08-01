import unittest

import config
from renderer import Renderer


class TestRendererCoordinates(unittest.TestCase):

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


if __name__ == "__main__":
    unittest.main()