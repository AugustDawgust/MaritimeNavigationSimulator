import math

import pygame

import config
from vessel import Vessel


class Renderer:
    def __init__(self, screen: pygame.Surface) -> None:
        self.screen = screen

    def render(
            self,
            vessel: Vessel,
            destination_x_m: float,
            destination_y_m: float,
            arrived: bool,
    ) -> None:
        self.screen.fill(config.BACKGROUND_COLOR)

        self._draw_destination(
            destination_x_m,
            destination_y_m,
            arrived,
        )
        self._draw_vessel(vessel)

        pygame.display.flip()

    def _draw_vessel(self, vessel: Vessel) -> None:
        heading_rad = math.radians(vessel.heading_deg)

        forward_x = math.sin(heading_rad)
        forward_y = math.cos(heading_rad)

        right_x = math.cos(heading_rad)
        right_y = -math.sin(heading_rad)

        half_length = config.VESSEL_LENGTH_M / 2.0
        half_beam = config.VESSEL_BEAM_M / 2.0

        bow = (
            vessel.x_m + forward_x * half_length,
            vessel.y_m + forward_y * half_length,
        )
        stern_center = (
            vessel.x_m - forward_x * half_length,
            vessel.y_m - forward_y * half_length,
        )
        port_stern = (
            stern_center[0] - right_x * half_beam,
            stern_center[1] - right_y * half_beam,
        )
        starboard_stern = (
            stern_center[0] + right_x * half_beam,
            stern_center[1] + right_y * half_beam,
        )

        screen_points = [
            self._world_to_screen(*bow),
            self._world_to_screen(*port_stern),
            self._world_to_screen(*starboard_stern),
        ]

        pygame.draw.polygon(
            self.screen,
            config.VESSEL_COLOR,
            screen_points,
        )
    def _draw_destination(
        self,
        destination_x_m: float,
        destination_y_m: float,
        arrived: bool,
    ) -> None:
        screen_position = self._world_to_screen(
            destination_x_m,
            destination_y_m,
        )

        color = (
            config.DESTINATION_REACHED_COLOR
            if arrived
            else config.DESTINATION_COLOR
        )

        pygame.draw.circle(
            self.screen,
            color,
            screen_position,
            config.DESTINATION_RADIUS_PX,
            config.DESTINATION_LINE_WIDTH_PX,
        )


    @staticmethod
    def _world_to_screen(x_m: float, y_m: float) -> tuple[float, float]:
        screen_x = config.WINDOW_WIDTH / 2 + x_m * config.PIXELS_PER_METER
        screen_y = config.WINDOW_HEIGHT / 2 - y_m * config.PIXELS_PER_METER
        return screen_x, screen_y