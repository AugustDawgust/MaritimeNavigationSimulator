import pygame

import config
from vessel import Vessel

from obstacle import Obstacle
from mission_metrics import (
    MissionMetrics,
    calculate_fuel_burn_rate_lph,
)

class Renderer:
    def __init__(self, screen: pygame.Surface) -> None:
        self.screen = screen
        self.status_font = pygame.font.Font(
            None,
            config.STATUS_FONT_SIZE,
        )

    def render(
            self,
            vessel: Vessel,
            destination_x_m: float,
            destination_y_m: float,
            arrived: bool,
            collided: bool,
            navigation_blocked: bool,
            invalid_destination: bool,
            trail_points: list[tuple[float, float]],
            distance_to_destination_m: float,
            obstacles: list[Obstacle],
            metrics: MissionMetrics,
    ) -> None:
        self.screen.fill(config.BACKGROUND_COLOR)

        self._draw_trail(trail_points)
        self._draw_obstacles(obstacles)

        self._draw_destination(
            destination_x_m,
            destination_y_m,
            arrived,
        )
        self._draw_vessel(vessel)

        self._draw_status_panel(
            vessel,
            distance_to_destination_m,
            arrived,
            collided,
            navigation_blocked,
            invalid_destination,
            metrics,
        )

        pygame.display.flip()

    def _draw_status_panel(
            self,
            vessel: Vessel,
            distance_to_destination_m: float,
            arrived: bool,
            collided: bool,
            navigation_blocked: bool,
            invalid_destination: bool,
            metrics: MissionMetrics,
    ) -> None:
        panel_rect = pygame.Rect(
            config.STATUS_PANEL_X_PX,
            config.STATUS_PANEL_Y_PX,
            config.STATUS_PANEL_WIDTH_PX,
            config.STATUS_PANEL_HEIGHT_PX,
        )

        pygame.draw.rect(
            self.screen,
            config.STATUS_PANEL_COLOR,
            panel_rect,
            border_radius=8,
        )

        if invalid_destination:
            status_text = "INVALID DESTINATION"
            status_color = (
                config.STATUS_INVALID_DESTINATION_COLOR
            )
        elif collided:
            status_text = "COLLISION"
            status_color = config.STATUS_COLLISION_COLOR
        elif arrived:
            status_text = "ARRIVED"
            status_color = config.STATUS_ARRIVED_COLOR
        elif navigation_blocked:
            status_text = "NAVIGATION BLOCKED"
            status_color = config.STATUS_BLOCKED_COLOR
        else:
            status_text = "NAVIGATING"
            status_color = config.STATUS_TEXT_COLOR

        current_fuel_burn_rate_lph = (
            calculate_fuel_burn_rate_lph(
                vessel.speed_mps,
                metrics.base_fuel_burn_rate_lph,
                metrics.speed_cubed_coefficient,
            )
        )

        lines = [
            (f"Status: {status_text}", status_color),
            (
                f"Heading: {vessel.heading_deg:.1f} deg",
                config.STATUS_TEXT_COLOR,
            ),
            (
                f"Speed: {vessel.speed_mps:.1f} m/s",
                config.STATUS_TEXT_COLOR,
            ),
            (
                f"Distance: {distance_to_destination_m:.1f} m",
                config.STATUS_TEXT_COLOR,
            ),
            (
                f"Elapsed: {metrics.elapsed_time_s:.1f} s",
                config.STATUS_TEXT_COLOR,
            ),
            (
                f"Route: {metrics.distance_traveled_m:.1f} m",
                config.STATUS_TEXT_COLOR,
            ),
            (
                f"Fuel used: {metrics.fuel_used_l:.3f} L",
                config.STATUS_TEXT_COLOR,
            ),
            (
                f"Fuel rate: {current_fuel_burn_rate_lph:.1f} L/h",
                config.STATUS_TEXT_COLOR,
            ),
        ]

        text_x = (
                config.STATUS_PANEL_X_PX
                + config.STATUS_PANEL_PADDING_PX
        )
        text_y = (
                config.STATUS_PANEL_Y_PX
                + config.STATUS_PANEL_PADDING_PX
        )

        for line, color in lines:
            text_surface = self.status_font.render(
                line,
                True,
                color,
            )
            self.screen.blit(text_surface, (text_x, text_y))
            text_y += 26

    def _draw_vessel(self, vessel: Vessel) -> None:
        world_points = vessel.hull_vertices(
            config.VESSEL_LENGTH_M,
            config.VESSEL_BEAM_M,
        )

        screen_points = [
            self._world_to_screen(x_m, y_m)
            for x_m, y_m in world_points
        ]

        pygame.draw.polygon(
            self.screen,
            config.VESSEL_COLOR,
            screen_points,
        )
    def _draw_trail(
        self,
        trail_points: list[tuple[float, float]],
    ) -> None:
        if len(trail_points) < 2:
            return

        screen_points = [
            self._world_to_screen(x_m, y_m)
            for x_m, y_m in trail_points
        ]

        pygame.draw.lines(
            self.screen,
            config.TRAIL_COLOR,
            False,
            screen_points,
            config.TRAIL_WIDTH_PX,
        )

    def _draw_obstacles(
        self,
        obstacles: list[Obstacle],
    ) -> None:
        for obstacle in obstacles:
            screen_position = self._world_to_screen(
                obstacle.x_m,
                obstacle.y_m,
            )
            radius_px = round(
                obstacle.radius_m * config.PIXELS_PER_METER
            )

            pygame.draw.circle(
                self.screen,
                config.OBSTACLE_COLOR,
                screen_position,
                radius_px,
            )
            pygame.draw.circle(
                self.screen,
                config.OBSTACLE_OUTLINE_COLOR,
                screen_position,
                radius_px,
                config.OBSTACLE_OUTLINE_WIDTH_PX,
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

    @staticmethod
    def screen_to_world(
            screen_x_px: float,
            screen_y_px: float,
    ) -> tuple[float, float]:
        world_x_m = (
                            screen_x_px - config.WINDOW_WIDTH / 2
                    ) / config.PIXELS_PER_METER

        world_y_m = (
                            config.WINDOW_HEIGHT / 2 - screen_y_px
                    ) / config.PIXELS_PER_METER

        return world_x_m, world_y_m