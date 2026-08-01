import math

import pygame

import config
from mission_metrics import (
    MissionMetrics,
    calculate_fuel_burn_rate_lph,
)
from mission_route import MissionRoute
from obstacle import Obstacle
from vessel import Vessel


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
        trail_points: list[tuple[float, float]],
        distance_to_destination_m: float,
        obstacles: list[Obstacle],
        avoidance_waypoint: tuple[float, float] | None,
        metrics: MissionMetrics,
        route: MissionRoute,
    ) -> None:
        self.screen.fill(config.BACKGROUND_COLOR)

        self._draw_mission_route(route)
        self._draw_trail(trail_points)
        self._draw_obstacles(obstacles)
        self._draw_avoidance_waypoint(avoidance_waypoint)

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
            avoidance_waypoint,
            metrics,
            route,
        )

        pygame.display.flip()

    def _draw_status_panel(
        self,
        vessel: Vessel,
        distance_to_destination_m: float,
        arrived: bool,
        collided: bool,
        avoidance_waypoint: tuple[float, float] | None,
        metrics: MissionMetrics,
        route: MissionRoute,
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

        if collided:
            status_text = "COLLISION"
            status_color = config.STATUS_COLLISION_COLOR
        elif arrived:
            status_text = "ARRIVED"
            status_color = config.STATUS_ARRIVED_COLOR
        elif avoidance_waypoint is not None:
            status_text = "AVOIDING"
            status_color = config.STATUS_AVOIDING_COLOR
        else:
            status_text = "NAVIGATING"
            status_color = config.STATUS_TEXT_COLOR

        if route.is_complete:
            displayed_waypoint_number = route.total_waypoint_count
        else:
            displayed_waypoint_number = (
                route.current_waypoint_index + 1
            )

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
                "Waypoint: "
                f"{displayed_waypoint_number}/"
                f"{route.total_waypoint_count}",
                config.STATUS_TEXT_COLOR,
            ),
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
            text_y += config.STATUS_LINE_SPACING_PX

    def _draw_mission_route(
        self,
        route: MissionRoute,
    ) -> None:
        route_points = [
            (
                config.INITIAL_VESSEL_X_M,
                config.INITIAL_VESSEL_Y_M,
            ),
            *route.waypoints,
        ]

        for waypoint_index in range(
            route.total_waypoint_count
        ):
            start_position = self._world_to_screen(
                *route_points[waypoint_index]
            )
            end_position = self._world_to_screen(
                *route_points[waypoint_index + 1]
            )

            if waypoint_index < route.current_waypoint_index:
                segment_color = (
                    config.MISSION_ROUTE_COMPLETED_COLOR
                )
            elif (
                waypoint_index == route.current_waypoint_index
                and not route.is_complete
            ):
                segment_color = (
                    config.MISSION_ROUTE_ACTIVE_COLOR
                )
            else:
                segment_color = (
                    config.MISSION_ROUTE_UPCOMING_COLOR
                )

            pygame.draw.line(
                self.screen,
                segment_color,
                start_position,
                end_position,
                config.MISSION_ROUTE_WIDTH_PX,
            )

        for waypoint_index, waypoint in enumerate(
            route.waypoints
        ):
            screen_position = self._world_to_screen(*waypoint)

            is_final_waypoint = (
                waypoint_index
                == route.total_waypoint_count - 1
            )

            radius_px = (
                config.DESTINATION_RADIUS_PX
                if is_final_waypoint
                else config.MISSION_WAYPOINT_RADIUS_PX
            )

            if waypoint_index < route.current_waypoint_index:
                pygame.draw.circle(
                    self.screen,
                    config.MISSION_ROUTE_COMPLETED_COLOR,
                    screen_position,
                    radius_px,
                )
            elif (
                waypoint_index == route.current_waypoint_index
                and not route.is_complete
            ):
                pygame.draw.circle(
                    self.screen,
                    config.MISSION_ROUTE_ACTIVE_COLOR,
                    screen_position,
                    radius_px,
                    config.MISSION_WAYPOINT_OUTLINE_WIDTH_PX,
                )
            else:
                pygame.draw.circle(
                    self.screen,
                    config.MISSION_ROUTE_UPCOMING_COLOR,
                    screen_position,
                    radius_px,
                    config.MISSION_WAYPOINT_OUTLINE_WIDTH_PX,
                )

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

    def _draw_avoidance_waypoint(
        self,
        avoidance_waypoint: tuple[float, float] | None,
    ) -> None:
        if avoidance_waypoint is None:
            return

        screen_position = self._world_to_screen(
            avoidance_waypoint[0],
            avoidance_waypoint[1],
        )

        pygame.draw.circle(
            self.screen,
            config.AVOIDANCE_WAYPOINT_COLOR,
            screen_position,
            config.AVOIDANCE_WAYPOINT_SIZE_PX,
        )
        pygame.draw.circle(
            self.screen,
            config.AVOIDANCE_WAYPOINT_OUTLINE_COLOR,
            screen_position,
            config.AVOIDANCE_WAYPOINT_SIZE_PX,
            2,
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
    def _world_to_screen(
        x_m: float,
        y_m: float,
    ) -> tuple[float, float]:
        screen_x = (
            config.WINDOW_WIDTH / 2
            + x_m * config.PIXELS_PER_METER
        )
        screen_y = (
            config.WINDOW_HEIGHT / 2
            - y_m * config.PIXELS_PER_METER
        )
        return screen_x, screen_y