import secrets
import pygame

import config
from simulation import Simulation
from renderer import Renderer


def main() -> None:
    pygame.init()

    screen = pygame.display.set_mode(
        (config.WINDOW_WIDTH, config.WINDOW_HEIGHT)
    )
    pygame.display.set_caption(config.WINDOW_TITLE)

    simulation = Simulation()
    simulation.reset_scenario(secrets.randbits(32))
    renderer = Renderer(screen)
    clock = pygame.time.Clock()
    running = True

    invalid_destination_feedback_remaining_s = 0.0

    while running:
        dt_s = clock.tick(config.FPS) / 1000.0
        invalid_destination_feedback_remaining_s = max(
            0.0,
            invalid_destination_feedback_remaining_s - dt_s,
        )

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif (
                    event.type == pygame.KEYDOWN
                    and event.key == pygame.K_r
            ):
                simulation.reset_scenario(
                    secrets.randbits(32)
                )
                invalid_destination_feedback_remaining_s = 0.0
            elif (
                    event.type == pygame.MOUSEBUTTONDOWN
                    and event.button == 1
            ):
                destination_x_m, destination_y_m = (
                    renderer.screen_to_world(
                        event.pos[0],
                        event.pos[1],
                    )
                )

                destination_was_set = simulation.set_destination(
                    destination_x_m,
                    destination_y_m,
                )

                if destination_was_set:
                    invalid_destination_feedback_remaining_s = 0.0
                else:
                    invalid_destination_feedback_remaining_s = (
                        config.INVALID_DESTINATION_FEEDBACK_DURATION_S
                    )

        simulation.update(dt_s)


        renderer.render(
            simulation.vessel,
            simulation.destination_x_m,
            simulation.destination_y_m,
            simulation.arrived,
            simulation.collided,
            simulation.navigation_blocked,
            invalid_destination_feedback_remaining_s > 0.0,
            simulation.trail_points,
            simulation.distance_to_destination_m,
            simulation.obstacles,
            simulation.avoidance_waypoint,
            simulation.metrics,
        )

    pygame.quit()


if __name__ == "__main__":
    main()