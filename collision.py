import math

import config
from obstacle import Obstacle
from vessel import Vessel


def _point_to_segment_distance(
        point: tuple[float, float],
        segment_start: tuple[float, float],
        segment_end: tuple[float, float],
) -> float:
    point_x, point_y = point
    start_x, start_y = segment_start
    end_x, end_y = segment_end

    segment_x = end_x - start_x
    segment_y = end_y - start_y
    segment_length_squared = (
        segment_x ** 2 + segment_y ** 2
    )

    if segment_length_squared == 0.0:
        return math.hypot(
            point_x - start_x,
            point_y - start_y,
        )

    projection = (
        (point_x - start_x) * segment_x
        + (point_y - start_y) * segment_y
    ) / segment_length_squared

    projection = max(0.0, min(1.0, projection))

    closest_x = start_x + projection * segment_x
    closest_y = start_y + projection * segment_y

    return math.hypot(
        point_x - closest_x,
        point_y - closest_y,
    )


def _point_is_inside_triangle(
        point: tuple[float, float],
        vertex_a: tuple[float, float],
        vertex_b: tuple[float, float],
        vertex_c: tuple[float, float],
) -> bool:
    def cross_product(
            first: tuple[float, float],
            second: tuple[float, float],
            third: tuple[float, float],
    ) -> float:
        return (
            (second[0] - first[0])
            * (third[1] - first[1])
            - (second[1] - first[1])
            * (third[0] - first[0])
        )

    cross_ab = cross_product(vertex_a, vertex_b, point)
    cross_bc = cross_product(vertex_b, vertex_c, point)
    cross_ca = cross_product(vertex_c, vertex_a, point)

    has_negative = (
        cross_ab < 0.0
        or cross_bc < 0.0
        or cross_ca < 0.0
    )
    has_positive = (
        cross_ab > 0.0
        or cross_bc > 0.0
        or cross_ca > 0.0
    )

    return not (has_negative and has_positive)


def vessel_collides_with_obstacle(
        vessel: Vessel,
        vessel_bounding_radius_m: float,
        obstacle: Obstacle,
) -> bool:
    center_distance_m = math.hypot(
        obstacle.x_m - vessel.x_m,
        obstacle.y_m - vessel.y_m,
    )

    if (
            center_distance_m
            > vessel_bounding_radius_m + obstacle.radius_m
    ):
        return False

    hull_vertices = vessel.hull_vertices(
        config.VESSEL_LENGTH_M,
        config.VESSEL_BEAM_M,
    )
    obstacle_center = (obstacle.x_m, obstacle.y_m)

    if _point_is_inside_triangle(
            obstacle_center,
            *hull_vertices,
    ):
        return True

    hull_edges = (
        (hull_vertices[0], hull_vertices[1]),
        (hull_vertices[1], hull_vertices[2]),
        (hull_vertices[2], hull_vertices[0]),
    )

    return any(
        _point_to_segment_distance(
            obstacle_center,
            edge_start,
            edge_end,
        )
        <= obstacle.radius_m
        for edge_start, edge_end in hull_edges
    )


def vessel_collides_with_any_obstacle(
        vessel: Vessel,
        vessel_bounding_radius_m: float,
        obstacles: list[Obstacle],
) -> bool:
    return any(
        vessel_collides_with_obstacle(
            vessel,
            vessel_bounding_radius_m,
            obstacle,
        )
        for obstacle in obstacles
    )