import math
from dataclasses import dataclass


@dataclass
class Vessel:
    x_m: float
    y_m: float
    heading_deg: float
    speed_mps: float

    def hull_vertices(
            self,
            length_m: float,
            beam_m: float,
    ) -> tuple[
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
    ]:
        heading_rad = math.radians(self.heading_deg)

        forward_x = math.sin(heading_rad)
        forward_y = math.cos(heading_rad)
        right_x = math.cos(heading_rad)
        right_y = -math.sin(heading_rad)

        half_length_m = length_m / 2.0
        half_beam_m = beam_m / 2.0

        bow = (
            self.x_m + forward_x * half_length_m,
            self.y_m + forward_y * half_length_m,
        )
        stern_center = (
            self.x_m - forward_x * half_length_m,
            self.y_m - forward_y * half_length_m,
        )
        port_stern = (
            stern_center[0] - right_x * half_beam_m,
            stern_center[1] - right_y * half_beam_m,
        )
        starboard_stern = (
            stern_center[0] + right_x * half_beam_m,
            stern_center[1] + right_y * half_beam_m,
        )

        return bow, port_stern, starboard_stern
    def update(self, dt_s: float) -> None:
        heading_rad = math.radians(self.heading_deg)

        self.x_m += self.speed_mps * math.sin(heading_rad) * dt_s
        self.y_m += self.speed_mps * math.cos(heading_rad) * dt_s