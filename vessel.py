import math
from dataclasses import dataclass


@dataclass
class Vessel:
    x_m: float
    y_m: float
    heading_deg: float
    speed_mps: float

    def update(self, dt_s: float) -> None:
        heading_rad = math.radians(self.heading_deg)

        self.x_m += self.speed_mps * math.sin(heading_rad) * dt_s
        self.y_m += self.speed_mps * math.cos(heading_rad) * dt_s