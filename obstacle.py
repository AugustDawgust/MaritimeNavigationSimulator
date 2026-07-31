from dataclasses import dataclass


@dataclass
class Obstacle:
    x_m: float
    y_m: float
    radius_m: float

    def __post_init__(self) -> None:
        if self.radius_m <= 0.0:
            raise ValueError("Obstacle radius must be greater than zero.")