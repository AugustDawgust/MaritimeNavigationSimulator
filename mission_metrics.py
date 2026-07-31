def calculate_fuel_burn_rate_lph(
    speed_mps: float,
    base_fuel_burn_rate_lph: float,
    speed_cubed_coefficient: float,
) -> float:
    if speed_mps < 0.0:
        raise ValueError("Speed cannot be negative.")

    if base_fuel_burn_rate_lph < 0.0:
        raise ValueError("Base fuel burn rate cannot be negative.")

    if speed_cubed_coefficient < 0.0:
        raise ValueError(
            "Speed-cubed coefficient cannot be negative."
        )

    if speed_mps == 0.0:
        return 0.0

    return (
        base_fuel_burn_rate_lph
        + speed_cubed_coefficient * speed_mps**3
    )


def calculate_fuel_used_l(
    fuel_burn_rate_lph: float,
    dt_s: float,
) -> float:
    if fuel_burn_rate_lph < 0.0:
        raise ValueError("Fuel burn rate cannot be negative.")

    if dt_s < 0.0:
        raise ValueError("Elapsed time cannot be negative.")

    return fuel_burn_rate_lph * dt_s / 3600.0

class MissionMetrics:
    def __init__(
        self,
        base_fuel_burn_rate_lph: float,
        speed_cubed_coefficient: float,
    ) -> None:
        if base_fuel_burn_rate_lph < 0.0:
            raise ValueError(
                "Base fuel burn rate cannot be negative."
            )

        if speed_cubed_coefficient < 0.0:
            raise ValueError(
                "Speed-cubed coefficient cannot be negative."
            )

        self.base_fuel_burn_rate_lph = (
            base_fuel_burn_rate_lph
        )
        self.speed_cubed_coefficient = (
            speed_cubed_coefficient
        )

        self.elapsed_time_s = 0.0
        self.distance_traveled_m = 0.0
        self.fuel_used_l = 0.0

    def update(
        self,
        distance_traveled_m: float,
        speed_mps: float,
        dt_s: float,
    ) -> None:
        if distance_traveled_m < 0.0:
            raise ValueError(
                "Distance traveled cannot be negative."
            )

        if dt_s < 0.0:
            raise ValueError("Elapsed time cannot be negative.")

        fuel_burn_rate_lph = calculate_fuel_burn_rate_lph(
            speed_mps,
            self.base_fuel_burn_rate_lph,
            self.speed_cubed_coefficient,
        )

        fuel_used_l = calculate_fuel_used_l(
            fuel_burn_rate_lph,
            dt_s,
        )

        self.elapsed_time_s += dt_s
        self.distance_traveled_m += distance_traveled_m
        self.fuel_used_l += fuel_used_l