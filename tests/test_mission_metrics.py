import unittest

from mission_metrics import (
    MissionMetrics,
    calculate_fuel_burn_rate_lph,
    calculate_fuel_used_l,
)

class TestMissionMetrics(unittest.TestCase):
    def test_stationary_vessel_uses_no_fuel(self) -> None:
        result = calculate_fuel_burn_rate_lph(
            speed_mps=0.0,
            base_fuel_burn_rate_lph=12.0,
            speed_cubed_coefficient=0.08,
        )

        self.assertEqual(result, 0.0)

    def test_calculates_speed_based_fuel_burn_rate(self) -> None:
        result = calculate_fuel_burn_rate_lph(
            speed_mps=5.0,
            base_fuel_burn_rate_lph=12.0,
            speed_cubed_coefficient=0.08,
        )

        self.assertAlmostEqual(result, 22.0)

    def test_converts_hourly_rate_to_fuel_used(self) -> None:
        result = calculate_fuel_used_l(
            fuel_burn_rate_lph=18.0,
            dt_s=600.0,
        )

        self.assertAlmostEqual(result, 3.0)

    def test_rejects_negative_speed(self) -> None:
        with self.assertRaises(ValueError):
            calculate_fuel_burn_rate_lph(
                speed_mps=-1.0,
                base_fuel_burn_rate_lph=12.0,
                speed_cubed_coefficient=0.08,
            )

    def test_rejects_negative_fuel_burn_rate(self) -> None:
        with self.assertRaises(ValueError):
            calculate_fuel_used_l(
                fuel_burn_rate_lph=-1.0,
                dt_s=60.0,
            )

    def test_rejects_negative_elapsed_time(self) -> None:
        with self.assertRaises(ValueError):
            calculate_fuel_used_l(
                fuel_burn_rate_lph=18.0,
                dt_s=-1.0,
            )

    def test_tracker_accumulates_mission_metrics(self) -> None:
        metrics = MissionMetrics(
            base_fuel_burn_rate_lph=12.0,
            speed_cubed_coefficient=0.08,
        )

        metrics.update(
            distance_traveled_m=50.0,
            speed_mps=5.0,
            dt_s=10.0,
        )
        metrics.update(
            distance_traveled_m=25.0,
            speed_mps=5.0,
            dt_s=5.0,
        )

        self.assertAlmostEqual(metrics.elapsed_time_s, 15.0)
        self.assertAlmostEqual(
            metrics.distance_traveled_m,
            75.0,
        )
        self.assertAlmostEqual(
            metrics.fuel_used_l,
            22.0 * 15.0 / 3600.0,
        )

    def test_tracker_records_no_fuel_while_stationary(
        self,
    ) -> None:
        metrics = MissionMetrics(
            base_fuel_burn_rate_lph=12.0,
            speed_cubed_coefficient=0.08,
        )

        metrics.update(
            distance_traveled_m=0.0,
            speed_mps=0.0,
            dt_s=60.0,
        )

        self.assertAlmostEqual(metrics.elapsed_time_s, 60.0)
        self.assertEqual(metrics.distance_traveled_m, 0.0)
        self.assertEqual(metrics.fuel_used_l, 0.0)

    def test_tracker_rejects_negative_distance(self) -> None:
        metrics = MissionMetrics(
            base_fuel_burn_rate_lph=12.0,
            speed_cubed_coefficient=0.08,
        )

        with self.assertRaises(ValueError):
            metrics.update(
                distance_traveled_m=-1.0,
                speed_mps=5.0,
                dt_s=1.0,
            )

    def test_tracker_rejects_invalid_model_settings(
        self,
    ) -> None:
        with self.assertRaises(ValueError):
            MissionMetrics(
                base_fuel_burn_rate_lph=-1.0,
                speed_cubed_coefficient=0.08,
            )

        with self.assertRaises(ValueError):
            MissionMetrics(
                base_fuel_burn_rate_lph=12.0,
                speed_cubed_coefficient=-0.01,
            )


if __name__ == "__main__":
    unittest.main()