import unittest

from vessel import Vessel


class TestVessel(unittest.TestCase):
    def test_vessel_moves_north(self) -> None:
        vessel = Vessel(
            x_m=0.0,
            y_m=0.0,
            heading_deg=0.0,
            speed_mps=5.0,
        )

        vessel.update(dt_s=2.0)

        self.assertAlmostEqual(vessel.x_m, 0.0)
        self.assertAlmostEqual(vessel.y_m, 10.0)

    def test_vessel_moves_east(self) -> None:
        vessel = Vessel(
            x_m=0.0,
            y_m=0.0,
            heading_deg=90.0,
            speed_mps=5.0,
        )

        vessel.update(dt_s=2.0)

        self.assertAlmostEqual(vessel.x_m, 10.0)
        self.assertAlmostEqual(vessel.y_m, 0.0)

    def test_motion_is_independent_of_update_rate(self) -> None:
        single_update_vessel = Vessel(0.0, 0.0, 37.0, 8.0)
        repeated_update_vessel = Vessel(0.0, 0.0, 37.0, 8.0)

        single_update_vessel.update(dt_s=2.0)

        for _ in range(20):
            repeated_update_vessel.update(dt_s=0.1)

        self.assertAlmostEqual(
            single_update_vessel.x_m,
            repeated_update_vessel.x_m,
        )
        self.assertAlmostEqual(
            single_update_vessel.y_m,
            repeated_update_vessel.y_m,
        )

if __name__ == "__main__":
    unittest.main()