import unittest

from navigation import (
    calculate_desired_heading,
    calculate_heading_error,
    turn_toward_heading,
    calculate_distance,
)

class TestNavigation(unittest.TestCase):
    def test_cardinal_headings(self) -> None:
        self.assertAlmostEqual(
            calculate_desired_heading(0.0, 0.0, 0.0, 10.0),
            0.0,
        )
        self.assertAlmostEqual(
            calculate_desired_heading(0.0, 0.0, 10.0, 0.0),
            90.0,
        )
        self.assertAlmostEqual(
            calculate_desired_heading(0.0, 0.0, 0.0, -10.0),
            180.0,
        )
        self.assertAlmostEqual(
            calculate_desired_heading(0.0, 0.0, -10.0, 0.0),
            270.0,
        )

    def test_heading_to_planned_destination(self) -> None:
        heading = calculate_desired_heading(
            current_x_m=0.0,
            current_y_m=0.0,
            destination_x_m=80.0,
            destination_y_m=40.0,
        )

        self.assertAlmostEqual(heading, 63.4349488)

    def test_shortest_heading_error(self) -> None:
        self.assertAlmostEqual(
            calculate_heading_error(350.0, 10.0),
            20.0,
        )
        self.assertAlmostEqual(
            calculate_heading_error(10.0, 350.0),
            -20.0,
        )
        self.assertAlmostEqual(
            calculate_heading_error(45.0, 90.0),
            45.0,
        )
        self.assertAlmostEqual(
            calculate_heading_error(90.0, 45.0),
            -45.0,
        )

    def test_turn_rate_limit(self) -> None:
        self.assertAlmostEqual(
            turn_toward_heading(0.0, 90.0, 20.0, 0.5),
            10.0,
        )
        self.assertAlmostEqual(
            turn_toward_heading(90.0, 0.0, 20.0, 0.5),
            80.0,
        )
        self.assertAlmostEqual(
            turn_toward_heading(350.0, 10.0, 20.0, 0.5),
            0.0,
        )
        self.assertAlmostEqual(
            turn_toward_heading(5.0, 10.0, 20.0, 0.5),
            10.0,
        )

    def test_distance_to_destination(self) -> None:
        self.assertAlmostEqual(
            calculate_distance(0.0, 0.0, 3.0, 4.0),
            5.0,
        )
        self.assertAlmostEqual(
            calculate_distance(10.0, 20.0, 10.0, 20.0),
            0.0,
        )
if __name__ == "__main__":
    unittest.main()