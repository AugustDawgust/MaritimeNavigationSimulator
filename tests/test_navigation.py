import unittest

from navigation import (
    calculate_desired_heading,
    calculate_distance,
    calculate_guidance_speed,
    calculate_heading_error,
    has_reached_or_passed_waypoint,
    turn_toward_heading,
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
    def test_waypoint_is_completed_inside_radius(self) -> None:
        self.assertTrue(
            has_reached_or_passed_waypoint(
                current_x_m=19.0,
                current_y_m=10.0,
                waypoint_x_m=20.0,
                waypoint_y_m=10.0,
                onward_target_x_m=100.0,
                onward_target_y_m=10.0,
                waypoint_radius_m=2.0,
            )
        )

    def test_waypoint_is_completed_after_being_passed(
            self,
    ) -> None:
        self.assertTrue(
            has_reached_or_passed_waypoint(
                current_x_m=22.0,
                current_y_m=10.0,
                waypoint_x_m=20.0,
                waypoint_y_m=10.0,
                onward_target_x_m=100.0,
                onward_target_y_m=10.0,
                waypoint_radius_m=1.0,
            )
        )

    def test_waypoint_is_not_completed_before_crossing(
            self,
    ) -> None:
        self.assertFalse(
            has_reached_or_passed_waypoint(
                current_x_m=15.0,
                current_y_m=10.0,
                waypoint_x_m=20.0,
                waypoint_y_m=10.0,
                onward_target_x_m=100.0,
                onward_target_y_m=10.0,
                waypoint_radius_m=1.0,
            )
        )

    def test_sideways_position_does_not_count_as_passed(
            self,
    ) -> None:
        self.assertFalse(
            has_reached_or_passed_waypoint(
                current_x_m=20.0,
                current_y_m=20.0,
                waypoint_x_m=20.0,
                waypoint_y_m=10.0,
                onward_target_x_m=100.0,
                onward_target_y_m=10.0,
                waypoint_radius_m=1.0,
            )
        )
if __name__ == "__main__":
    unittest.main()