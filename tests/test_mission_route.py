import unittest

from mission_route import MissionRoute


class TestMissionRoute(unittest.TestCase):
    def test_starts_at_first_waypoint(self) -> None:
        route = MissionRoute(
            waypoints=[
                (10.0, 20.0),
                (30.0, 40.0),
            ]
        )

        self.assertEqual(route.current_waypoint, (10.0, 20.0))
        self.assertEqual(route.current_waypoint_index, 0)
        self.assertEqual(route.completed_waypoint_count, 0)
        self.assertEqual(route.total_waypoint_count, 2)
        self.assertFalse(route.is_complete)

    def test_advances_to_next_waypoint(self) -> None:
        route = MissionRoute(
            waypoints=[
                (10.0, 20.0),
                (30.0, 40.0),
            ]
        )

        advanced = route.advance()

        self.assertTrue(advanced)
        self.assertEqual(route.current_waypoint, (30.0, 40.0))
        self.assertEqual(route.current_waypoint_index, 1)
        self.assertEqual(route.completed_waypoint_count, 1)
        self.assertFalse(route.is_complete)

    def test_completes_after_final_waypoint(self) -> None:
        route = MissionRoute(
            waypoints=[
                (10.0, 20.0),
                (30.0, 40.0),
            ]
        )

        route.advance()
        route.advance()

        self.assertTrue(route.is_complete)
        self.assertIsNone(route.current_waypoint)
        self.assertEqual(route.completed_waypoint_count, 2)

    def test_cannot_advance_beyond_completion(self) -> None:
        route = MissionRoute(
            waypoints=[(10.0, 20.0)]
        )

        self.assertTrue(route.advance())
        self.assertFalse(route.advance())
        self.assertEqual(route.current_waypoint_index, 1)
        self.assertTrue(route.is_complete)

    def test_rejects_empty_waypoint_list(self) -> None:
        with self.assertRaises(ValueError):
            MissionRoute(waypoints=[])

    def test_copies_waypoint_list(self) -> None:
        waypoints = [(10.0, 20.0)]
        route = MissionRoute(waypoints)

        waypoints.append((30.0, 40.0))

        self.assertEqual(route.total_waypoint_count, 1)

    def test_reset_returns_to_first_waypoint(self) -> None:
        route = MissionRoute(
            waypoints=[
                (10.0, 20.0),
                (30.0, 40.0),
            ]
        )
        route.advance()
        route.advance()

        route.reset()

        self.assertEqual(route.current_waypoint_index, 0)
        self.assertEqual(route.current_waypoint, (10.0, 20.0))
        self.assertFalse(route.is_complete)

    def test_reports_next_waypoint(self) -> None:
        route = MissionRoute(
            waypoints=[
                (10.0, 20.0),
                (30.0, 40.0),
            ]
        )

        self.assertEqual(
            route.next_waypoint,
            (30.0, 40.0),
        )

        route.advance()

        self.assertIsNone(route.next_waypoint)

if __name__ == "__main__":
    unittest.main()