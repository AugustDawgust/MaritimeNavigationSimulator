class MissionRoute:
    def __init__(
        self,
        waypoints: list[tuple[float, float]],
    ) -> None:
        if not waypoints:
            raise ValueError(
                "A mission must contain at least one waypoint."
            )

        self.waypoints = list(waypoints)
        self.current_waypoint_index = 0

    @property
    def current_waypoint(self) -> tuple[float, float] | None:
        if self.is_complete:
            return None

        return self.waypoints[self.current_waypoint_index]

    @property
    def next_waypoint(self) -> tuple[float, float] | None:
        next_waypoint_index = self.current_waypoint_index + 1

        if next_waypoint_index >= self.total_waypoint_count:
            return None

        return self.waypoints[next_waypoint_index]

    @property
    def total_waypoint_count(self) -> int:
        return len(self.waypoints)

    @property
    def completed_waypoint_count(self) -> int:
        return self.current_waypoint_index

    @property
    def is_complete(self) -> bool:
        return (
            self.current_waypoint_index
            >= self.total_waypoint_count
        )

    def advance(self) -> bool:
        if self.is_complete:
            return False

        self.current_waypoint_index += 1
        return True

    def reset(self) -> None:
        self.current_waypoint_index = 0