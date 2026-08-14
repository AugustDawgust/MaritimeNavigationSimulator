# Engineering Notes

## Version 1.0 Scope

The Autonomous Surface Vessel Navigation Simulator is a two-dimensional engineering simulation of a single autonomous vessel operating among static circular obstacles.

Version 1.0 focuses on:

* Frame-rate-independent vessel motion
* Maritime heading conventions
* Destination-based autonomous navigation
* Turn-rate and speed-change limits
* Reactive local obstacle avoidance
* Safe failure when no local route is available
* Matching rendered and collision hull geometry
* Interactive mission control
* Rolling route-history visualization
* Mission-time, distance, and fuel-use estimates
* Automated unit, integration, and end-to-end validation

The simulator demonstrates navigation logic, simplified maneuverability, software architecture, and engineering verification. It is not a full hydrodynamic, propulsion, or marine-traffic simulation.

## Coordinate System

The simulation uses real-world units and a maritime coordinate convention:

* Position is measured in meters.
* Speed is measured in meters per second.
* The positive x-axis points east.
* The positive y-axis points north.
* Heading is measured clockwise from north.
* A heading of 0 degrees points north.
* A heading of 90 degrees points east.
* A heading of 180 degrees points south.
* A heading of 270 degrees points west.

This differs from Pygame screen coordinates, whose origin is at the upper-left and whose positive y-axis points downward.

## Time Integration

Simulation updates use the elapsed time supplied to each update rather than assuming a fixed frame rate.

For a vessel traveling at speed `v` for elapsed time `dt`:

```text
distance traveled = v × dt
```

Using maritime heading `ψ`, the position update is:

```text
x change = sin(ψ) × v × dt
y change = cos(ψ) × v × dt
```

The trigonometric functions operate on the heading converted from degrees to radians.

This method makes the simulated motion approximately independent of rendering frame rate. Small floating-point differences can still occur when the same total time is divided into different update sizes, especially during state transitions.

## Vessel Geometry and Motion Model

The vessel's dynamic state contains:

* Center position
* Heading
* Forward speed

Motion is kinematic. The model updates pose directly from heading and speed and does not calculate forces, moments, mass, inertia, rudder force, propeller thrust, or hydrodynamic resistance.

The vessel is displayed and collision-tested as an oriented triangle based on the configured vessel length and beam. The triangle contains:

* One bow vertex
* One aft-port vertex
* One aft-starboard vertex

The triangle rotates with vessel heading and translates with vessel position. The vessel does not currently model reverse motion, lateral velocity, sideslip, or a physically derived turning radius.

The current default vessel dimensions are:

```text
length = 12 m
beam = 5 m
```

## Waypoint Navigation

The vessel navigates toward an active target. The active target is either:

* The selected final destination
* The next obstacle-avoidance waypoint

The desired maritime heading from the vessel to a target is calculated from the east and north position differences. The result is normalized to the range from 0 degrees up to, but not including, 360 degrees.

The heading controller calculates the shortest signed angular error between the current heading and desired heading. The error is normalized so the vessel does not make an unnecessarily long rotation across the 0-degree and 360-degree boundary.

* Positive heading error commands a clockwise turn.
* Negative heading error commands a counterclockwise turn.

## Turn-Rate Limiting

The vessel cannot change heading instantaneously. The maximum permitted heading change during one update is:

```text
maximum heading change = maximum turn rate × elapsed time
```

The current default maximum turn rate is:

```text
20 degrees per second
```

If the remaining heading error is smaller than the permitted change, the controller sets the vessel directly to the desired heading without overshooting it.

This model limits angular rate but does not model the hydrodynamic relationship between vessel speed, rudder angle, and turning radius.

## Guidance-Speed Selection

Version 1.0 allows the navigation system to request different target speeds rather than commanding the vessel to remain at one constant speed throughout every maneuver.

The guidance-speed calculation considers navigation conditions such as:

* Heading error
* The need for a significant turn
* Obstacle proximity or an obstructed route
* Whether the vessel is actively maneuvering around an obstacle

The configured normal operating speed is 8 m/s, and the configured minimum guidance speed is 1 m/s. The vessel therefore continues moving forward during normal avoidance turns instead of stopping and rotating in place.

The guidance calculation requests a speed. A separate speed controller limits how quickly the vessel can reach that requested value.

## Acceleration and Deceleration Limiting

Actual vessel speed changes gradually.

When the requested speed is greater than the current speed:

```text
maximum speed increase =
    maximum acceleration × elapsed time
```

When the requested speed is lower than the current speed:

```text
maximum speed decrease =
    maximum deceleration × elapsed time
```

The current default limits are:

```text
maximum acceleration = 2 m/s²
maximum deceleration = 3 m/s²
```

The controller clamps the new speed so it cannot overshoot the requested speed.

Under the default acceleration limit, increasing speed from 1 m/s to 8 m/s requires 3.5 simulated seconds. Under the default deceleration limit, reducing speed from 8 m/s to 1 m/s requires approximately 2.33 simulated seconds.

These limits improve visual and physical plausibility, but they do not represent an engine, propeller, transmission, or force-based surge model.

## Arrival Detection

The vessel is considered to have arrived when its straight-line distance from the final destination is less than or equal to the configured arrival radius.

The current default arrival radius is:

```text
3 m
```

After arrival:

* `arrived` becomes true.
* Vessel speed is set to zero.
* Navigation updates stop.
* Mission metrics stop accumulating.
* The interface displays `ARRIVED`.
* Trail history continues to age and expire.

A valid new destination clears the arrival state and begins a new mission from the vessel's current pose.

## Obstacle Model

The environment contains multiple static circular obstacles.

Each obstacle stores:

* An x-coordinate in meters
* A y-coordinate in meters
* A radius in meters

Obstacle radii must be greater than zero. Attempting to create an obstacle with a zero or negative radius raises a `ValueError`.

The circular obstacle model simplifies geometric calculations and represents stationary hazards rather than specifically modeling buoys, vessels, islands, or shoreline boundaries.

## Triangular-Hull Collision Detection

Physical collision detection uses the vessel's oriented triangular hull and each obstacle's circular boundary.

A vessel-obstacle collision exists if the obstacle circle overlaps or touches the triangle. The geometric test accounts for cases in which:

* The obstacle center lies inside the vessel triangle.
* The obstacle circle overlaps a triangle edge.
* The obstacle circle touches a triangle edge or vertex.

Touching boundaries count as a collision.

The vessel is checked against every obstacle after movement. If a collision is detected:

* `collided` becomes true.
* Vessel speed is set to zero.
* Future navigation and mission-metric updates stop.
* The interface displays `COLLISION`.
* Trail history continues to age and expire.

This collision model matches the displayed hull more closely than the earlier circular approximation.

Collision checks occur at discrete update times. A sufficiently large time step could allow the vessel to pass through an obstacle between checks. Normal real-time update sizes make this less likely, but swept or continuous collision detection would be required to eliminate the possibility.

## Conservative Route-Clearance Geometry

Collision detection and route planning intentionally use different geometric approximations.

Physical collision checks use the triangular hull. Route-obstruction and waypoint-safety calculations use a conservative circular vessel envelope derived from vessel length and beam:

```text
bounding radius =
    √[(vessel length / 2)² + (vessel beam / 2)²]
```

For the default 12 m length and 5 m beam, the radius is 6.5 m.

This circle contains the vessel's modeled extents and simplifies line-segment clearance calculations. It can produce more side clearance than the exact triangular hull requires, but that conservatism is appropriate for local route planning.

## Direct-Route Obstacle Detection

Before following a direct route, the simulator checks whether any obstacle intersects the required clearance area around the finite segment from the vessel to the target.

The test uses the nearest point on the finite route segment rather than an infinite line. Obstacles behind the vessel or beyond the route endpoint therefore do not block that particular segment.

If multiple obstacles block the route, the nearest relevant blocking obstacle is handled first.

After an avoidance waypoint is reached, the route is evaluated again. This allows the local controller to make successive decisions for aligned or staggered obstacles.

## Autonomous Obstacle Avoidance

The simulator uses geometric local waypoint generation around static circular obstacles.

For the nearest blocking obstacle, the route planner generates avoidance candidates on opposite sides of the obstacle. Candidate offset includes:

* Obstacle radius
* Vessel route-clearance allowance
* Configured avoidance clearance
* Additional safety offset

The planner evaluates candidate safety against the complete obstacle collection. A candidate or approach segment is rejected when it violates the configured clearance around an obstacle.

The planner evaluates the preferred side first and uses the opposite side when the preferred candidate is unsafe. Safe waypoints are stored in route order and followed sequentially.

During avoidance, the vessel:

1. Follows the next planned avoidance waypoint.
2. Uses turn-rate and speed-change limits while maneuvering.
3. Advances the route after reaching an intermediate waypoint.
4. Rechecks whether the destination route is clear.
5. Generates or follows additional safe waypoints when needed.
6. Resumes direct destination navigation when the route is clear.

The system has been validated with aligned and staggered multiple-obstacle layouts.

This is a reactive local planner, not a global shortest-path optimizer. It does not exhaustively search all possible routes or guarantee that the selected route is the shortest available route.

## Safe Navigation Failure

If the planner cannot identify a locally safe route, the simulator enters a blocked-navigation state rather than knowingly selecting an unsafe waypoint.

When navigation becomes blocked:

* `navigation_blocked` becomes true.
* No unsafe avoidance target is activated.
* Vessel speed is set to zero.
* The selected final destination remains visible.
* Navigation and mission-metric updates stop.
* The interface displays `NAVIGATION BLOCKED`.
* Trail history continues to age and expire.

This state is distinct from collision. It represents safe failure before the controller intentionally commits to a route it has determined to be unsafe.

A valid new destination clears the blocked state and starts a new mission.

## Destination Validation

Mouse-selected destinations are converted to world coordinates and passed to `Simulation.set_destination()`.

Before the current mission is changed, the simulator checks whether the requested destination satisfies the configured obstacle-clearance rules. A destination inside an obstacle or within its prohibited clearance region is rejected.

If a destination is invalid:

* `set_destination()` returns `False`.
* The existing destination is preserved.
* Current mission and terminal-state data are not reset.
* No new route is planned.

If a destination is valid:

* `set_destination()` accepts the coordinates.
* Mission and terminal-state data are reset as required.
* A new route-planning attempt begins.

Separating validation from state mutation prevents an unsafe click from destroying a valid mission.

## Screen-to-World Coordinate Conversion

Pygame reports mouse positions in screen coordinates:

* The screen origin is at the upper-left corner.
* Positive screen x points right.
* Positive screen y points downward.

The simulation uses world coordinates centered in the window:

* Positive world x points east.
* Positive world y points north.

Screen x is converted to world x using:

```text
world x =
    (screen x - window width / 2)
    / pixels per meter
```

Screen y is converted to world y using:

```text
world y =
    (window height / 2 - screen y)
    / pixels per meter
```

The reversed y subtraction accounts for the opposite vertical-axis directions. This operation is the inverse of the renderer's world-to-screen conversion.

The current default display uses a 1200-by-800-pixel window and a scale of 5 pixels per meter.

## Mission Reset Behavior

`Simulation.set_destination()` starts a new mission without recreating the environment or teleporting the vessel to its configured initial pose.

When a valid new destination is accepted:

* Destination coordinates are replaced.
* `arrived` becomes false.
* `collided` becomes false.
* `navigation_blocked` becomes false.
* Existing avoidance-route waypoints are cleared.
* Vessel speed is restored to the configured initial operating speed.
* Mission elapsed time is reset to zero.
* Route distance is reset to zero.
* Estimated fuel use is reset to zero.
* Vessel position is preserved.
* Vessel heading is preserved.
* Obstacles are preserved.
* Recent trail history is preserved by default.

Preserving position and heading means the new mission begins from the vessel's actual current pose. The turn-rate-limited controller then redirects the vessel toward the new destination.

Restoring the initial operating speed is an intentional mission-control simplification. A more detailed model could instead accelerate from the vessel's terminal-state speed.

## Complete Scenario Reset

A full scenario reset differs from selecting a new destination.

The complete reset can recreate or reposition simulation elements, so it always:

* Clears the existing trail
* Resets trail time to zero
* Starts the trail at the reset vessel position
* Clears route waypoints and terminal states
* Resets mission metrics

Trail clearing is unconditional during a complete scenario reset because preserving the old trail could draw a false line between unrelated vessel positions.

## Rolling Route-Trail History

Version 1.0 stores a rolling time history of sampled vessel positions.

Trail position and recording time are stored in synchronized lists:

* `trail_points[index]` contains an `(x, y)` world position.
* `trail_point_times_s[index]` contains the simulated time at which that point was recorded.

A new point is recorded only after the vessel moves at least the configured minimum spacing from the previous trail point. This avoids adding one point every rendered frame.

At each simulation update:

1. Trail time advances by the elapsed update time.
2. Points older than the configured retention period are removed.
3. A new position is recorded if the vessel has moved far enough.

Trail expiration runs even when navigation or mission metrics have stopped because of arrival, collision, or blocked navigation.

The current defaults are:

```python
TRAIL_RETENTION_TIME_S = 60.0
RESET_TRAIL_ON_DESTINATION_CHANGE = False
```

With the default setting, selecting a new destination preserves recent vessel history. Setting `RESET_TRAIL_ON_DESTINATION_CHANGE` to `True` starts a new trail at every accepted destination change.

A complete scenario reset always clears the trail regardless of this setting.

## Mission Metrics

The simulator tracks:

* Mission elapsed time
* Actual route distance traveled
* Estimated cumulative fuel use
* Current estimated fuel-burn rate

Metrics accumulate only while the mission is active. Arrival, collision, or blocked navigation freezes the mission record.

Selecting a valid new destination creates a new `MissionMetrics` object with zero elapsed time, route distance, and fuel use. This reset is independent of trail preservation: the visual trail can span multiple missions even though the performance metrics describe only the current mission.

Route distance is integrated after movement from the vessel's previous and current positions:

```text
distance traveled =
    √[(x₂ - x₁)² + (y₂ - y₁)²]
```

Each movement interval is added to the cumulative route distance. The result measures the actual simulated path, including turns and avoidance detours, instead of only measuring straight-line distance to the destination.

## Fuel-Consumption Model

The simulator uses a simplified cubic-speed model while the vessel is moving:

```text
fuel burn rate =
    base fuel burn rate
    + speed coefficient × speed³
```

Fuel used during one update is:

```text
fuel used =
    fuel burn rate × elapsed time / 3600
```

The division by 3,600 converts elapsed time from seconds to hours because burn rate is measured in liters per hour.

When vessel speed is zero, the model returns a zero burn rate. The configured base rate therefore represents an underway component rather than engine idling or hotel loads.

Because speed now changes during navigation, fuel burn varies throughout a mission. Lower maneuvering speeds reduce instantaneous estimated burn, and the accumulated estimate reflects the simulated speed history.

The cubic relationship approximates the general increase in propulsion power with speed for a displacement vessel under comparable conditions. The coefficients are not calibrated to a specific engine, hull, propeller, displacement, loading condition, or sea state. The output is intended for relative comparison rather than real-vessel prediction.

## Terminal and Navigation States

The main displayed states are:

* `NAVIGATING`
* `AVOIDING`
* `ARRIVED`
* `COLLISION`
* `NAVIGATION BLOCKED`

Arrival, collision, and blocked navigation are terminal mission states. They stop motion and mission-metric accumulation. `NAVIGATING` and `AVOIDING` describe active guidance behavior.

Terminal states can be cleared by selecting a valid new destination. Trail timestamps continue advancing in every state so old visual history can expire normally.

## Rendering

The physics model stores positions in meters. The renderer converts them to pixels using the configured display scale and reverses the world y-axis for the Pygame display.

The renderer displays:

* Oriented triangular vessel hull
* Selected destination marker
* Static circular obstacles
* Rolling vessel trail
* Planned or active avoidance waypoints
* Current heading
* Current speed
* Distance remaining
* Mission elapsed time
* Actual route distance
* Estimated cumulative fuel use
* Current estimated fuel-burn rate
* Navigation, avoidance, arrival, collision, and blocked-navigation status

Rendering is separated from simulation state so visual scale does not change the physical values used by navigation or mission calculations.

## Automated Validation

The Version 1.0 suite contains 88 automated tests.

The suite covers:

* Vessel motion
* Frame-rate-independent updates
* Maritime heading calculations
* Shortest-direction turning
* Turn-rate limiting
* Guidance-speed calculation
* Acceleration and deceleration limiting
* Arrival behavior
* Obstacle validation
* Triangular-hull collision detection
* Touching and immediately separated collision boundaries
* Collision checks across obstacle collections
* Finite-route obstruction detection
* Nearest blocking-obstacle selection
* Avoidance-waypoint calculations
* Preferred-side and opposite-side selection
* Safe stopping when avoidance candidates are blocked
* Multiple aligned and staggered obstacles
* End-to-end arrival without collision
* Destination validation
* Mission-state resetting
* Vessel position and heading preservation across destination changes
* Obstacle preservation across destination changes
* Speed restoration for a new mission
* Screen-to-world coordinate conversion
* Trail recording and spacing
* Configurable trail preservation and resetting
* Time-based trail expiration
* Unconditional trail clearing during full scenario reset
* Mission elapsed time
* Actual route distance
* Fuel-burn calculations
* Cumulative fuel use
* Terminal-state metric freezing

Run the full suite with:

```bash
python -m unittest discover -s tests
```

Expected result:

```text
Ran 88 tests
OK
```

## Configuration Strategy

Shared engineering and display settings are centralized in `config.py`. Current configuration categories include:

* Initial vessel state
* Vessel dimensions
* Arrival tolerance
* Turn-rate limit
* Guidance-speed limits
* Acceleration and deceleration limits
* Obstacle generation and validation
* Route-clearance geometry
* Avoidance offsets and waypoint radii
* Trail spacing and retention
* Fuel-model coefficients
* Window dimensions and display scale
* Rendering colors and sizes

Centralization makes experiments reproducible, reduces duplicated constants, and allows model behavior to be adjusted without rewriting navigation algorithms.

## Current Limitations

* Kinematic rather than force-based vessel dynamics
* No mass, inertia, thrust, drag, rudder, or propeller model
* No reverse motion, sideslip, or lateral drift
* Fixed maximum turn rate rather than speed-dependent turning behavior
* Acceleration and deceleration limits are prescribed rather than derived from propulsion forces
* Static circular obstacles only
* No moving-vessel tracking or COLREGs behavior
* No wind, waves, current, tide, or water-depth effects
* Perfect knowledge of position, obstacles, and destination
* No sensor range, noise, latency, uncertainty, or failure
* Reactive local avoidance rather than global route optimization
* No guarantee of finding a route through tightly clustered obstacles
* Preferred-side ordering does not guarantee the shortest safe route
* Conservative circular planning clearance can exceed exact triangular-hull requirements
* Discrete rather than continuous collision detection
* Fixed camera, simulation scale, and operating area
* Mouse input is the only interactive destination control
* Selecting a destination after collision clears the terminal state without physically separating the vessel from an obstacle
* New missions restore configured speed rather than modeling restart acceleration from zero
* Simplified and uncalibrated cubic-speed fuel model
* No engine-efficiency, propeller-efficiency, battery, or fuel-tank model
* No idle consumption, hotel loads, or auxiliary electrical demand
* No loading, hull-condition, sea-state, or shallow-water corrections

## Future Work

Potential extensions include:

* Wind, waves, and current disturbances
* Force-based surge, sway, and yaw dynamics
* Speed-dependent rudder response and turning radius
* Moving vessels and COLREGs-aware avoidance
* Simulated radar, AIS, GPS, and sensor uncertainty
* Global path planning and route-length optimization
* Continuous collision detection
* Engine, propeller, battery, and endurance modeling
* Mission-data export and post-mission plots
* Adjustable camera controls and larger operating areas
* Hardware-in-the-loop testing
* Integration with a physical autonomous surface-vessel prototype

These features are intentionally outside Version 1.0. The completed release is a focused, tested demonstration of autonomous navigation, local avoidance, simplified maneuverability, safe failure, and mission-performance tracking.
